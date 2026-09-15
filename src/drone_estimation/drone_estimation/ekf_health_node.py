"""ekf_health_node - sinh co healthy nhi phan ma robot_localization khong co san.

failsafe_monitor_node can mot cau tra loi co/khong ("EKF con dang tin duoc khong"),
trong khi robot_localization chi xuat covariance va /diagnostics. Node nay lam cau noi.

Kiem them viec keo EKF ve marker (/set_pose) khi EKF lech xa ma van thay tag - xem
MarkerResetPolicy.
"""

import math

import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from rclpy.experimental import EventsExecutor
from rclpy.node import Node
from rclpy.time import Time

from drone_estimation.estimation_math import HealthMonitor, MarkerResetPolicy
from drone_estimation.qos import EVENT_QOS, SENSOR_QOS
from drone_interfaces.msg import EkfHealth

SET_POSE_ANGLE_VARIANCE = 0.01   # rad^2


class EkfHealthNode(Node):

    def __init__(self):
        super().__init__('ekf_health_node')

        self.declare_parameter('max_pos_variance', 2.0)     # m^2
        self.declare_parameter('unhealthy_after_s', 2.0)    # vuot nguong lien tuc bao lau thi bao xau
        self.declare_parameter('odom_timeout_s', 1.0)       # EKF im lang bao lau thi coi la hong
        self.declare_parameter('publish_rate_hz', 5.0)
        # Ep EKF ve marker: lech qua reset_max_error_m lien tuc reset_after_s, hoac khong healthy.
        self.declare_parameter('reset_max_error_m', 1.0)
        self.declare_parameter('reset_after_s', 1.0)
        # Pose marker tre 250-430 ms luc day tai (do 09-14) + chu ky timer 0,2 s.
        self.declare_parameter('reset_marker_max_age_s', 1.0)
        self.declare_parameter('reset_cooldown_s', 2.0)
        # Coi la da neo khi EKF cach pose marker khong qua nguong nay. Mac dinh bang
        # pose0_rejection_threshold cua ekf.yaml: trong nguong do la EKF DA fuse marker.
        self.declare_parameter('anchor_tol_m', 2.0)

        self.last_odom = None
        self.last_odom_time = None
        self.diag_problem = ''
        self.monitor = HealthMonitor(self.get_parameter('max_pos_variance').value,
                                     self.get_parameter('unhealthy_after_s').value,
                                     self.get_parameter('odom_timeout_s').value)
        self.last_healthy = None
        self.last_marker = None
        self.anchored = False
        self.reset_policy = MarkerResetPolicy(self.get_parameter('reset_max_error_m').value,
                                              self.get_parameter('reset_after_s').value,
                                              self.get_parameter('reset_marker_max_age_s').value,
                                              self.get_parameter('reset_cooldown_s').value)

        self.create_subscription(Odometry, '/odometry/filtered', self.on_odom, SENSOR_QOS)
        self.create_subscription(DiagnosticArray, '/diagnostics', self.on_diagnostics, EVENT_QOS)
        self.create_subscription(
            PoseWithCovarianceStamped, '/marker/pose_odom', self.on_marker, SENSOR_QOS)
        self.pub_health = self.create_publisher(EkfHealth, '/ekf/health', EVENT_QOS)
        # Topic set_pose cua robot_localization/ekf_node: dat lai trang thai ve pose nay.
        self.pub_set_pose = self.create_publisher(PoseWithCovarianceStamped, '/set_pose', EVENT_QOS)

        period = 1.0 / self.get_parameter('publish_rate_hz').value
        self.create_timer(period, self.publish_health)

    def on_odom(self, msg):
        self.last_odom = msg
        self.last_odom_time = self.get_clock().now()

    def on_marker(self, msg):
        self.last_marker = msg

    def on_diagnostics(self, msg):
        """Ghi canh bao/loi robot_localization tu bao de bo sung ly do (khong doi healthy)."""
        # Bo muc "topic status" (bo dem tan so cua diagnostic_updater): bao "No events recorded"
        # ca khi /odometry/filtered dang ra deu 30 Hz - do 09-14.
        problems = [f'{s.name}: {s.message}' for s in msg.status
                    if 'ekf' in s.name.lower() and 'topic status' not in s.name
                    and s.level >= DiagnosticStatus.WARN]
        self.diag_problem = '; '.join(problems)

    def publish_health(self):
        """healthy=false khi odom im qua odom_timeout_s, phuong sai vi tri vuot max_pos_variance
        lien tuc qua unhealthy_after_s, hoac gap nan/inf."""
        now = self.get_clock().now()
        msg = EkfHealth()
        msg.stamp = now.to_msg()
        variance = (0.0, 0.0, 0.0)
        finite = True
        if self.last_odom is not None:
            c = self.last_odom.pose.covariance
            variance = (c[0], c[7], c[14])
            p = self.last_odom.pose.pose.position
            finite = all(math.isfinite(v) for v in (p.x, p.y, p.z, *variance))
            msg.pos_variance = [float(v) for v in variance]
        stamp_s = None if self.last_odom_time is None else self.last_odom_time.nanoseconds / 1e9
        msg.healthy, msg.reason = self.monitor.evaluate(
            now.nanoseconds / 1e9, stamp_s, variance, finite)
        msg.anchored = self.update_anchored()
        if self.diag_problem:
            msg.reason = f'{msg.reason} | {self.diag_problem}' if msg.reason else self.diag_problem
        # Chi log khi co doi trang thai - reason chua so thay doi moi chu ky.
        if msg.healthy != self.last_healthy:
            if msg.healthy:
                self.get_logger().info(f'EKF healthy {msg.reason}')
            else:
                self.get_logger().warning(f'EKF KHONG healthy: {msg.reason}')
            self.last_healthy = msg.healthy
        self.pub_health.publish(msg)
        self.check_marker_reset(now, stamp_s, msg.healthy)

    def update_anchored(self):
        """Latching: odom da neo theo bang tag chua.

        Neo = EKF da fuse it nhat mot pose marker. Khong quan sat truc tiep duoc tu ngoai
        robot_localization, nen suy ra: co pose marker VA vi tri EKF cach no khong qua
        anchor_tol_m (= pose0_rejection_threshold) => pose do da duoc nhan chu khong bi loai.
        Ep /set_pose ve marker cung la neo, dat co ngay tai cho do.
        """
        if self.anchored or self.last_marker is None or self.last_odom is None:
            return self.anchored
        a = self.last_odom.pose.pose.position
        b = self.last_marker.pose.pose.position
        if math.dist((a.x, a.y, a.z), (b.x, b.y, b.z)) <= \
                self.get_parameter('anchor_tol_m').value:
            self.anchored = True
            self.get_logger().info('odom DA NEO theo bang tag - vi tri tuyet doi dung duoc')
        return self.anchored

    def check_marker_reset(self, now, odom_stamp_s, healthy):
        """Pose marker moi ma EKF lech/khong healthy -> gui pose marker len /set_pose."""
        now_s = now.nanoseconds / 1e9
        alive = (odom_stamp_s is not None
                 and now_s - odom_stamp_s <= self.get_parameter('odom_timeout_s').value)
        marker = self.last_marker
        if not alive or marker is None:
            self.reset_policy.evaluate(now_s, alive, healthy, (0, 0, 0), (0, 0, 0), None)
            return
        p, m = self.last_odom.pose.pose.position, marker.pose.pose.position
        # Tuoi marker tinh theo stamp anh (tre 250-430 ms luc day tai) - cung dong ho node.
        reason = self.reset_policy.evaluate(
            now_s, alive, healthy, (p.x, p.y, p.z), (m.x, m.y, m.z),
            Time.from_msg(marker.header.stamp).nanoseconds / 1e9)
        if not reason:
            return
        out = PoseWithCovarianceStamped()
        out.header.frame_id = marker.header.frame_id
        out.header.stamp = now.to_msg()
        out.pose.pose = marker.pose.pose
        # Truc goc cua pose marker mang 1e6 (khong do) - dat lai vua phai, IMU keo huong ngay sau.
        cov = list(marker.pose.covariance)
        for i in range(3, 6):
            cov[i * 7] = SET_POSE_ANGLE_VARIANCE
        out.pose.covariance = cov
        self.pub_set_pose.publish(out)
        # Ep EKF ve marker la dinh nghia cua "neo theo bang tag".
        self.anchored = True
        self.get_logger().warning(
            f'Ep EKF ve marker ({reason}): EKF ({p.x:.2f}, {p.y:.2f}, {p.z:.2f}) -> '
            f'marker ({m.x:.2f}, {m.y:.2f}, {m.z:.2f})')


def main(args=None):
    rclpy.init(args=args)
    node = EkfHealthNode()
    # EventsExecutor: tren Pi 4 executor mac dinh cua rclpy ton phan lon CPU de dung lai wait-set
    # moi lan thuc day (do 09-14: mission_manager_node 45-50 % -> 13,5 %).
    executor = EventsExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
