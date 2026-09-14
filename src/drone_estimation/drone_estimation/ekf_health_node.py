"""ekf_health_node - sinh co healthy nhi phan ma robot_localization khong co san.

failsafe_monitor_node can mot cau tra loi co/khong ("EKF con dang tin duoc khong"),
trong khi robot_localization chi xuat covariance va /diagnostics. Node nay lam cau noi.
"""

import math

import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
from nav_msgs.msg import Odometry
from rclpy.experimental import EventsExecutor
from rclpy.node import Node

from drone_estimation.estimation_math import HealthMonitor
from drone_estimation.qos import EVENT_QOS, SENSOR_QOS
from drone_interfaces.msg import EkfHealth


class EkfHealthNode(Node):

    def __init__(self):
        super().__init__('ekf_health_node')

        self.declare_parameter('max_pos_variance', 2.0)     # m^2
        self.declare_parameter('unhealthy_after_s', 2.0)    # vuot nguong lien tuc bao lau thi bao xau
        self.declare_parameter('odom_timeout_s', 1.0)       # EKF im lang bao lau thi coi la hong
        self.declare_parameter('publish_rate_hz', 5.0)

        self.last_odom = None
        self.last_odom_time = None
        self.diag_problem = ''
        self.monitor = HealthMonitor(self.get_parameter('max_pos_variance').value,
                                     self.get_parameter('unhealthy_after_s').value,
                                     self.get_parameter('odom_timeout_s').value)
        self.last_healthy = None

        self.create_subscription(Odometry, '/odometry/filtered', self.on_odom, SENSOR_QOS)
        self.create_subscription(DiagnosticArray, '/diagnostics', self.on_diagnostics, EVENT_QOS)
        self.pub_health = self.create_publisher(EkfHealth, '/ekf/health', EVENT_QOS)

        period = 1.0 / self.get_parameter('publish_rate_hz').value
        self.create_timer(period, self.publish_health)

    def on_odom(self, msg):
        self.last_odom = msg
        self.last_odom_time = self.get_clock().now()

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
