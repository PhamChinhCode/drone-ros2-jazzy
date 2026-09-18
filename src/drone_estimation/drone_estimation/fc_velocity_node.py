"""fc_velocity_node - dua van toc FC (ODOMETRY) vao EKF, bo mau khong hop le.

FC bao van toc khong hop le bang covariance 1e6 (MAVROS bo truong quality - GIAO_UOC_FC_ROS2.md
11.2). Quy tac 4.2a: BO HAN mau, khong dua vao EKF voi trong so thap - robot_localization khong
tu loc duoc theo covariance nen node nay loc truoc (chot 11.3 P3).

Tach hai topic vi vx/vy (flow) va vz (do cao) hop le doc lap: nam sat dat flow tat nhung vz van
dung. Van toc giu trong khung than base_link (FLU) nhu MAVROS phat.

vz CHI chuyen khi /range/vertical vua hop le: FC bo laser (nghieng > 25 do, ngoai tam, bi che) thi
do cao FC chi con baro + gia toc va troi toi ~0,9 m/s, nhung van bao sigma ~0,06 m/s - EKF Pi tin
theo va z lao xuong -1,9 m khi cam tay nghieng (do 2026-09-18). range_vertical_node dung cung
nguong 25 do / tre 3 do voi FC.

Chua arm thi phat van toc (0, 0, 0) len /zupt/velocity (zero-velocity update): nam dat thuong
khong co nguon van toc nao - flow FC tat duoi 0,2 m, flow camera/vz FC can laser hop le, tag hay
ngoai tam camera chech 20 do - va EKF chi con tich phan IMU, troi toi hang tram met (do 09-18).
Arm roi thi tat de flow va tag lam viec.
"""

import math

import rclpy
from geometry_msgs.msg import TwistWithCovarianceStamped
from mavros_msgs.msg import State
from nav_msgs.msg import Odometry
from rclpy.experimental import EventsExecutor
from rclpy.node import Node
from sensor_msgs.msg import Range

from drone_estimation.estimation_math import fc_velocity_validity, zupt_active
from drone_estimation.qos import EVENT_QOS, SENSOR_QOS

# sigma 0,05 m/s: dung yen that tren mat dat, nhung de tag van keo duoc vi tri.
ZUPT_VARIANCE = 0.0025


class FcVelocityNode(Node):

    def __init__(self):
        super().__init__('fc_velocity_node')
        self.xy_valid = None
        self.z_valid = None
        # Laser 20 Hz: 0,3 s cho phep lo vai mau.
        self.declare_parameter('range_max_age_s', 0.3)
        # Laser vua hop le lai thi FC CHUA sua xong vz: FC neo lai do cao sau 0,5 s khong dung laser
        # (EST_RANGE_REANCHOR_MS) - trong quang do vz cua FC van la gia tri da troi.
        self.declare_parameter('range_settle_s', 0.6)
        self.range_ok_s = None      # lan gan nhat /range/vertical hop le
        self.range_ok_since_s = None  # bat dau chuoi mau hop le lien tuc hien tai
        self.create_subscription(Range, '/range/vertical', self.on_range, SENSOR_QOS)
        self.create_subscription(Odometry, '/mavros/odometry/in', self.on_odom, SENSOR_QOS)

        # /mavros/state theo heartbeat FC ~1 Hz: 2,5 s cho phep lo 1-2 goi.
        self.declare_parameter('state_max_age_s', 2.5)
        self.fc_state = None
        self.fc_state_s = None
        self.zupt_on = None
        self.create_subscription(State, '/mavros/state', self.on_fc_state, EVENT_QOS)
        self.pub_zupt = self.create_publisher(
            TwistWithCovarianceStamped, '/zupt/velocity', SENSOR_QOS)
        self.create_timer(0.1, self.publish_zupt)
        self.pub_xy = self.create_publisher(
            TwistWithCovarianceStamped, '/fc/velocity_xy', SENSOR_QOS)
        self.pub_z = self.create_publisher(
            TwistWithCovarianceStamped, '/fc/velocity_z', SENSOR_QOS)

    def now_s(self):
        return self.get_clock().now().nanoseconds / 1e9

    def on_fc_state(self, msg):
        self.fc_state = msg
        self.fc_state_s = self.now_s()

    def publish_zupt(self):
        age = None if self.fc_state_s is None else self.now_s() - self.fc_state_s
        on = zupt_active(age, self.fc_state is not None and self.fc_state.connected,
                         self.fc_state is not None and self.fc_state.armed,
                         self.get_parameter('state_max_age_s').value)
        if on != self.zupt_on:
            self.get_logger().info(f'van toc 0 khi nam dat (ZUPT) {"BAT" if on else "TAT"}')
            self.zupt_on = on
        if not on:
            return
        out = TwistWithCovarianceStamped()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = 'base_link'
        for i in (0, 7, 14):
            out.twist.covariance[i] = ZUPT_VARIANCE
        self.pub_zupt.publish(out)

    def laser_fresh(self, now):
        return (self.range_ok_s is not None and
                now - self.range_ok_s <= self.get_parameter('range_max_age_s').value)

    def on_range(self, msg):
        now = self.now_s()
        if math.isfinite(msg.range) and msg.min_range <= msg.range <= msg.max_range:
            if not self.laser_fresh(now):
                self.range_ok_since_s = now
            self.range_ok_s = now

    def on_odom(self, msg):
        cov = msg.twist.covariance
        xy_ok, z_ok = fc_velocity_validity(cov)
        now = self.now_s()
        laser_ok = (self.laser_fresh(now) and
                    now - self.range_ok_since_s >= self.get_parameter('range_settle_s').value)
        z_ok = z_ok and laser_ok
        if z_ok != self.z_valid:
            state = 'HOP LE' if z_ok else 'KHONG dung (FC mat laser hoac khong hop le) - bo mau'
            self.get_logger().info(f'van toc doc FC {state}')
            self.z_valid = z_ok
        if xy_ok != self.xy_valid:
            state = 'HOP LE' if xy_ok else 'KHONG hop le - bo mau'
            self.get_logger().info(f'van toc ngang FC {state}')
            self.xy_valid = xy_ok
        if xy_ok:
            out = self.make_msg(msg)
            out.twist.twist.linear.x = msg.twist.twist.linear.x
            out.twist.twist.linear.y = msg.twist.twist.linear.y
            out.twist.covariance[0] = cov[0]
            out.twist.covariance[1] = cov[1]
            out.twist.covariance[6] = cov[6]
            out.twist.covariance[7] = cov[7]
            self.pub_xy.publish(out)
        if z_ok:
            out = self.make_msg(msg)
            out.twist.twist.linear.z = msg.twist.twist.linear.z
            out.twist.covariance[14] = cov[14]
            self.pub_z.publish(out)

    @staticmethod
    def make_msg(odom):
        out = TwistWithCovarianceStamped()
        out.header.stamp = odom.header.stamp
        out.header.frame_id = odom.child_frame_id or 'base_link'
        return out


def main(args=None):
    rclpy.init(args=args)
    node = FcVelocityNode()
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
