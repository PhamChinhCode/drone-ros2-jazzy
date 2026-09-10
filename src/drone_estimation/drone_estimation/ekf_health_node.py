"""ekf_health_node - sinh co healthy nhi phan ma robot_localization khong co san.

failsafe_monitor_node can mot cau tra loi co/khong ("EKF con dang tin duoc khong"),
trong khi robot_localization chi xuat covariance va /diagnostics. Node nay lam cau noi.
"""

import rclpy
from diagnostic_msgs.msg import DiagnosticArray
from nav_msgs.msg import Odometry
from rclpy.node import Node

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
        self.over_threshold_since = None

        self.create_subscription(Odometry, '/odometry/filtered', self.on_odom, SENSOR_QOS)
        self.create_subscription(DiagnosticArray, '/diagnostics', self.on_diagnostics, EVENT_QOS)
        self.pub_health = self.create_publisher(EkfHealth, '/ekf/health', EVENT_QOS)

        period = 1.0 / self.get_parameter('publish_rate_hz').value
        self.create_timer(period, self.publish_health)

    def on_odom(self, msg):
        self.last_odom = msg
        self.last_odom_time = self.get_clock().now()

    def on_diagnostics(self, msg):
        """TODO: doc trang thai do robot_localization tu bao (neu co) de bo sung ly do."""
        del msg

    def publish_health(self):
        """TODO: healthy=false khi (a) odom im qua odom_timeout_s, (b) phuong sai vi tri vuot
        max_pos_variance lien tuc qua unhealthy_after_s, hoac (c) gap nan/inf. Dien reason."""


def main(args=None):
    rclpy.init(args=args)
    node = EkfHealthNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
