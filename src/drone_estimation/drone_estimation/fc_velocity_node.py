"""fc_velocity_node - dua van toc FC (ODOMETRY) vao EKF, bo mau khong hop le.

FC bao van toc khong hop le bang covariance 1e6 (MAVROS bo truong quality - GIAO_UOC_FC_ROS2.md
11.2). Quy tac 4.2a: BO HAN mau, khong dua vao EKF voi trong so thap - robot_localization khong
tu loc duoc theo covariance nen node nay loc truoc (chot 11.3 P3).

Tach hai topic vi vx/vy (flow) va vz (do cao) hop le doc lap: nam sat dat flow tat nhung vz van
dung. Van toc giu trong khung than base_link (FLU) nhu MAVROS phat.
"""

import rclpy
from geometry_msgs.msg import TwistWithCovarianceStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node

from drone_estimation.estimation_math import fc_velocity_validity
from drone_estimation.qos import SENSOR_QOS


class FcVelocityNode(Node):

    def __init__(self):
        super().__init__('fc_velocity_node')
        self.xy_valid = None
        self.create_subscription(Odometry, '/mavros/odometry/in', self.on_odom, SENSOR_QOS)
        self.pub_xy = self.create_publisher(
            TwistWithCovarianceStamped, '/fc/velocity_xy', SENSOR_QOS)
        self.pub_z = self.create_publisher(
            TwistWithCovarianceStamped, '/fc/velocity_z', SENSOR_QOS)

    def on_odom(self, msg):
        cov = msg.twist.covariance
        xy_ok, z_ok = fc_velocity_validity(cov)
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
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
