"""marker_pose_republisher_node - dua pose marker vao khung odom cho EKF.

apriltag_ros cho pose marker trong khung quang hoc cua camera. robot_localization can
PoseWithCovarianceStamped trong khung odom va KHONG tu lam buoc chuyen nay.

Diem de sai nhat: nham chieu transform camera <-> body. Trieu chung dac trung la
"thay dung marker nhung bay lech tam".
"""

import rclpy
from apriltag_msgs.msg import AprilTagDetectionArray
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener

from drone_estimation.qos import SENSOR_QOS


class MarkerPoseRepublisherNode(Node):

    def __init__(self):
        super().__init__('marker_pose_republisher_node')

        # Marker cho vi tri TUYET DOI nen covariance thap (tin nhieu) - nguoc voi optical flow.
        self.declare_parameter('pos_covariance', 0.01)      # m^2
        self.declare_parameter('target_frame', 'odom')
        # Toa do da biet cua tung bai dap trong khung map, dang phang [id, x, y, z, ...].
        self.declare_parameter('known_tags', [0.0])

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.create_subscription(
            AprilTagDetectionArray, '/apriltag/detections', self.on_detections, SENSOR_QOS)
        self.pub_pose = self.create_publisher(
            PoseWithCovarianceStamped, '/marker/pose_odom', SENSOR_QOS)

    def on_detections(self, msg):
        """TODO: voi moi detection - tra cuu toa do tag trong known_tags, lookup transform
        camera_optical_frame -> target_frame, suy ra vi tri drone trong odom, dien covariance
        duong cheo tu pos_covariance roi publish. Bo qua tag khong co trong known_tags."""
        del msg


def main(args=None):
    rclpy.init(args=args)
    node = MarkerPoseRepublisherNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
