"""optical_flow_node - do van toc troi ngang khi khong thay marker.

Node rui ro cao nhat cua lop cam nhan: van toc nhieu day thang vao EKF se gay troi vi tri.
Ba nguyen tac bat buoc (muc 1.4 tai lieu huong dan):
  1. covariance ti le nghich voi so dac trung bam duoc - do la cach bao chat luong thap cho EKF;
  2. duoi nguong dac trung toi thieu thi KHONG publish gi ca;
  3. phat hien lai dac trung dinh ky de khong bam mai diem da troi khoi khung hinh.
"""

import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import TwistWithCovarianceStamped
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import Float64

from drone_perception.optical_flow_estimator import OpticalFlowEstimator
from drone_perception.qos import SENSOR_QOS


class OpticalFlowNode(Node):

    def __init__(self):
        super().__init__('optical_flow_node')

        self.declare_parameter('max_corners', 100)
        self.declare_parameter('quality_level', 0.3)
        self.declare_parameter('min_distance', 7)
        self.declare_parameter('min_tracked_features', 8)
        self.declare_parameter('refresh_interval_s', 1.5)
        self.declare_parameter('process_every_n_frames', 2)
        self.declare_parameter('base_covariance', 0.05)

        self.bridge = CvBridge()
        self.estimator = None          # tao sau khi co focal length tu camera_info
        self.altitude_m = None         # chua co do cao thi chua quy doi duoc pixel -> met
        self.last_stamp = None
        self.frame_counter = 0

        self.create_subscription(CameraInfo, '/camera/camera_info', self.on_camera_info, SENSOR_QOS)
        self.create_subscription(Image, '/camera/image_raw', self.on_image, SENSOR_QOS)
        self.create_subscription(Float64, '/mavros/global_position/rel_alt', self.on_altitude, SENSOR_QOS)

        self.pub_velocity = self.create_publisher(
            TwistWithCovarianceStamped, '/optical_flow/velocity', SENSOR_QOS)

    def on_camera_info(self, msg):
        """Lay focal length tu ma tran noi tai da hieu chinh - khong bao gio do tay."""
        if self.estimator is None:
            self.estimator = OpticalFlowEstimator(
                focal_px=msg.k[0],
                max_corners=self.get_parameter('max_corners').value,
                quality_level=self.get_parameter('quality_level').value,
                min_distance=self.get_parameter('min_distance').value,
                min_tracked_features=self.get_parameter('min_tracked_features').value)
            self.get_logger().info(f'Nhan focal length tu camera_info: {msg.k[0]:.1f} px')

    def on_altitude(self, msg):
        self.altitude_m = msg.data

    def on_image(self, msg):
        """TODO: cvtColor sang xam, bo bot khung theo process_every_n_frames, goi estimator,
        dat covariance ti le nghich voi n_tracked, publish TwistWithCovarianceStamped."""
        del msg

    def publish_velocity(self, vel_xy, n_tracked, stamp):
        """TODO: dien Twist + covariance = base_covariance * min_tracked / max(n_tracked, 1)."""
        del vel_xy, n_tracked, stamp


def main(args=None):
    rclpy.init(args=args)
    node = OpticalFlowNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
