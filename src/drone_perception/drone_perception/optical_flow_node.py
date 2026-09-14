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
from rclpy.experimental import EventsExecutor
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image, Range

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
        self.declare_parameter('max_lk_error', 20.0)

        self.bridge = CvBridge()
        self.estimator = None          # tao sau khi co focal length tu camera_info
        self.altitude_m = None         # chua co do cao thi chua quy doi duoc pixel -> met
        self.last_stamp = None
        self.last_refresh = None
        self.frame_counter = 0

        self.create_subscription(CameraInfo, '/camera/camera_info', self.on_camera_info, SENSOR_QOS)
        self.create_subscription(Image, '/camera/image_raw', self.on_image, SENSOR_QOS)
        # Do cao lay tu laser MTF01P (20 Hz, ngang tam camera) thay /mavros/global_position/rel_alt:
        # relative_alt chua duoc giao uoc kiem (12.A1) va de FC giam GLOBAL_POSITION_INT (11.1 #14).
        self.create_subscription(Range, '/mavros/mtf01p', self.on_altitude, SENSOR_QOS)

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
                min_tracked_features=self.get_parameter('min_tracked_features').value,
                max_lk_error=self.get_parameter('max_lk_error').value)
            self.get_logger().info(f'Nhan focal length tu camera_info: {msg.k[0]:.1f} px')

    def on_altitude(self, msg):
        # Ngoai [min_range, max_range] la khong do duoc -> khong quy doi, khong publish.
        ok = msg.min_range <= msg.range <= msg.max_range
        self.altitude_m = msg.range if ok else None

    def on_image(self, msg):
        # Chua co camera_info (thieu focal length) hoac chua co do cao thi khong quy doi
        # duoc pixel -> met. Im lang cho, khong doan bua.
        if self.estimator is None or self.altitude_m is None:
            return

        self.frame_counter += 1
        moi_n = int(self.get_parameter('process_every_n_frames').value)
        if moi_n > 1 and self.frame_counter % moi_n:
            return

        gray = self.bridge.imgmsg_to_cv2(msg, desired_encoding='mono8')
        stamp = Time.from_msg(msg.header.stamp)

        # Nguyen tac 3: phat hien lai dac trung dinh ky, khong bam mai nhung diem
        # da troi ra ria hoac da bi che khuat.
        if self.last_refresh is None:
            self.last_refresh = stamp
        elif (stamp - self.last_refresh).nanoseconds * 1e-9 >= \
                self.get_parameter('refresh_interval_s').value:
            self.estimator.reset()
            self.last_refresh = stamp

        dt = 0.0 if self.last_stamp is None else (stamp - self.last_stamp).nanoseconds * 1e-9
        self.last_stamp = stamp

        vel_xy, n_tracked = self.estimator.process(gray, dt, self.altitude_m)
        if vel_xy is None:
            return
        self.publish_velocity(vel_xy, n_tracked, msg.header.stamp)

    def publish_velocity(self, vel_xy, n_tracked, stamp):
        msg = TwistWithCovarianceStamped()
        msg.header.stamp = stamp
        # Phat trong khung QUANG HOC; robot_localization tu xoay ve base_link qua TF
        # (chuoi base_link -> camera_link -> camera_optical_frame trong estimation.launch.py).
        msg.header.frame_id = 'camera_optical_frame'

        # DAU: estimator tra ve van toc cua CANH VAT tren anh. Camera di sang phai thi
        # canh vat troi sang trai, nen van toc CAMERA nguoc dau voi flow.
        # >>> Day la cho dao dau DUY NHAT. Phai xac nhan bang phep do thuc te (di chuyen
        # camera mot quang da biet) truoc khi cho EKF dung - sai dau se gay troi vi tri.
        msg.twist.twist.linear.x = float(-vel_xy[0])
        msg.twist.twist.linear.y = float(-vel_xy[1])

        # Covariance ti le nghich voi so dac trung bam duoc: bam duoc it -> bao chat luong
        # thap cho EKF thay vi giau di (nguyen tac 1).
        cov = (self.get_parameter('base_covariance').value
               * self.get_parameter('min_tracked_features').value / max(n_tracked, 1))
        msg.twist.covariance[0] = cov       # vx
        msg.twist.covariance[7] = cov       # vy
        self.pub_velocity.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = OpticalFlowNode()
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
