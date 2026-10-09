"""optical_flow_node - do van toc troi ngang khi khong thay marker.

Node rui ro cao nhat cua lop cam nhan: van toc nhieu day thang vao EKF se gay troi vi tri.
Ba nguyen tac bat buoc (muc 1.4 tai lieu huong dan):
  1. covariance ti le nghich voi so dac trung bam duoc - do la cach bao chat luong thap cho EKF;
  2. duoi nguong dac trung toi thieu thi KHONG publish gi ca;
  3. phat hien lai dac trung dinh ky de khong bam mai diem da troi khoi khung hinh.

Camera lap truoc tam 90 mm, nghieng 20 do ve phia truoc (KHONG nhin thang xuong): pixel duoc doi
sang van toc bang flow_geometry (chieu tia nhin xuong mat dat theo TF lap dat + tu the IMU luc
chup moi khung), phat trong base_link. Xem docstring flow_geometry ve sai so cua cach cu.
"""

import collections
from concurrent.futures import ThreadPoolExecutor
import signal
import threading

import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import TwistWithCovarianceStamped
from rclpy.experimental import EventsExecutor
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image, Imu, Range
from tf2_ros import Buffer, TransformException, TransformListener

from drone_perception.flow_geometry import base_velocity, quat_to_matrix
from drone_perception.optical_flow_estimator import OpticalFlowEstimator
from drone_perception.qos import SENSOR_QOS


class OpticalFlowNode(Node):

    def __init__(self):
        super().__init__('optical_flow_node')

        self.declare_parameter('max_corners', 100)
        self.declare_parameter('quality_level', 0.02)
        self.declare_parameter('min_distance', 7)
        self.declare_parameter('min_tracked_features', 8)
        self.declare_parameter('refresh_interval_s', 1.5)
        self.declare_parameter('process_every_n_frames', 2)
        self.declare_parameter('base_covariance', 0.05)
        self.declare_parameter('max_lk_error', 20.0)
        # Tia nhin lech phuong thang dung qua goc nay bi bo (giao mat dat xa, nhay sai so do cao).
        self.declare_parameter('max_ray_angle_deg', 65.0)
        # IMU (MAVROS ~30 Hz) gan stamp anh nhat phai cach khong qua muc nay, khong thi bo khung.
        self.declare_parameter('max_imu_gap_s', 0.05)
        self.declare_parameter('altitude_filter_tau_s', 0.2)

        self.bridge = CvBridge()
        self.estimator = None          # tao sau khi co camera_info
        self.K = self.D = None         # noi tai + meo tu camera_info - khong bao gio do tay
        self.R_bo = self.p_cam_b = None  # TF lap dat base_link -> camera_optical_frame
        self.altitude_m = None         # chua co do cao thi chua quy doi duoc pixel -> met
        self.altitude_t = None         # stamp mau laser cuoi (cho bo loc)
        self.imu = collections.deque(maxlen=60)   # (t, R_wb, omega_b), ~2 s
        self.prev_view = None          # (t, R_wb, h_cam) cua khung xu ly truoc
        self.last_refresh = None
        self.frame_counter = 0

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self, spin_thread=True)

        self.create_subscription(CameraInfo, '/camera/camera_info', self.on_camera_info, SENSOR_QOS)
        self.create_subscription(Image, '/camera/image_raw', self.on_image, SENSOR_QOS)
        # Do cao lay tu laser MTF01P (20 Hz, ngang tam camera) thay /mavros/global_position/rel_alt:
        # relative_alt chua duoc giao uoc kiem (12.A1) va de FC giam GLOBAL_POSITION_INT (11.1 #14).
        self.create_subscription(Range, '/range/vertical', self.on_altitude, SENSOR_QOS)
        self.create_subscription(Imu, '/mavros/imu/data', self.on_imu, SENSOR_QOS)

        self.pub_velocity = self.create_publisher(
            TwistWithCovarianceStamped, '/optical_flow/velocity', SENSOR_QOS)

    def on_camera_info(self, msg):
        """Lay noi tai + he so meo da hieu chinh - khong bao gio do tay."""
        if self.estimator is None:
            self.K = np.array(msg.k, np.float64).reshape(3, 3)
            self.D = np.array(msg.d, np.float64)
            self.estimator = OpticalFlowEstimator(
                max_corners=self.get_parameter('max_corners').value,
                quality_level=self.get_parameter('quality_level').value,
                min_distance=self.get_parameter('min_distance').value,
                min_tracked_features=self.get_parameter('min_tracked_features').value,
                max_lk_error=self.get_parameter('max_lk_error').value)
            self.get_logger().info(
                f'Nhan camera_info: fx {msg.k[0]:.1f} px, {len(msg.d)} he so meo')

    def on_altitude(self, msg):
        # Ngoai [min_range, max_range] la khong do duoc -> khong quy doi, khong publish.
        ok = msg.min_range <= msg.range <= msg.max_range
        if not ok:
            self.altitude_m = self.altitude_t = None
            return
        # Loc thong thap: laser luong tu 1 cm, ma camera nhin chech truoc nen chenh do cao giua
        # hai khung bi hieu thanh di toi (~tan20 = 0,36 lan) - nhay 1 cm = 0,05 m/s vx gia khi
        # dung yen (do 10-08). EMA van bam dung toc do leo/xuong deu, chi tre gia tri tuyet doi.
        t = Time.from_msg(msg.header.stamp).nanoseconds * 1e-9
        if self.altitude_m is None or self.altitude_t is None or t <= self.altitude_t:
            self.altitude_m = msg.range
        else:
            tau = self.get_parameter('altitude_filter_tau_s').value
            a = 1.0 - np.exp(-(t - self.altitude_t) / tau) if tau > 0.0 else 1.0
            self.altitude_m += a * (msg.range - self.altitude_m)
        self.altitude_t = t

    def on_imu(self, msg):
        q = msg.orientation
        w = msg.angular_velocity
        self.imu.append((Time.from_msg(msg.header.stamp).nanoseconds * 1e-9,
                         quat_to_matrix(q.x, q.y, q.z, q.w), np.array([w.x, w.y, w.z])))

    def _imu_at(self, t):
        """(R_wb, omega_b) cua mau IMU gan t nhat, hoac None neu lech qua max_imu_gap_s."""
        if not self.imu:
            return None
        best = min(self.imu, key=lambda m: abs(m[0] - t))
        if abs(best[0] - t) > self.get_parameter('max_imu_gap_s').value:
            return None
        return best[1], best[2]

    def _lookup_mount(self):
        """TF lap dat camera (tinh) - lay mot lan tu estimation.launch.py, khong chep so."""
        try:
            tf = self.tf_buffer.lookup_transform('base_link', 'camera_optical_frame', Time())
        except TransformException:
            self.get_logger().warn('Chua co TF base_link -> camera_optical_frame',
                                   throttle_duration_sec=5.0)
            return False
        r, p = tf.transform.rotation, tf.transform.translation
        self.R_bo = quat_to_matrix(r.x, r.y, r.z, r.w)
        self.p_cam_b = np.array([p.x, p.y, p.z])
        self.get_logger().info(f'TF lap dat camera: vi tri {self.p_cam_b.round(3).tolist()} m')
        return True

    def on_image(self, msg):
        # Chua co camera_info (thieu focal length) hoac chua co do cao thi khong quy doi
        # duoc pixel -> met. Im lang cho, khong doan bua.
        if self.estimator is None or self.altitude_m is None:
            return
        if self.R_bo is None and not self._lookup_mount():
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

        t = stamp.nanoseconds * 1e-9
        att = self._imu_at(t)
        pairs, _ = self.estimator.track(gray)
        if att is None:
            # Khong biet tu the luc chup thi khong chieu duoc xuong dat; bam lai tu khung sau.
            self.estimator.reset()
            self.prev_view = None
            return
        R_wb, omega_b = att
        # Do cao camera = do cao laser (thang dung) + do lech dung cua camera theo tu the.
        # Gia dinh laser o tam than (vi tri lap laser chua do - mavros.yaml send_tf=false).
        h_cam = self.altitude_m + float((R_wb @ self.p_cam_b)[2])
        prev, self.prev_view = self.prev_view, (t, R_wb, h_cam)
        if pairs is None or prev is None:
            return

        vel, n_ok = base_velocity(
            pairs[0], pairs[1], self.K, self.D, self.R_bo, self.p_cam_b, prev[1], R_wb,
            prev[2], h_cam, t - prev[0], omega_b, self.get_parameter('max_ray_angle_deg').value)
        if vel is None or n_ok < self.get_parameter('min_tracked_features').value:
            return
        self.publish_velocity(vel, n_ok, msg.header.stamp)

    def publish_velocity(self, vel_b, n_tracked, stamp):
        msg = TwistWithCovarianceStamped()
        msg.header.stamp = stamp
        # Van toc THAN MAY (FLU) tinh tu hinh hoc mat dat - dau suy ra tu hinh hoc, khong con
        # buoc dao dau tay. Van phai xac nhan bang phep do thuc te (dich drone mot quang da
        # biet) truoc khi bay. EKF chi dung vx, vy (ekf.yaml twist0_config).
        msg.header.frame_id = 'base_link'
        msg.twist.twist.linear.x = float(vel_b[0])
        msg.twist.twist.linear.y = float(vel_b[1])
        msg.twist.twist.linear.z = float(vel_b[2])

        # Covariance ti le nghich voi so dac trung bam duoc: bam duoc it -> bao chat luong
        # thap cho EKF thay vi giau di (nguyen tac 1).
        cov = (self.get_parameter('base_covariance').value
               * self.get_parameter('min_tracked_features').value / max(n_tracked, 1))
        msg.twist.covariance[0] = cov       # vx
        msg.twist.covariance[7] = cov       # vy
        self.pub_velocity.publish(msg)


def main(args=None):
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = OpticalFlowNode()
    # EventsExecutor: tren Pi 4 executor mac dinh cua rclpy ton phan lon CPU de dung lai wait-set
    # moi lan thuc day (do 09-14: mission_manager_node 45-50 % -> 13,5 %).
    executor = EventsExecutor()
    executor.add_node(node)
    # Tu bat SIGINT/SIGTERM (init voi SignalHandlerOptions.NO): handler mac dinh cua rclpy tat
    # context ngay trong luc EventsExecutor con chay callback -> publish vao context da chet,
    # spin() nem loi, node thoat code 1 moi lan tat dich vu (2026-10-08). Dung executor TRUOC roi
    # moi tat context. Loi trong callback van lam node chet nhu cu (spin.result() nem lai).
    # Cho CO timeout: tin hieu roi vao luong khac thi wait() vo han khong bao gio thuc de
    # chay handler -> node treo toi khi bi SIGKILL (gap 1 lan khi tat dich vu).
    stop = threading.Event()
    for sig in (signal.SIGINT, signal.SIGTERM):
        signal.signal(sig, lambda *_: stop.set())
    spin = ThreadPoolExecutor(1).submit(executor.spin)
    spin.add_done_callback(lambda _: stop.set())
    while not stop.wait(0.5):
        pass
    executor.shutdown()
    try:
        spin.result()
    finally:
        node.destroy_node()
        rclpy.shutdown()
