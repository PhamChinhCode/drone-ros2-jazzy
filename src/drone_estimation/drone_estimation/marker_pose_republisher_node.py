"""marker_pose_republisher_node - dua pose marker vao khung odom cho EKF.

apriltag_ros cho pose marker trong khung quang hoc cua camera. robot_localization can
PoseWithCovarianceStamped trong khung odom va KHONG tu lam buoc chuyen nay.

Diem de sai nhat: nham chieu transform camera <-> body. Trieu chung dac trung la
"thay dung marker nhung bay lech tam".

AprilTagDetection KHONG mang pose: apriltag_ros phat pose tag qua TF (khung anh -> ten trong
tag.frames cua apriltag.yaml). Node nay tra TF base_link -> tag tai dung stamp anh, xoay bang huong
odom -> base_link (tu IMU qua EKF) va tru vao toa do da biet cua tag.

Tra TF tag MOI NHAT chu khong tra dung stamp detection: do 09-14 luc CPU day tai, /tf cua tag toi
SAU ban tin detections hon 100 ms -> tra dung stamp luon ExtrapolationException (pose 0,2 Hz du
detections 15 Hz). Pose phat ra mang stamp cua chinh TF tag nen EKF van nhan dung thoi diem chup.

Chi TAG TO neo EKF (2026-10-22): tag nho nay dan tren hop hang dat tay, lech vai cm. Servo camera dang
quay (/camera/tilt NaN) thi bo moi detection - TF lap dat camera chua dung goc that.
"""

from concurrent.futures import ThreadPoolExecutor
import math
import signal
import threading

import rclpy
from apriltag_msgs.msg import AprilTagDetectionArray
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.experimental import EventsExecutor
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from rclpy.time import Time
from std_msgs.msg import Float32
from tf2_ros import Buffer, TransformException, TransformListener

from drone_estimation.estimation_math import drone_position_from_tag
from drone_estimation.pad_map import build_pad_map
from drone_estimation.qos import EVENT_QOS, SENSOR_QOS

# TF tag cu hon muc nay (so voi dong ho node) thi bo - pose da troi, EKF khong nen nhan.
MAX_TAG_AGE_S = 0.5
LARGE_VARIANCE = 1e6   # truc khong do (goc) - EKF chi fuse x, y, z cua pose0


class MarkerPoseRepublisherNode(Node):

    def __init__(self):
        super().__init__('marker_pose_republisher_node')

        # Marker cho vi tri TUYET DOI nen covariance thap (tin nhieu) - nguoc voi optical flow.
        self.declare_parameter('pos_covariance', 0.01)      # m^2
        self.declare_parameter('target_frame', 'odom')
        # Toa do da biet cua tung bai dap trong khung target_frame, dang phang [id, x, y, z, ...].
        self.declare_parameter('known_tags', [0.0])
        # Ten khung TF tung tag, CUNG THU TU voi known_tags - khop tag.frames trong apriltag.yaml.
        self.declare_parameter('tag_frames', [''])
        self.declare_parameter('base_frame', 'base_link')

        # Khong truyen huong bai -> pad_map khong suy tag nho: chi tag to neo EKF.
        pads = build_pad_map(self.get_parameter('known_tags').value,
                             self.get_parameter('tag_frames').value, {}, 0, 0.0)
        self.known_tags = {i: t.pos for i, t in pads.items()}
        self.tag_frames = {i: t.frame for i, t in pads.items()}
        self.get_logger().info(f'ban do: {len(self.known_tags)} bai, khung '
                               f'{sorted(self.tag_frames.values())}')
        self.tilt_moving = False
        self.warned = set()
        self.last_tag_stamp = {}         # tag id -> stamp TF da dung, tranh phat trung

        self.tf_buffer = Buffer()
        # spin_thread=True: buffer TF van cap nhat khi executor chinh ban callback (CPU day tai).
        self.tf_listener = TransformListener(self.tf_buffer, self, spin_thread=True)

        self.create_subscription(
            AprilTagDetectionArray, '/apriltag/detections', self.on_detections, SENSOR_QOS)
        self.create_subscription(Float32, '/camera/tilt', self.on_tilt, EVENT_QOS)
        self.pub_pose = self.create_publisher(
            PoseWithCovarianceStamped, '/marker/pose_odom', SENSOR_QOS)

    def on_tilt(self, msg):
        self.tilt_moving = math.isnan(msg.data)

    def on_detections(self, msg):
        """Moi detection co trong known_tags -> mot pose drone trong target_frame."""
        if self.tilt_moving:
            return
        target = self.get_parameter('target_frame').value
        base = self.get_parameter('base_frame').value
        var = self.get_parameter('pos_covariance').value
        now = self.get_clock().now()
        for det in msg.detections:
            if det.id not in self.tag_frames:
                continue
            try:
                # TF tag moi nhat (base_link -> camera do camera_tilt_node phat 30 Hz).
                t_tag = self.tf_buffer.lookup_transform(base, self.tag_frames[det.id], Time())
                # Huong than may: moi nhat la du, huong doi cham hon nhieu so voi vi tri.
                t_att = self.tf_buffer.lookup_transform(target, base, Time())
            except TransformException as exc:
                self.warn_once(f'tf{det.id}', f'khong tra duoc TF cho tag {det.id}: {exc}')
                continue
            tag_stamp = Time.from_msg(t_tag.header.stamp)
            if (self.last_tag_stamp.get(det.id) == tag_stamp
                    or (now - tag_stamp).nanoseconds / 1e9 > MAX_TAG_AGE_S):
                continue
            self.last_tag_stamp[det.id] = tag_stamp
            tr, q = t_tag.transform.translation, t_att.transform.rotation
            pos = drone_position_from_tag(self.known_tags[det.id], (tr.x, tr.y, tr.z),
                                          (q.x, q.y, q.z, q.w))
            if not all(math.isfinite(v) for v in pos):
                continue

            out = PoseWithCovarianceStamped()
            out.header.stamp = t_tag.header.stamp
            out.header.frame_id = target
            out.pose.pose.position.x, out.pose.pose.position.y, out.pose.pose.position.z = pos
            out.pose.pose.orientation = q
            cov = [0.0] * 36
            for i in range(6):
                cov[i * 7] = var if i < 3 else LARGE_VARIANCE
            out.pose.covariance = cov
            self.pub_pose.publish(out)

    def warn_once(self, key, text):
        if key not in self.warned:
            self.warned.add(key)
            self.get_logger().warning(text)


def main(args=None):
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = MarkerPoseRepublisherNode()
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
