"""landing_target_bridge_node - node DUY NHAT duoc phep khang dinh "dang bam dung marker".

Tach trach nhiem xac thuc ID khoi marker_detector_node (chi bao moi ID thay duoc) va khoi
mission_manager_node (khong nen tu parse pose tho).

Nguyen tac bat buoc: mat marker thi NGUNG PUBLISH ngay - khong noi suy, khong giu gia tri cu.
FC bam vao mot vi tri cu da khong con dung con nguy hiem hon la khong co du lieu.

Pose tag lay qua TF (apriltag_ros phat khung anh -> ten trong tag.frames), tra TF MOI NHAT va dung
stamp cua chinh TF do: khi CPU day tai /tf toi sau ban tin detections >100 ms (do 09-14, xem
marker_pose_republisher_node). Ten khung theo ID lay tu config/tags.yaml.

Phat trong he THAN PHANG base_level (2026-10-08, drone_control.level_frame): goc + huong mui cua
base_link nhung bo roll/pitch. Camera gan cung than: than nghieng 5-10 do (tang toc, ham) thi tag
ngay duoi bi bao lech ngang 9-17 cm o 1 m trong base_link -> dieu khien ha canh sua theo lech gia.
Roll/pitch lay tu TF world_frame -> base_link (EKF, nguon IMU); vi tri EKF khong anh huong.

Hai loai tag (docs/ke_hoach_camera_servo_gap_hang.md, 2026-10-22):
  - expected_marker_id = ID TAG TO: phat vi tri tag to = TAM BAI (ha canh / tha hang);
  - expected_marker_id = ID TAG NHO (id_to + offset): tag dan tren NOC HOP HANG - phat vi tri CHINH
    tag do de controller dat co cau gap (tam drone) ngay tren hop. Khong suy nguoc ve tam bai nua:
    hop khong con cung mat phang/cach co dinh voi tag to.
/landing_target/pad_odom: tu the bai do tu TAG TO (tam + huong) cho mission (quay mui, vi tri hop).
Servo camera dang quay (/camera/tilt NaN): khong phat gi - TF lap dat chua dung goc that.
"""

from concurrent.futures import ThreadPoolExecutor
import math
import signal
import threading

import rclpy
from apriltag_msgs.msg import AprilTagDetectionArray
from geometry_msgs.msg import PoseStamped
from rclpy.exceptions import ParameterUninitializedException
from rclpy.experimental import EventsExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.signals import SignalHandlerOptions
from rclpy.time import Time
from std_msgs.msg import Bool, Float32, Int32
from tf2_ros import Buffer, TransformException, TransformListener

from drone_control.level_frame import rotate, to_level
from drone_control.qos import EVENT_QOS, SENSOR_QOS
from drone_control.target_tracker import TargetTracker
from drone_estimation.pad_map import build_pad_map, SMALL_FRAME_SUFFIX, tag_up


class LandingTargetBridgeNode(Node):

    def __init__(self):
        super().__init__('landing_target_bridge_node')

        # timeout_s: qua dai -> FC bam vi tri cu da sai; qua ngan -> bao mat bam gia khi rung
        # mot khung hinh. 0.5-1.0 s tuy toc do ha canh.
        self.declare_parameter('timeout_s', 0.7)
        self.declare_parameter('publish_rate_hz', 25.0)     # 20-30 Hz trong pha bam marker
        self.declare_parameter('target_frame', 'base_link')
        # Khung the gioi cua EKF - chi lay roll/pitch cua base_link trong khung nay.
        self.declare_parameter('world_frame', 'odom')
        # Ban do tag dung chung (config/tags.yaml): chi can ID -> ten khung TF.
        self.declare_parameter('known_tags', Parameter.Type.DOUBLE_ARRAY)
        self.declare_parameter('tag_frames', Parameter.Type.STRING_ARRAY)
        self.declare_parameter('pad_small_tag_id_offset', 10)

        try:
            pads = build_pad_map(self.get_parameter('known_tags').value,
                                 self.get_parameter('tag_frames').value, {}, 0, 0.0)
        except ParameterUninitializedException:
            pads = {}
            self.get_logger().error('chua nap config/tags.yaml - khong bam duoc tag nao')
        self.tag_frames = {i: t.frame for i, t in pads.items()}
        # Tag nho (tren hop) cua MOI bai: id + offset, khung <ten>_s (apriltag.yaml).
        self.small_offset = self.get_parameter('pad_small_tag_id_offset').value
        self.small_frames = {i + self.small_offset: f + SMALL_FRAME_SUFFIX
                             for i, f in self.tag_frames.items()}
        self.tilt_moving = False

        self.expected_id = -1
        # Bai dang TIEP CAN (cong G, truc bai - chua bam tag de ha): chi phat tu the bai, khong
        # phat /landing_target/pose (pose do lam position_controller bo luat dan, bam tag).
        self.approach_id = -1
        self.last_pad_stamp = None
        self.tracker = TargetTracker(self.get_parameter('timeout_s').value)
        self.last_tf_stamp = None

        self.tf_buffer = Buffer()
        # spin_thread=True: buffer TF van cap nhat khi executor chinh ban callback (CPU day tai).
        self.tf_listener = TransformListener(self.tf_buffer, self, spin_thread=True)

        self.create_subscription(
            AprilTagDetectionArray, '/apriltag/detections', self.on_detections, SENSOR_QOS)
        self.create_subscription(
            Int32, '/mission/expected_marker_id', self.on_expected_id, EVENT_QOS)
        self.create_subscription(
            Int32, '/mission/approach_pad_id', self.on_approach_id, EVENT_QOS)
        self.create_subscription(Float32, '/camera/tilt', self.on_tilt, EVENT_QOS)

        # Topic NOI BO cho position_controller_node. KHONG gui LANDING_TARGET len FC: ban tin 149
        # phe bo tu giao uoc 1.4 - Pi tu dong vong van toc theo marker (11.1 #12, #13c).
        self.pub_target = self.create_publisher(PoseStamped, '/landing_target/pose', SENSOR_QOS)
        self.pub_lost = self.create_publisher(Bool, '/landing_target/lost', EVENT_QOS)
        # Tu the bai DO tu tag trong khung the gioi (world_frame): vi tri tam + huong "tren" (yaw
        # trong orientation). mission_manager dung thay huong khai bao cho cong G / truc bai /
        # huong mui khi ha - bai dat quay bat ky huong nao van dung.
        self.pub_pad = self.create_publisher(PoseStamped, '/landing_target/pad_odom', SENSOR_QOS)

        self.create_timer(1.0 / self.get_parameter('publish_rate_hz').value, self.check_timeout)

    def now_s(self):
        return self.get_clock().now().nanoseconds / 1e9

    def on_expected_id(self, msg):
        """Doi bai dap thi reset dong ho mat bam, tranh bao mat bam gia ngay sau khi doi ID."""
        if msg.data == self.expected_id:
            return
        self.publish_event(self.tracker.reset())     # bao mat bam theo ID CU truoc khi doi
        self.expected_id = msg.data
        self.last_tf_stamp = None
        if msg.data >= 0 and self.frame_of(msg.data) is None:
            self.get_logger().warning(f'tag {msg.data} khong co trong config/tags.yaml')

    def on_approach_id(self, msg):
        if msg.data != self.approach_id:
            self.approach_id = msg.data
            self.last_pad_stamp = None

    def on_tilt(self, msg):
        self.tilt_moving = math.isnan(msg.data)

    def frame_of(self, tag_id):
        """Ten khung TF cua tag to hoac tag nho; None = khong co trong ban do."""
        return self.tag_frames.get(tag_id) or self.small_frames.get(tag_id)

    def measure(self, tag_id, seen):
        """(TF target <- tag, vi tri (x, y, z), quaternion) cua tag_id neu dang thay, khong thi None."""
        frame = self.frame_of(tag_id)
        if tag_id not in seen or frame is None:
            return None
        try:
            t = self.tf_buffer.lookup_transform(self.get_parameter('target_frame').value, frame,
                                                Time())
        except TransformException:
            return None
        tr, rq = t.transform.translation, t.transform.rotation
        return t, (tr.x, tr.y, tr.z), (rq.x, rq.y, rq.z, rq.w)

    def on_detections(self, msg):
        """Tag expected_id -> /landing_target/pose (base_level); tag TO cua bai dang tiep can
        (approach_id, khong co thi bai cua expected_id) -> /landing_target/pad_odom."""
        if self.tilt_moving:
            return
        seen = {d.id for d in msg.detections}
        try:
            att = self.tf_buffer.lookup_transform(
                self.get_parameter('world_frame').value,
                self.get_parameter('target_frame').value, Time()).transform
        except TransformException:
            return
        q_wb = (att.rotation.x, att.rotation.y, att.rotation.z, att.rotation.w)
        m = self.measure(self.expected_id, seen)
        if m is not None:
            self.publish_landing(m, q_wb)
        pad_id = self.approach_id
        if pad_id not in self.tag_frames:
            pad_id = (self.expected_id - self.small_offset if self.expected_id in self.small_frames
                      else self.expected_id)
        if pad_id in self.tag_frames:
            m = self.measure(pad_id, seen)
            if m is not None:
                self.publish_pad(m, q_wb, att.translation)

    def publish_landing(self, m, q_wb):
        t, p, q = m
        stamp = Time.from_msg(t.header.stamp)
        age_s = self.now_s() - stamp.nanoseconds / 1e9
        if stamp == self.last_tf_stamp or age_s > self.tracker.timeout_s:
            return      # TF cu hoac da dung - khong phat lai gia tri cu (nguyen tac o dau file)
        self.last_tf_stamp = stamp
        pos, quat = to_level(q_wb, p, q)
        out = PoseStamped()
        out.header.stamp = t.header.stamp
        out.header.frame_id = 'base_level'      # khong co TF ten nay - xem docstring dau file
        out.pose.position.x, out.pose.position.y, out.pose.position.z = pos
        (out.pose.orientation.x, out.pose.orientation.y, out.pose.orientation.z,
         out.pose.orientation.w) = quat
        self.pub_target.publish(out)
        self.publish_event(self.tracker.seen(self.now_s()))

    def publish_pad(self, m, q_wb, base_in_world):
        """Tam bai + huong "tren" DO DUOC trong khung the gioi (yaw la ban gia dinh dung)."""
        t, p, q = m
        stamp = Time.from_msg(t.header.stamp)
        if stamp == self.last_pad_stamp or self.now_s() - stamp.nanoseconds / 1e9 > 0.5:
            return
        self.last_pad_stamp = stamp
        c = rotate(q_wb, p)
        up = rotate(q_wb, tag_up(q))
        yaw = math.atan2(up[1], up[0])
        out = PoseStamped()
        out.header.stamp = t.header.stamp
        out.header.frame_id = self.get_parameter('world_frame').value
        out.pose.position.x = base_in_world.x + c[0]
        out.pose.position.y = base_in_world.y + c[1]
        out.pose.position.z = base_in_world.z + c[2]
        out.pose.orientation.z, out.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
        self.pub_pad.publish(out)

    def check_timeout(self):
        """Qua timeout_s khong thay -> /landing_target/lost = true dung mot lan."""
        self.publish_event(self.tracker.check(self.now_s()))

    def publish_event(self, event):
        if event is None:
            return
        self.pub_lost.publish(Bool(data=(event == 'lost')))
        text = f'tag {self.expected_id}: {"MAT BAM" if event == "lost" else "bat duoc"}'
        if event == 'lost':
            self.get_logger().warning(text)
        else:
            self.get_logger().info(text)


def main(args=None):
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = LandingTargetBridgeNode()
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
