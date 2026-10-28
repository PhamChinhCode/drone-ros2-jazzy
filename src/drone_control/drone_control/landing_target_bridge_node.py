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

Hai loai tag theo dai ID (user 10-27, apriltag.yaml):
  - TAG NEO (< box_tag_id_min): expected_marker_id = ID TAG TO cua bai -> phat TAM BAI. Bai hai tag
    (mau 10-09, giao uoc GCS 0.8): thay tag to thi dung tag to; xuong thap tag to ra khoi khung thi
    suy tam bai tu tag nho (id + offset, in cung to, forward_m ve phia "tren") theo truc TREN DO
    DUOC cua chinh tag nho - bai quay bat ky huong nao van dung. Mission/controller khong can biet
    dang bam tag nao.
  - TAG HOP (>= box_tag_id_min): expected_marker_id la mot id hop bat ky -> nhan MOI tag hop, khoa
    vao hop thay DAU TIEN toi khi expected_marker_id doi (moi lan tiep can mot hop, tam hop khong
    nhay giua hai hop); phat vi tri CHINH tag hop de controller dat co cau gap ngay tren hop.
/landing_target/pad_odom: tu the bai DO (tam + huong, tu tag to hoac tag nho) cho mission (quay mui,
  vi tri hop) - bai dang tiep can, khong co thi bai mong doi gan nhat.
/landing_target/world: vi tri tag expected_marker_id trong world_frame tai DUNG luc chup anh (vi tri
  hop cho mission - ghep pose tuong doi voi vi tri drone MOI NHAT lech toi 10 cm trong gio, 10-25);
  orientation = yaw huong "tren" cua tag (mission quay mui theo hop).
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

from drone_control.level_frame import to_level
from drone_control.qos import EVENT_QOS, SENSOR_QOS
from drone_control.target_tracker import TargetTracker
from drone_estimation.pad_map import (box_frames, build_pad_map, small_to_center,
                                      SMALL_FRAME_SUFFIX, tag_up)


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
        self.declare_parameter('pad_small_tag_forward_m', 0.21)
        self.declare_parameter('box_tag_id_min', 20)
        self.declare_parameter('box_tag_count', 10)
        self.forward_m = self.get_parameter('pad_small_tag_forward_m').value

        try:
            pads = build_pad_map(self.get_parameter('known_tags').value,
                                 self.get_parameter('tag_frames').value, {}, 0, 0.0)
        except ParameterUninitializedException:
            pads = {}
            self.get_logger().error('chua nap config/tags.yaml - khong bam duoc tag nao')
        self.tag_frames = {i: t.frame for i, t in pads.items()}
        # Tag nho cua MOI bai (ke ca bai chua khai huong): id + offset, khung <ten>_s.
        self.small_offset = self.get_parameter('pad_small_tag_id_offset').value
        self.small_frame = {i: f + SMALL_FRAME_SUFFIX for i, f in self.tag_frames.items()}
        self.box_frames = box_frames(self.get_parameter('box_tag_id_min').value,
                                     self.get_parameter('box_tag_count').value)
        self.tilt_moving = False

        self.expected_id = -1
        self.pad_id = -1        # bai mong doi gan nhat (pad_odom khi dang bam tag hop)
        self.box_id = None      # tag hop dang khoa khi expected_id la tag hop; None = chua thay hop
        self.source = None      # dang bam tag nao cua bai: 'to' | 'nho' (chi de ghi log khi doi)
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
        self.pub_world = self.create_publisher(PoseStamped, '/landing_target/world', SENSOR_QOS)

        self.create_timer(1.0 / self.get_parameter('publish_rate_hz').value, self.check_timeout)

    def now_s(self):
        return self.get_clock().now().nanoseconds / 1e9

    def on_expected_id(self, msg):
        """Doi bai dap thi reset dong ho mat bam, tranh bao mat bam gia ngay sau khi doi ID."""
        if msg.data == self.expected_id:
            return
        self.publish_event(self.tracker.reset())     # bao mat bam theo ID CU truoc khi doi
        self.expected_id = msg.data
        self.box_id = None
        self.source = None
        self.last_tf_stamp = None
        if msg.data in self.tag_frames:
            self.pad_id = msg.data
        elif msg.data >= 0 and msg.data not in self.box_frames:
            self.get_logger().warning(f'tag {msg.data} khong phai tag to cua bai nao '
                                      f'(config/tags.yaml) cung khong phai tag hop')

    def on_approach_id(self, msg):
        if msg.data != self.approach_id:
            self.approach_id = msg.data
            self.last_pad_stamp = None

    def on_tilt(self, msg):
        self.tilt_moving = math.isnan(msg.data)

    def lookup(self, frame, frame_to):
        """(TF frame_to <- frame, vi tri (x, y, z), quaternion) hoac None."""
        try:
            t = self.tf_buffer.lookup_transform(frame_to, frame, Time())
        except TransformException:
            return None
        tr, rq = t.transform.translation, t.transform.rotation
        return t, (tr.x, tr.y, tr.z), (rq.x, rq.y, rq.z, rq.w)

    def measure_pad(self, pad_id, seen, frame_to):
        """Tam bai pad_id trong frame_to tu tag to (uu tien) hoac tag nho: (TF, tam, quaternion
        frame_to <- tag da dung, la tag nho?) hoac None."""
        if pad_id in seen:
            m, small = self.lookup(self.tag_frames[pad_id], frame_to), False
        elif pad_id + self.small_offset in seen:
            m, small = self.lookup(self.small_frame[pad_id], frame_to), True
        else:
            return None
        if m is None:
            return None
        t, p, q = m
        return t, (small_to_center(p, q, self.forward_m) if small else p), q, small

    def on_detections(self, msg):
        """Bai expected_id (tag to / tag nho) hoac hop dang khoa -> /landing_target/pose (base_level);
        hop con -> vi tri trong world_frame (/landing_target/world); bai dang tiep can (approach_id,
        khong co thi bai mong doi gan nhat) -> /landing_target/pad_odom."""
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
        target = self.get_parameter('target_frame').value
        world = self.get_parameter('world_frame').value
        if self.expected_id in self.tag_frames:
            m = self.measure_pad(self.expected_id, seen, target)
            if m is not None:
                source = 'nho' if m[3] else 'to'
                if source != self.source:
                    self.get_logger().info(f'bai {self.expected_id}: bam theo tag {source}')
                    self.source = source
                self.publish_landing(m[:3], q_wb)
        elif self.expected_id in self.box_frames:
            box_id = self.track_box(seen)
            if box_id is not None:
                m = self.lookup(self.box_frames[box_id], target)
                if m is not None and self.publish_landing(m, q_wb):
                    w = self.lookup(self.box_frames[box_id], world)
                    if w is not None:
                        self.publish_world(w)
        pad_id = self.approach_id if self.approach_id in self.tag_frames else self.pad_id
        if pad_id in self.tag_frames:
            m = self.measure_pad(pad_id, seen, world)
            if m is not None:
                self.publish_pad(m[:3])

    def track_box(self, seen):
        """Hop dang khoa neu dang thay; chua khoa thi khoa hop thay duoc (uu tien expected_id, roi
        id nho nhat). None = khong thay hop can bam."""
        if self.box_id is None:
            hop = sorted(i for i in seen if i in self.box_frames)
            if not hop:
                return None
            self.box_id = self.expected_id if self.expected_id in hop else hop[0]
            self.get_logger().info(f'khoa hop tag {self.box_id}')
        return self.box_id if self.box_id in seen else None

    def publish_landing(self, m, q_wb):
        t, p, q = m
        stamp = Time.from_msg(t.header.stamp)
        age_s = self.now_s() - stamp.nanoseconds / 1e9
        if stamp == self.last_tf_stamp or age_s > self.tracker.timeout_s:
            return False    # TF cu hoac da dung - khong phat lai gia tri cu (nguyen tac o dau file)
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
        return True

    def publish_world(self, m):
        """Vi tri tag dang bam trong world_frame (TF noi suy ca chuoi tai luc chup, nhu publish_pad),
        orientation = yaw huong "tren" cua tag."""
        t, p, q = m
        up = tag_up(q)
        yaw = math.atan2(up[1], up[0])
        out = PoseStamped()
        out.header.stamp = t.header.stamp
        out.header.frame_id = self.get_parameter('world_frame').value
        out.pose.position.x, out.pose.position.y, out.pose.position.z = p
        out.pose.orientation.z, out.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
        self.pub_world.publish(out)

    def publish_pad(self, m):
        """Tam bai + huong "tren" DO DUOC trong khung the gioi (yaw la ban gia dinh dung).

        m tra thang world <- tag: TF noi suy CA chuoi odom -> base_link -> camera tai DUNG luc
        chup anh. Truoc 10-25 ghep tag (luc chup) voi vi tri / tu the drone MOI NHAT: drone di
        chuyen trong do tre xu ly anh -> tam bai dao dong +-3 cm theo van toc (mo phong)."""
        t, p, q = m
        stamp = Time.from_msg(t.header.stamp)
        if stamp == self.last_pad_stamp or self.now_s() - stamp.nanoseconds / 1e9 > 0.5:
            return
        self.last_pad_stamp = stamp
        up = tag_up(q)
        yaw = math.atan2(up[1], up[0])
        out = PoseStamped()
        out.header.stamp = t.header.stamp
        out.header.frame_id = self.get_parameter('world_frame').value
        out.pose.position.x, out.pose.position.y, out.pose.position.z = p
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
