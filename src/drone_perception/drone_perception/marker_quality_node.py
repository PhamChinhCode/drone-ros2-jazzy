"""marker_quality_node - tom tat chat luong bam marker cho lop an toan.

apriltag_ros khong publish san "dang bam ID nao, xa bao nhieu, sai so bao nhieu".
Node nay dich detection tho thanh mot ban tom tat, de failsafe_monitor_node khong phai
tu parse cau truc detection chi tiet.

Publish DEU bang timer ke ca khi khong thay marker (visible=false) - de ben nhan phan biet
duoc "khong thay marker" voi "node da chet".
"""

from concurrent.futures import ThreadPoolExecutor
import signal
import threading

import rclpy
from apriltag_msgs.msg import AprilTagDetectionArray
from rclpy.experimental import EventsExecutor
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from std_msgs.msg import Int32

from drone_interfaces.msg import MarkerQuality
from drone_perception.qos import EVENT_QOS, SENSOR_QOS


class MarkerQualityNode(Node):

    def __init__(self):
        super().__init__('marker_quality_node')

        self.declare_parameter('publish_rate_hz', 10.0)
        self.declare_parameter('visible_timeout_s', 0.5)
        self.declare_parameter('expected_marker_id', -1)   # -1 = chap nhan moi ID

        self.expected_id = self.get_parameter('expected_marker_id').value
        self.last_detection = None      # (marker_id, distance_m, lateral_offset_m, reproj_err)
        self.last_seen_time = None

        self.create_subscription(
            AprilTagDetectionArray, '/apriltag/detections', self.on_detections, SENSOR_QOS)
        self.create_subscription(
            Int32, '/mission/expected_marker_id', self.on_expected_id, EVENT_QOS)

        self.pub_quality = self.create_publisher(
            MarkerQuality, '/marker/tracking_quality', EVENT_QOS)

        period = 1.0 / self.get_parameter('publish_rate_hz').value
        self.create_timer(period, self.publish_quality)

    def on_expected_id(self, msg):
        """mission_manager_node bao ID cua bai dap dang huong toi."""
        self.expected_id = msg.data

    def on_detections(self, msg):
        """TODO: duyet msg.detections, giu lai detection khop expected_id (hoac moi ID neu -1),
        tinh distance_m va lateral_offset_m tu pose, luu vao last_detection + last_seen_time."""
        del msg

    def publish_quality(self):
        """TODO: neu qua visible_timeout_s ke tu last_seen_time -> visible=false, marker_id=-1;
        nguoc lai dien tu last_detection. Luon publish."""


def main(args=None):
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = MarkerQualityNode()
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
