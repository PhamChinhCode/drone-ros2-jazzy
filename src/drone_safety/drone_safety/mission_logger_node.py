"""mission_logger_node - log NGHIEP VU co cau truc, khac muc dich voi rosbag2.

rosbag2 ghi tho de debug ky thuat (khai bao trong launch bang ExecuteProcess).
Node nay ghi log doi chieu duoc voi CSDL GCS (bang mission, mission_waypoint, mission_photo):
JSON-lines + anh xac nhan tai thoi diem gap/tha.

Luu y Pi 4: ghi anh tho full-size lien tuc ton I/O the SD dang ke tren chuyen bay dai -
dung the U3/A2 hoac SSD qua USB 3.0, va/hoac chi ghi anh nen.
"""

import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image

from drone_interfaces.msg import FailsafeEvent, GripperStatus, MarkerQuality, MissionState
from drone_safety.qos import EVENT_QOS, SENSOR_QOS


class MissionLoggerNode(Node):

    def __init__(self):
        super().__init__('mission_logger_node')

        self.declare_parameter('log_root', '~/drone_logs')
        self.declare_parameter('save_photos', True)

        self.bridge = CvBridge()
        self.mission_id = None
        self.log_file = None
        self.last_image = None
        self.last_state = None

        self.create_subscription(MissionState, '/mission/state', self.on_mission_state, EVENT_QOS)
        self.create_subscription(FailsafeEvent, '/failsafe_event', self.on_failsafe, EVENT_QOS)
        self.create_subscription(
            MarkerQuality, '/marker/tracking_quality', self.on_marker, EVENT_QOS)
        self.create_subscription(GripperStatus, '/gripper/status', self.on_gripper, EVENT_QOS)
        self.create_subscription(Image, '/camera/image_raw', self.on_image, SENSOR_QOS)

    def open_mission_log(self, mission_id):
        """TODO: tao thu muc log_root/<mission_id>/{photos} va mo events.jsonl che do append."""
        del mission_id

    def write_event(self, event_type, payload):
        """TODO: ghi mot dong JSON {stamp, mission_id, event_type, ...payload}.
        event_type: STATE_CHANGE | FAILSAFE | MARKER_MATCH | GRIP_OK | GRIP_FAIL."""
        del event_type, payload

    def on_mission_state(self, msg):
        """TODO: mission_id moi -> open_mission_log; state doi -> write_event STATE_CHANGE."""
        del msg

    def on_failsafe(self, msg):
        """TODO: write_event FAILSAFE."""
        del msg

    def on_marker(self, msg):
        """TODO: chi ghi khi visible chuyen tu false sang true (MARKER_MATCH), khong ghi moi khung."""
        del msg

    def on_gripper(self, msg):
        """TODO: sensor_confirmed doi -> write_event GRIP_OK/GRIP_FAIL kem luu anh xac nhan."""
        del msg

    def on_image(self, msg):
        """Chi giu khung moi nhat trong RAM; chi ghi ra the khi co su kien gap/tha."""
        self.last_image = msg


def main(args=None):
    rclpy.init(args=args)
    node = MissionLoggerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
