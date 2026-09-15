"""mission_logger_node - log NGHIEP VU co cau truc, khac muc dich voi rosbag2.

rosbag2 ghi tho de debug ky thuat (khai bao trong launch bang ExecuteProcess).
Node nay ghi log doi chieu duoc voi CSDL GCS (bang mission, mission_waypoint, mission_photo):
JSON-lines + anh xac nhan tai thoi diem gap/tha. Logic ghi nam trong mission_log.py.

Bat / mat tag lay tu /landing_target/lost cua landing_target_bridge_node (node duy nhat xac thuc
dung ID mong doi), khong tu /marker/tracking_quality.

Luu y Pi 4: chuyen anh 640x400 30 FPS vao Python ton CPU dang ke - chi dang ky
/camera/image_raw khi dang ACTUATE_GRIPPER, trang thai duy nhat can anh xac nhan.
"""

import rclpy
from cv_bridge import CvBridge
from rclpy.experimental import EventsExecutor
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Bool

from drone_interfaces.msg import FailsafeEvent, GripperStatus, MissionState
from drone_safety.mission_log import MissionLog
from drone_safety.qos import EVENT_QOS, SENSOR_QOS

STATE_NAMES = {getattr(MissionState, n): n for n in (
    'IDLE', 'TAKEOFF', 'ENROUTE', 'MARKER_SEARCH', 'PRECISION_LAND', 'ACTUATE_GRIPPER',
    'RETRY_LOITER', 'RTH', 'EMERGENCY_LAND', 'MISSION_COMPLETE', 'FAILSAFE')}
PHOTO_MAX_AGE_S = 1.0          # anh cu hon muc nay khong con la anh "tai thoi diem" gap/tha


class MissionLoggerNode(Node):

    def __init__(self):
        super().__init__('mission_logger_node')

        self.declare_parameter('log_root', '~/drone_logs')
        self.declare_parameter('save_photos', True)

        self.bridge = CvBridge()
        self.log = MissionLog(self.get_parameter('log_root').value)
        self.expected_marker_id = -1
        self.last_image = None
        self.last_image_s = None
        self.image_sub = None

        self.create_subscription(MissionState, '/mission/state', self.on_mission_state, EVENT_QOS)
        self.create_subscription(FailsafeEvent, '/failsafe_event', self.on_failsafe, EVENT_QOS)
        self.create_subscription(Bool, '/landing_target/lost', self.on_target_lost, EVENT_QOS)
        self.create_subscription(GripperStatus, '/gripper/status', self.on_gripper, EVENT_QOS)
        self.get_logger().info(f'ghi log nhiem vu vao {self.log.log_root}')

    def now_s(self):
        return self.get_clock().now().nanoseconds / 1e9

    def on_mission_state(self, msg):
        # MissionState.expected_marker_id = -1 ca trong ban phat tu on_plan - chi nho gia tri >= 0.
        if msg.expected_marker_id >= 0:
            self.expected_marker_id = msg.expected_marker_id
        name = STATE_NAMES.get(msg.state, str(msg.state))
        for rec in self.log.on_mission_state(self.now_s(), msg.mission_id, name,
                                             msg.current_wp_index, msg.retry_count,
                                             msg.expected_marker_id, msg.detail):
            self.get_logger().info(f'{rec["event_type"]}: {rec.get("to", name)} {msg.detail}')
        self.update_image_subscription(name == 'ACTUATE_GRIPPER')

    def update_image_subscription(self, need):
        if not self.get_parameter('save_photos').value:
            need = False
        if need and self.image_sub is None:
            self.image_sub = self.create_subscription(
                Image, '/camera/image_raw', self.on_image, SENSOR_QOS)
        elif not need and self.image_sub is not None:
            self.destroy_subscription(self.image_sub)
            self.image_sub = None
            self.last_image = None

    def on_failsafe(self, msg):
        self.log.on_failsafe(self.now_s(), msg.type, msg.escalate_to, msg.active, msg.detail)

    def on_target_lost(self, msg):
        self.log.on_target_lost(self.now_s(), msg.data, self.expected_marker_id)

    def on_gripper(self, msg):
        now = self.now_s()
        image = None
        if self.last_image is not None and now - self.last_image_s <= PHOTO_MAX_AGE_S:
            image = self.bridge.imgmsg_to_cv2(self.last_image, desired_encoding='mono8')
        rec = self.log.on_gripper(now, msg.sensor_confirmed, msg.state, msg.force_reading_n, image)
        if rec is not None:
            self.get_logger().info(f'{rec["event_type"]} anh={rec["photo"]}')

    def on_image(self, msg):
        """Chi giu khung moi nhat trong RAM; chi ghi ra the khi co su kien gap/tha."""
        self.last_image = msg
        self.last_image_s = self.now_s()


def main(args=None):
    rclpy.init(args=args)
    node = MissionLoggerNode()
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
