"""marker_quality_node - tom tat chat luong bam marker cho lop an toan.

apriltag_ros khong publish san "dang bam ID nao, xa bao nhieu, sai so bao nhieu".
Node nay dich detection tho thanh mot ban tom tat, de failsafe_monitor_node khong phai
tu parse cau truc detection chi tiet.

Publish DEU bang timer ke ca khi khong thay marker (visible=false) - de ben nhan phan biet
duoc "khong thay marker" voi "node da chet".
"""

import rclpy
from apriltag_msgs.msg import AprilTagDetectionArray
from rclpy.node import Node
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
    rclpy.init(args=args)
    node = MarkerQualityNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
