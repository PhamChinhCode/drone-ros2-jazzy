"""landing_target_bridge_node - node DUY NHAT duoc phep khang dinh "dang bam dung marker".

Tach trach nhiem xac thuc ID khoi marker_detector_node (chi bao moi ID thay duoc) va khoi
mission_manager_node (khong nen tu parse pose tho).

Nguyen tac bat buoc: mat marker thi NGUNG PUBLISH ngay - khong noi suy, khong giu gia tri cu.
FC bam vao mot vi tri cu da khong con dung con nguy hiem hon la khong co du lieu.
"""

import rclpy
from apriltag_msgs.msg import AprilTagDetectionArray
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from std_msgs.msg import Bool, Int32
from tf2_ros import Buffer, TransformListener

from drone_control.qos import EVENT_QOS, SENSOR_QOS


class LandingTargetBridgeNode(Node):

    def __init__(self):
        super().__init__('landing_target_bridge_node')

        # timeout_s: qua dai -> FC bam vi tri cu da sai; qua ngan -> bao mat bam gia khi rung
        # mot khung hinh. 0.5-1.0 s tuy toc do ha canh.
        self.declare_parameter('timeout_s', 0.7)
        self.declare_parameter('publish_rate_hz', 25.0)     # 20-30 Hz trong pha bam marker
        self.declare_parameter('target_frame', 'base_link')

        self.expected_id = -1
        self.last_seen_time = None
        self.lost_reported = False

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.create_subscription(
            AprilTagDetectionArray, '/apriltag/detections', self.on_detections, SENSOR_QOS)
        self.create_subscription(
            Int32, '/mission/expected_marker_id', self.on_expected_id, EVENT_QOS)

        # Topic NOI BO cho position_controller_node. KHONG gui LANDING_TARGET len FC: ban tin 149
        # phe bo tu giao uoc 1.4 - Pi tu dong vong van toc theo marker (11.1 #12, #13c).
        self.pub_target = self.create_publisher(PoseStamped, '/landing_target/pose', SENSOR_QOS)
        self.pub_lost = self.create_publisher(Bool, '/landing_target/lost', EVENT_QOS)

        self.create_timer(1.0 / self.get_parameter('publish_rate_hz').value, self.check_timeout)

    def on_expected_id(self, msg):
        """Doi bai dap thi reset dong ho mat bam, tranh bao mat bam gia ngay sau khi doi ID."""
        if msg.data != self.expected_id:
            self.expected_id = msg.data
            self.last_seen_time = None
            self.lost_reported = False

    def on_detections(self, msg):
        """TODO: tim detection co ID == expected_id (bo qua moi ID khac);
        chuyen pose sang target_frame bang tf2, dien PoseStamped va publish;
        cap nhat last_seen_time. Khong thay ID mong doi -> khong publish gi ca."""
        del msg

    def check_timeout(self):
        """TODO: qua timeout_s ke tu last_seen_time -> publish /landing_target/lost = true
        dung mot lan (lost_reported), va publish false khi bam lai duoc."""


def main(args=None):
    rclpy.init(args=args)
    node = LandingTargetBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
