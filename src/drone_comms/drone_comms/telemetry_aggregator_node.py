"""telemetry_aggregator_node - gop nhieu nguon thanh dung mot goi TelemetryPacket.

Tach khoi gcs_link_node de logic gop du lieu khong lan voi logic ma hoa/giai ma giao thuc.

Publish o TAN SO CO DINH bang timer (mac dinh 2 Hz), KHONG publish theo su kien cua tung
nguon - tranh lam ngap kenh 4G/radio bang thong hep.
"""

import rclpy
from mavros_msgs.msg import State
from rclpy.node import Node
from sensor_msgs.msg import BatteryState, NavSatFix

from drone_comms.qos import EVENT_QOS, SENSOR_QOS
from drone_interfaces.msg import (FailsafeEvent, GripperStatus, MarkerQuality,
                                  MissionState, TelemetryPacket)


class TelemetryAggregatorNode(Node):

    def __init__(self):
        super().__init__('telemetry_aggregator_node')

        self.declare_parameter('publish_rate_hz', 2.0)

        self.mission_state = None
        self.fc_state = None
        self.battery = None
        self.global_pos = None
        self.gripper = None
        self.marker = None
        self.failsafe = None

        self.create_subscription(MissionState, '/mission/state', self.on_mission_state, EVENT_QOS)
        self.create_subscription(State, '/mavros/state', self.on_fc_state, EVENT_QOS)
        self.create_subscription(BatteryState, '/mavros/battery', self.on_battery, SENSOR_QOS)
        self.create_subscription(
            NavSatFix, '/mavros/global_position/global', self.on_global_pos, SENSOR_QOS)
        self.create_subscription(GripperStatus, '/gripper/status', self.on_gripper, EVENT_QOS)
        self.create_subscription(
            MarkerQuality, '/marker/tracking_quality', self.on_marker, EVENT_QOS)
        self.create_subscription(FailsafeEvent, '/failsafe_event', self.on_failsafe, EVENT_QOS)

        self.pub_telemetry = self.create_publisher(
            TelemetryPacket, '/telemetry/outgoing', EVENT_QOS)
        self.create_timer(1.0 / self.get_parameter('publish_rate_hz').value, self.publish_packet)

    def on_mission_state(self, msg):
        self.mission_state = msg

    def on_fc_state(self, msg):
        self.fc_state = msg

    def on_battery(self, msg):
        self.battery = msg

    def on_global_pos(self, msg):
        self.global_pos = msg

    def on_gripper(self, msg):
        self.gripper = msg

    def on_marker(self, msg):
        self.marker = msg

    def on_failsafe(self, msg):
        self.failsafe = msg

    def read_rssi_dbm(self):
        """TODO: doc RSSI modem 4G qua AT command/ModemManager; tra 0 neu chua co."""
        return 0

    def publish_packet(self):
        """TODO: gop cac truong da luu thanh TelemetryPacket va publish. Nguon nao chua co
        du lieu thi de gia tri mac dinh, van publish dung nhip."""


def main(args=None):
    rclpy.init(args=args)
    node = TelemetryAggregatorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
