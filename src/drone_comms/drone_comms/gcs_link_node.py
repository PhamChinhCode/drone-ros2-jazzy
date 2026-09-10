"""gcs_link_node - duong lien lac rieng Pi 4 <-> GCS qua 4G/LTE (UDP).

Doc lap voi USART3/radio telemetry cua FC (phuong an (b), muc 1 tai lieu kien truc):
khong phai sua firmware FC de forward ban tin nhiem vu tuy bien.

Hai nguyen tac bat buoc:
  1. lenh khan cap (RTH, ha canh khan, huy nhiem vu) di qua HANG DOI UU TIEN RIENG -
     khong xep sau cac goi telemetry thuong;
  2. TU phat hien mat ket noi bang watchdog - khong doi GCS bao, vi luc mat ket noi
     GCS khong bao duoc gi ca.
"""

import queue

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool

from drone_comms.qos import EVENT_QOS
from drone_interfaces.msg import MissionPlan, TelemetryPacket

PRIORITY_EMERGENCY = 0
PRIORITY_MISSION_ACK = 1
PRIORITY_TELEMETRY = 2


class GcsLinkNode(Node):

    def __init__(self):
        super().__init__('gcs_link_node')

        self.declare_parameter('gcs_host', '127.0.0.1')
        self.declare_parameter('gcs_port', 14550)
        self.declare_parameter('local_port', 14551)
        self.declare_parameter('link_timeout_s', 5.0)
        self.declare_parameter('heartbeat_interval_s', 1.0)

        # PriorityQueue: so nho hon di truoc, nen lenh khan cap luon vuot len truoc telemetry.
        self.tx_queue = queue.PriorityQueue()
        self.tx_counter = 0
        self.socket = None
        self.last_rx_time = None
        self.connected = False

        self.create_subscription(
            TelemetryPacket, '/telemetry/outgoing', self.on_telemetry, EVENT_QOS)

        self.pub_plan = self.create_publisher(MissionPlan, '/mission/plan', EVENT_QOS)
        self.pub_connected = self.create_publisher(Bool, '/gcs_link/connected', EVENT_QOS)

        self.create_timer(0.05, self.pump_tx)          # 20 Hz rut hang doi gui di
        self.create_timer(0.5, self.check_watchdog)

    def enqueue(self, priority, payload):
        """tx_counter giu thu tu FIFO trong cung mot muc uu tien."""
        self.tx_counter += 1
        self.tx_queue.put((priority, self.tx_counter, payload))

    def on_telemetry(self, msg):
        """TODO: ma hoa TelemetryPacket thanh ban tin MAVLink tuy bien, enqueue PRIORITY_TELEMETRY."""
        del msg

    def pump_tx(self):
        """TODO: rut toi da N goi moi chu ky va gui qua socket UDP toi gcs_host:gcs_port."""

    def on_rx(self, data):
        """TODO: giai ma ban tin den; mission_plan -> publish /mission/plan;
        lenh khan cap -> xu ly ngay va enqueue ACK o PRIORITY_EMERGENCY. Cap nhat last_rx_time."""
        del data

    def check_watchdog(self):
        """TODO: qua link_timeout_s khong nhan duoc gi -> publish /gcs_link/connected = false."""


def main(args=None):
    rclpy.init(args=args)
    node = GcsLinkNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
