"""failsafe_monitor_node - giam sat tap trung, noi DUY NHAT duoc CHU DONG yeu cau doi trang
thai nhiem vu khi co su co. Khong chi canh bao suong roi cho node khac tu quyet.

Cau truc bat buoc: kiem tra DINH KY BANG TIMER, khong kiem theo callback le te - de khong
su co nao lot chi vi khong co su kien moi kich hoat kiem tra.

Nguong phai KHOP bang system_config phia GCS. Lech nguong hai ben gay hoang mang nguoi van
hanh (GCS nghi pin con an toan trong khi Pi 4 da kich RTH).
"""

import rclpy
from mavros_msgs.msg import State
from rclpy.node import Node
from sensor_msgs.msg import BatteryState
from std_msgs.msg import Bool

from drone_interfaces.msg import EkfHealth, FailsafeEvent, GripperStatus, MissionState
from drone_interfaces.srv import FcSimpleCommand
from drone_safety.qos import EVENT_QOS, SENSOR_QOS


class FailsafeMonitorNode(Node):

    def __init__(self):
        super().__init__('failsafe_monitor_node')

        self.declare_parameter('low_battery_pct', 25.0)
        self.declare_parameter('critical_battery_pct', 15.0)
        self.declare_parameter('link_lost_timeout_s', 10.0)
        self.declare_parameter('marker_search_timeout_s', 20.0)
        self.declare_parameter('fc_comm_timeout_s', 3.0)
        self.declare_parameter('max_retries', 3)
        self.declare_parameter('check_rate_hz', 2.0)

        self.battery_pct = None
        self.ekf_healthy = None
        self.gcs_connected = None
        self.last_fc_heartbeat = None
        self.mission_state = None
        self.gripper = None
        self.active_events = {}          # type -> FailsafeEvent dang bat, tranh spam lap lai

        self.create_subscription(BatteryState, '/mavros/battery', self.on_battery, SENSOR_QOS)
        self.create_subscription(State, '/mavros/state', self.on_fc_state, EVENT_QOS)
        self.create_subscription(Bool, '/gcs_link/connected', self.on_gcs_link, EVENT_QOS)
        self.create_subscription(Bool, '/landing_target/lost', self.on_target_lost, EVENT_QOS)
        self.create_subscription(EkfHealth, '/ekf/health', self.on_ekf_health, EVENT_QOS)
        self.create_subscription(GripperStatus, '/gripper/status', self.on_gripper, EVENT_QOS)
        self.create_subscription(MissionState, '/mission/state', self.on_mission_state, EVENT_QOS)

        self.pub_event = self.create_publisher(FailsafeEvent, '/failsafe_event', EVENT_QOS)
        self.cli_simple = self.create_client(
            FcSimpleCommand, '/fc_command_bridge_node/simple_command')

        self.create_timer(1.0 / self.get_parameter('check_rate_hz').value, self.periodic_check)

    def on_battery(self, msg):
        # FC gui battery_remaining = -1 ("khong biet", co y) -> MAVROS cho percentage < 0.
        # Coi la khong biet (None), KHONG phai 0% - neu khong periodic_check se hieu
        # battery_pct = -1.0 < critical_battery_pct va kich ESCALATE_EMERGENCY_LAND oan.
        self.battery_pct = msg.percentage * 100.0 if msg.percentage >= 0.0 else None

    def on_fc_state(self, msg):
        if msg.connected:
            self.last_fc_heartbeat = self.get_clock().now()

    def on_gcs_link(self, msg):
        self.gcs_connected = msg.data

    def on_target_lost(self, msg):
        """TODO: dung ket hop voi mission_state == PRECISION_LAND de phat FS_MARKER_TIMEOUT."""
        del msg

    def on_ekf_health(self, msg):
        self.ekf_healthy = msg.healthy

    def on_gripper(self, msg):
        self.gripper = msg

    def on_mission_state(self, msg):
        self.mission_state = msg

    def periodic_check(self):
        """TODO: kiem tra tung nhanh theo dung thu tu leo thang loiter -> RTH -> ha canh khan:
          battery < critical_battery_pct   -> FS_LOW_BATTERY,   ESCALATE_EMERGENCY_LAND
          battery < low_battery_pct        -> FS_LOW_BATTERY,   ESCALATE_RTH
          not ekf_healthy                  -> FS_EKF_UNHEALTHY, ESCALATE_LOITER
          mat heartbeat FC > fc_comm_timeout_s -> FS_FC_COMM_LOST, ESCALATE_EMERGENCY_LAND
          mat GCS > link_lost_timeout_s    -> FS_LINK_LOST,    ESCALATE_RTH
          MARKER_SEARCH qua marker_search_timeout_s -> FS_MARKER_TIMEOUT, ESCALATE_RETRY_LOITER
          gripper khong confirm sau khi da lenh -> FS_GRIP_CONFIRM_FAIL, ESCALATE_RETRY_LOITER
        """

    def raise_failsafe(self, fs_type, escalate_to, detail=''):
        """TODO: publish FailsafeEvent(active=true) mot lan khi vao trang thai su co, va
        publish active=false khi su co het; goi cli_simple de CHU DONG yeu cau leo thang."""
        del fs_type, escalate_to, detail


def main(args=None):
    rclpy.init(args=args)
    node = FailsafeMonitorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
