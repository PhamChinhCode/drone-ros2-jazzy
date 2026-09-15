"""failsafe_monitor_node - giam sat tap trung, noi DUY NHAT duoc CHU DONG yeu cau doi trang
thai nhiem vu khi co su co. Khong chi canh bao suong roi cho node khac tu quyet.

Cau truc bat buoc: kiem tra DINH KY BANG TIMER, khong kiem theo callback le te - de khong
su co nao lot chi vi khong co su kien moi kich hoat kiem tra.

Nguong phai KHOP bang system_config phia GCS. Lech nguong hai ben gay hoang mang nguoi van
hanh (GCS nghi pin con an toan trong khi Pi 4 da kich RTH).

"Chu dong yeu cau" = phat /failsafe_event (active + escalate_to); mission_manager_node uu tien
tuyet doi su kien nay truoc moi viec khac. KHONG goi fc_command_bridge_node truc tiep: moi lenh
xuong FC chi di tu mission_manager_node / position_controller_node (giao uoc 5.3), va FC khong co
lenh RTH/HOLD (5.1) - leo thang la doi trang thai FSM. Quy tac o failsafe_rules.py.
"""

import rclpy
from mavros_msgs.msg import State
from rclpy.experimental import EventsExecutor
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import BatteryState
from std_msgs.msg import Bool

from drone_interfaces.msg import EkfHealth, FailsafeEvent, GripperStatus, MissionState
from drone_safety import failsafe_rules as rules
from drone_safety.qos import EVENT_QOS, SENSOR_QOS

# /ekf/health phat 5 Hz: da tung nhan ma im qua muc nay = ekf_health_node chet -> khong healthy.
EKF_HEALTH_STALE_S = 1.0

STATE_NAMES = {getattr(MissionState, n): n for n in (
    'IDLE', 'TAKEOFF', 'ENROUTE', 'MARKER_SEARCH', 'PRECISION_LAND', 'ACTUATE_GRIPPER',
    'RETRY_LOITER', 'RTH', 'EMERGENCY_LAND', 'MISSION_COMPLETE', 'FAILSAFE')}


class FailsafeMonitorNode(Node):

    def __init__(self):
        super().__init__('failsafe_monitor_node')

        self.declare_parameter('low_battery_pct', 25.0)
        self.declare_parameter('critical_battery_pct', 15.0)
        self.declare_parameter('link_lost_timeout_s', 10.0)
        self.declare_parameter('marker_search_timeout_s', 20.0)
        self.declare_parameter('fc_comm_timeout_s', 3.0)
        # Phat hien gripper khong xac nhan - KHONG thay cho sensor_confirmed (nguyen tac 3 FSM).
        self.declare_parameter('grip_confirm_timeout_s', 5.0)
        self.declare_parameter('max_retries', 3)
        self.declare_parameter('check_rate_hz', 2.0)

        g = lambda k: self.get_parameter(k).value  # noqa: E731
        self.limits = rules.Limits(g('low_battery_pct'), g('critical_battery_pct'),
                                   g('link_lost_timeout_s'), g('marker_search_timeout_s'),
                                   g('fc_comm_timeout_s'), g('grip_confirm_timeout_s'))

        self.node_start_s = self.now_s()
        self.battery_pct = None
        self.ekf_healthy = None
        self.ekf_health_s = None
        self.last_fc_heartbeat_s = None
        self.gcs_disconnected_since_s = None
        self.mission_state = None
        self.gripper = None
        self.active_events = {}          # fs_type -> escalate_to dang bat, tranh spam lap lai

        self.create_subscription(BatteryState, '/mavros/battery', self.on_battery, SENSOR_QOS)
        self.create_subscription(State, '/mavros/state', self.on_fc_state, EVENT_QOS)
        self.create_subscription(Bool, '/gcs_link/connected', self.on_gcs_link, EVENT_QOS)
        self.create_subscription(EkfHealth, '/ekf/health', self.on_ekf_health, EVENT_QOS)
        self.create_subscription(GripperStatus, '/gripper/status', self.on_gripper, EVENT_QOS)
        self.create_subscription(MissionState, '/mission/state', self.on_mission_state, EVENT_QOS)

        self.pub_event = self.create_publisher(FailsafeEvent, '/failsafe_event', EVENT_QOS)

        self.create_timer(1.0 / self.get_parameter('check_rate_hz').value, self.periodic_check)

    def now_s(self):
        return self.get_clock().now().nanoseconds / 1e9

    def on_battery(self, msg):
        # FC gui battery_remaining = -1 ("khong biet", co y) -> MAVROS cho percentage < 0.
        # Coi la khong biet (None), KHONG phai 0% - neu khong periodic_check se hieu
        # battery_pct = -1.0 < critical_battery_pct va kich ESCALATE_EMERGENCY_LAND oan.
        self.battery_pct = msg.percentage * 100.0 if msg.percentage >= 0.0 else None

    def on_fc_state(self, msg):
        if msg.connected:
            self.last_fc_heartbeat_s = self.now_s()

    def on_gcs_link(self, msg):
        # Chi tinh mat GCS khi gcs_link_node BAO false; chua co node do thi khong suy ra su co.
        if msg.data:
            self.gcs_disconnected_since_s = None
        elif self.gcs_disconnected_since_s is None:
            self.gcs_disconnected_since_s = self.now_s()

    def on_ekf_health(self, msg):
        self.ekf_healthy = msg.healthy
        self.ekf_health_s = self.now_s()

    def on_gripper(self, msg):
        self.gripper = msg

    def on_mission_state(self, msg):
        self.mission_state = msg

    def periodic_check(self):
        """Danh gia quy tac, phat su kien khi mot su co BAT, DOI MUC hoac HET (khong spam)."""
        ms = self.mission_state
        now = self.now_s()
        ekf_healthy = self.ekf_healthy
        if self.ekf_health_s is not None and now - self.ekf_health_s > EKF_HEALTH_STALE_S:
            ekf_healthy = False
        inp = rules.Inputs(
            now_s=now, node_start_s=self.node_start_s, battery_pct=self.battery_pct,
            ekf_healthy=ekf_healthy, fc_heartbeat_s=self.last_fc_heartbeat_s,
            gcs_disconnected_since_s=self.gcs_disconnected_since_s,
            mission_state=None if ms is None else STATE_NAMES.get(ms.state),
            state_entered_s=(None if ms is None
                             else Time.from_msg(ms.state_entered_stamp).nanoseconds / 1e9),
            gripper_confirmed=None if self.gripper is None else self.gripper.sensor_confirmed)
        current = rules.evaluate(inp, self.limits)

        for fs_type, (escalate, detail) in current.items():
            if self.active_events.get(fs_type) != escalate:
                self.raise_failsafe(fs_type, escalate, detail)
        for fs_type in list(self.active_events):
            if fs_type not in current:
                self.clear_failsafe(fs_type)

    def raise_failsafe(self, fs_type, escalate_to, detail=''):
        self.active_events[fs_type] = escalate_to
        self.publish_event(fs_type, escalate_to, True, detail)
        self.get_logger().warning(f'FAILSAFE {fs_type} -> leo thang {escalate_to}: {detail}')

    def clear_failsafe(self, fs_type):
        self.active_events.pop(fs_type)
        self.publish_event(fs_type, FailsafeEvent.ESCALATE_NONE, False, 'het su co')
        self.get_logger().info(f'FAILSAFE {fs_type} het')

    def publish_event(self, fs_type, escalate_to, active, detail):
        msg = FailsafeEvent()
        msg.type = fs_type
        msg.escalate_to = escalate_to
        msg.active = active
        msg.detail = detail
        msg.stamp = self.get_clock().now().to_msg()
        self.pub_event.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = FailsafeMonitorNode()
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
