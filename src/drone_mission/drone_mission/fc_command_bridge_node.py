"""fc_command_bridge_node - diem DUY NHAT goi lenh COMMAND_LONG xuong FC qua MAVROS.

Tap trung mot cho de de log va validate, thay vi de cac node khac goi rai rac.

Bat buoc: gan seq tang dan cho moi lenh, cho ACK trong ack_timeout_s, va BAO LOI RO RANG
thay vi treo vo han khi FC khong tra loi.

Theo docs/GIAO_UOC_FC_ROS2.md (hop dong 1.3):
- ARM/DISARM gui MAV_CMD_COMPONENT_ARM_DISARM qua /mavros/cmd/command, doc truong result
  (0 ACCEPTED, 1 TEMPORARILY_REJECTED, 2 DENIED) - khong chi success (6.2).
- Pi mat quyen / khong biet quyen -> KHONG gui lenh nao, ke ca DISARM (6.3); ARM chi gui khi
  OB_ARM_RDY = 1.
- DISARM thuong chi duoc FC nhan khi <= 20 cm tren mat dat. Cat khan cap (param2 = 21196) la
  service rieng ~/emergency_disarm, CHI GCS goi (11.1 #12f).
- Khong co NAV_TAKEOFF (FC tra UNSUPPORTED) va KHONG goi SET_MODE (5.1): cat canh, di chuyen,
  ha canh deu la setpoint van toc qua position_controller_node.
"""

import threading

import rclpy
from mavros_msgs.msg import DebugValue, State
from mavros_msgs.srv import CommandLong
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

from drone_interfaces.srv import Arm, EmergencyDisarm, FcSimpleCommand, GotoWaypoint, Takeoff
from drone_mission import fc_link
from drone_mission.qos import EVENT_QOS

# 11 ten NAMED_VALUE_INT toi mot cum moi 0,5 s - depth nho se mat OB_AUTH (giao uoc 4.1, P8).
NAMED_VALUE_QOS = QoSProfile(depth=20, reliability=ReliabilityPolicy.BEST_EFFORT)

RESULT_NOT_SENT = -1


class FcCommandBridgeNode(Node):

    def __init__(self):
        super().__init__('fc_command_bridge_node')

        self.declare_parameter('ack_timeout_s', 6.0)

        self.seq_counter = 0
        self.fc_state = None
        self.fc_status = fc_link.FcStatus()
        self.command_lock = threading.Lock()   # moi luc chi mot lenh cho ACK

        cb = ReentrantCallbackGroup()
        self.create_subscription(State, '/mavros/state', self.on_fc_state, EVENT_QOS,
                                 callback_group=cb)
        self.create_subscription(DebugValue, '/mavros/debug_value/named_value_int',
                                 self.on_named_value, NAMED_VALUE_QOS, callback_group=cb)

        self.cli_command = self.create_client(
            CommandLong, '/mavros/cmd/command', callback_group=cb)

        self.create_service(Arm, '~/arm', self.on_arm, callback_group=cb)
        self.create_service(EmergencyDisarm, '~/emergency_disarm', self.on_emergency_disarm,
                            callback_group=cb)
        self.create_service(Takeoff, '~/takeoff', self.on_takeoff, callback_group=cb)
        self.create_service(GotoWaypoint, '~/goto_waypoint', self.on_goto_waypoint,
                            callback_group=cb)
        self.create_service(FcSimpleCommand, '~/simple_command', self.on_simple_command,
                            callback_group=cb)

    def next_seq(self):
        self.seq_counter += 1
        return self.seq_counter

    def now_s(self):
        return self.get_clock().now().nanoseconds / 1e9

    def on_fc_state(self, msg):
        self.fc_state = msg

    def on_named_value(self, msg):
        self.fc_status.update(msg.name, msg.value_int, self.now_s())

    def armed(self):
        """True/False theo HEARTBEAT; None khi MAVROS chua noi FC."""
        if self.fc_state is None or not self.fc_state.connected:
            return None
        return self.fc_state.armed

    def send_arm_disarm(self, arm, param2):
        """Gui COMMAND_LONG 400 va cho ACK. Tra (result, message)."""
        if not self.cli_command.service_is_ready():
            return RESULT_NOT_SENT, 'MAVROS chua san sang (/mavros/cmd/command)'
        req = CommandLong.Request()
        req.command = fc_link.MAV_CMD_COMPONENT_ARM_DISARM
        # confirmation != 0 BAT BUOC: FC khai MAV_AUTOPILOT_GENERIC nen MAVROS chi cho COMMAND_ACK
        # khi confirmation != 0; bang 0 no tu tra ACCEPTED ngay, khong doi FC (do 09-14).
        req.confirmation = 1
        req.param1 = 1.0 if arm else 0.0
        req.param2 = float(param2)

        timeout = self.get_parameter('ack_timeout_s').value
        with self.command_lock:
            done = threading.Event()
            future = self.cli_command.call_async(req)
            future.add_done_callback(lambda _: done.set())
            if not done.wait(timeout):
                future.cancel()
                return RESULT_NOT_SENT, f'khong co ACK sau {timeout:.1f} s'
        resp = future.result()
        if not resp.success and resp.result == 0:
            # MAVROS bao that bai ma khong co ma ACK - vd het thoi gian cho phia MAVROS.
            return RESULT_NOT_SENT, 'MAVROS bao that bai, khong co COMMAND_ACK'
        return resp.result, ''

    def on_arm(self, request, response):
        response.seq = self.next_seq()
        verb = 'ARM' if request.arm else 'DISARM'
        refusal = fc_link.local_refusal(request.arm, self.armed(), self.fc_status, self.now_s())
        if refusal:
            response.success, response.result = False, RESULT_NOT_SENT
            response.message = f'{verb} khong gui: {refusal}'
            self.get_logger().warn(f'[seq {response.seq}] {response.message}')
            return response

        result, err = self.send_arm_disarm(request.arm, 0)
        response.result = result
        response.success = result == fc_link.MAV_RESULT_ACCEPTED
        response.message = err or self.describe_result(verb, result)
        self.log_result(response)
        return response

    def on_emergency_disarm(self, request, response):
        """Cat ngay o moi do cao. Van ton trong quyen: FC tra DENIED khi Pi mat quyen (D8)."""
        response.seq = self.next_seq()
        self.get_logger().error(f'[seq {response.seq}] CAT KHAN CAP (21196): {request.reason}')
        result, err = self.send_arm_disarm(False, fc_link.DISARM_FORCE_MAGIC)
        response.result = result
        response.success = result == fc_link.MAV_RESULT_ACCEPTED
        response.message = err or self.describe_result('DISARM 21196', result)
        self.log_result(response)
        return response

    def log_result(self, response):
        # Moi muc log mot dong rieng: rclpy cam mot cho goi log doi muc giua cac lan goi
        # (ValueError lam node chet - da xay ra 09-14 khi DISARM bi tu choi sau ARM thanh cong).
        text = f'[seq {response.seq}] {response.message}'
        if response.success:
            self.get_logger().info(text)
        else:
            self.get_logger().warning(text)

    @staticmethod
    def describe_result(verb, result):
        if result == fc_link.MAV_RESULT_ACCEPTED:
            return f'{verb}: ACCEPTED'
        if result == fc_link.MAV_RESULT_TEMPORARILY_REJECTED:
            return f'{verb}: TEMPORARILY_REJECTED - chua du dieu kien, thu lai sau'
        if result == fc_link.MAV_RESULT_DENIED:
            return f'{verb}: DENIED - Pi khong co quyen, dung thu lai, cho nguoi lai'
        return f'{verb}: MAV_RESULT {result}'

    def on_takeoff(self, request, response):
        """FC khong ho tro NAV_TAKEOFF (giao uoc 5.1) - cat canh la setpoint van toc len."""
        del request
        response.success = False
        response.message = ('khong ho tro: FC tra UNSUPPORTED cho NAV_TAKEOFF, '
                            'cat canh bang setpoint')
        response.seq = self.next_seq()
        return response

    def on_goto_waypoint(self, request, response):
        """TODO: chuyen target_ned/yaw_deg thanh setpoint cho position_controller_node
        (vd publish /mission/setpoint), KHONG tu publish /mavros/setpoint_raw/local -
        node do moi la chu setpoint duy nhat (mat 2 doc thiet ke). Tra ve ngay."""
        del request
        response.success = False
        response.message = 'chua duoc hien thuc'
        response.seq = self.next_seq()
        return response

    def on_simple_command(self, request, response):
        """TODO: map HOLD/PRECISION_LAND/LAND_NOW/RTH sang setpoint qua position_controller_node.
        KHONG goi SET_MODE (giao uoc 5.1): FC khong hien thuc, MAVROS se treo toi timeout."""
        del request
        response.success = False
        response.message = 'chua duoc hien thuc'
        response.seq = self.next_seq()
        return response


def main(args=None):
    rclpy.init(args=args)
    node = FcCommandBridgeNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
