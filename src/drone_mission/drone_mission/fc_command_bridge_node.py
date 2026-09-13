"""fc_command_bridge_node - diem DUY NHAT goi API MAVROS.

Tap trung mot cho de de log va validate, thay vi de cac node khac goi rai rac
/mavros/cmd/arming, /mavros/set_mode.

Bat buoc: gan seq tang dan cho moi lenh, cho ACK trong ack_timeout_s, va BAO LOI RO RANG
thay vi treo vo han khi FC khong tra loi.

RUI RO LON NHAT CUA CA HE THONG: firmware FC la tuy bien, khong phai PX4/ArduPilot nguyen ban.
Phai xac minh som firmware da hien thuc du COMMAND_LONG + MAV_CMD_COMPONENT_ARM_DISARM,
SET_POSITION_TARGET_LOCAL_NED, va HEARTBEAT co base_mode/custom_mode hop le. Neu lech, node nay
boc pymavlink tho cho rieng lenh do thay vi tin plugin MAVROS.
"""

import rclpy
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, CommandTOL, SetMode
from rclpy.node import Node

from drone_interfaces.srv import Arm, FcSimpleCommand, GotoWaypoint, Takeoff
from drone_mission.qos import EVENT_QOS


class FcCommandBridgeNode(Node):

    def __init__(self):
        super().__init__('fc_command_bridge_node')

        self.declare_parameter('ack_timeout_s', 5.0)

        self.seq_counter = 0
        self.last_acked_seq = 0
        self.fc_state = None

        self.create_subscription(State, '/mavros/state', self.on_fc_state, EVENT_QOS)

        self.cli_arming = self.create_client(CommandBool, '/mavros/cmd/arming')
        self.cli_takeoff = self.create_client(CommandTOL, '/mavros/cmd/takeoff')
        self.cli_set_mode = self.create_client(SetMode, '/mavros/set_mode')

        self.create_service(Arm, '~/arm', self.on_arm)
        self.create_service(Takeoff, '~/takeoff', self.on_takeoff)
        self.create_service(GotoWaypoint, '~/goto_waypoint', self.on_goto_waypoint)
        self.create_service(FcSimpleCommand, '~/simple_command', self.on_simple_command)

    def next_seq(self):
        self.seq_counter += 1
        return self.seq_counter

    def on_fc_state(self, msg):
        self.fc_state = msg

    def on_arm(self, request, response):
        """TODO: goi /mavros/cmd/arming, cho ket qua trong ack_timeout_s, dien success/message/seq."""
        del request
        response.success = False
        response.message = 'chua duoc hien thuc'
        response.seq = self.next_seq()
        return response

    def on_takeoff(self, request, response):
        """TODO: goi /mavros/cmd/takeoff voi altitude_m."""
        del request
        response.success = False
        response.message = 'chua duoc hien thuc'
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
        """TODO: map HOLD/PRECISION_LAND/LAND_NOW/RTH sang set_mode hoac COMMAND_LONG tuong ung.
        Day la cho de kiem chung dau tien voi firmware tuy bien - ten che do co the khac PX4."""
        del request
        response.success = False
        response.message = 'chua duoc hien thuc'
        response.seq = self.next_seq()
        return response


def main(args=None):
    rclpy.init(args=args)
    node = FcCommandBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
