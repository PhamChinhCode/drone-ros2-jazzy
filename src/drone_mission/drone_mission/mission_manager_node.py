"""mission_manager_node - trai tim logic cap cao, boc lop MissionFsm vao ROS.

Node nay chi lam ba viec: gom input thanh Snapshot, goi FSM, dich Action ra service/topic.
Toan bo logic chuyen trang thai nam trong mission_fsm.py de test duoc khong can ROS.
"""

import rclpy
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import BatteryState
from std_msgs.msg import Bool, Int32

from drone_interfaces.msg import (FailsafeEvent, GripperCommand, GripperStatus,
                                  MarkerQuality, MissionPlan, MissionState)
from drone_interfaces.srv import Arm, FcSimpleCommand, GotoWaypoint, Takeoff
from drone_mission import mission_fsm
from drone_mission.qos import EVENT_QOS, SENSOR_QOS


class MissionManagerNode(Node):

    def __init__(self):
        super().__init__('mission_manager_node')

        self.declare_parameter('search_timeout_s', 20.0)
        self.declare_parameter('max_retries', 3)
        self.declare_parameter('acceptance_radius_m', 1.5)
        # Giu on dinh truoc khi mo gripper de khong tha hang khi con dang dao dong.
        self.declare_parameter('pre_dropoff_settle_s', 2.0)
        self.declare_parameter('takeoff_alt_m', 5.0)
        self.declare_parameter('state_publish_rate_hz', 5.0)

        self.fsm = mission_fsm.MissionFsm(params=mission_fsm.Params(
            search_timeout_s=self.get_parameter('search_timeout_s').value,
            max_retries=self.get_parameter('max_retries').value,
            acceptance_radius_m=self.get_parameter('acceptance_radius_m').value,
            pre_dropoff_settle_s=self.get_parameter('pre_dropoff_settle_s').value,
            takeoff_alt_m=self.get_parameter('takeoff_alt_m').value))

        self.snapshot = mission_fsm.Snapshot()
        self.plan = None
        self.gripper_seq = 0

        self.create_subscription(MissionPlan, '/mission/plan', self.on_plan, EVENT_QOS)
        self.create_subscription(State, '/mavros/state', self.on_fc_state, EVENT_QOS)
        self.create_subscription(BatteryState, '/mavros/battery', self.on_battery, SENSOR_QOS)
        self.create_subscription(Odometry, '/odometry/filtered', self.on_odom, SENSOR_QOS)
        self.create_subscription(Bool, '/landing_target/lost', self.on_target_lost, EVENT_QOS)
        self.create_subscription(MarkerQuality, '/marker/tracking_quality', self.on_marker, EVENT_QOS)
        self.create_subscription(GripperStatus, '/gripper/status', self.on_gripper, EVENT_QOS)
        self.create_subscription(FailsafeEvent, '/failsafe_event', self.on_failsafe, EVENT_QOS)

        self.pub_state = self.create_publisher(MissionState, '/mission/state', EVENT_QOS)
        self.pub_expected_id = self.create_publisher(Int32, '/mission/expected_marker_id', EVENT_QOS)
        self.pub_setpoint = self.create_publisher(PoseStamped, '/mission/setpoint', EVENT_QOS)
        self.pub_gripper = self.create_publisher(GripperCommand, '/gripper/command', EVENT_QOS)

        # Moi lenh xuong FC deu di qua fc_command_bridge_node, khong goi MAVROS truc tiep.
        self.cli_arm = self.create_client(Arm, '/fc_command_bridge_node/arm')
        self.cli_takeoff = self.create_client(Takeoff, '/fc_command_bridge_node/takeoff')
        self.cli_goto = self.create_client(GotoWaypoint, '/fc_command_bridge_node/goto_waypoint')
        self.cli_simple = self.create_client(
            FcSimpleCommand, '/fc_command_bridge_node/simple_command')

        self.create_timer(1.0 / self.get_parameter('state_publish_rate_hz').value, self.tick)

    def on_plan(self, msg):
        """TODO: xac thuc ke hoach (it nhat 1 waypoint, action hop le), nap vao FSM, ACK ve GCS."""
        del msg

    def on_fc_state(self, msg):
        self.snapshot.armed = msg.armed
        self.snapshot.fc_connected = msg.connected

    def on_battery(self, msg):
        self.snapshot.battery_pct = msg.percentage * 100.0

    def on_odom(self, msg):
        """TODO: tinh at_waypoint = khoang cach toi waypoint hien tai < acceptance_radius_m."""
        del msg

    def on_target_lost(self, msg):
        self.snapshot.landing_target_lost = msg.data

    def on_marker(self, msg):
        """TODO: marker_confirmed = visible va marker_id khop waypoint hien tai."""
        del msg

    def on_gripper(self, msg):
        self.snapshot.gripper_sensor_confirmed = msg.sensor_confirmed

    def on_failsafe(self, msg):
        self.snapshot.failsafe_escalate_to = msg.escalate_to if msg.active else 0

    def tick(self):
        """TODO: cap nhat now_s, goi self.fsm.step(self.snapshot), dich Action ra service/topic
        tuong ung, roi publish MissionState. Publish state DEU dan ke ca khi FSM khong doi."""

    def publish_mission_state(self):
        """TODO: dich fsm.state (chuoi) sang hang so MissionState.* va publish."""


def main(args=None):
    rclpy.init(args=args)
    node = MissionManagerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
