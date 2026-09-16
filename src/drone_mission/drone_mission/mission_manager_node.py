"""mission_manager_node - trai tim logic cap cao, boc lop MissionFsm vao ROS.

Node nay chi lam ba viec: gom input thanh Snapshot, goi FSM, dich Action ra service/topic.
Toan bo logic chuyen trang thai nam trong mission_fsm.py de test duoc khong can ROS.
"""

import math

import rclpy
from geometry_msgs.msg import PoseStamped, TwistStamped
from mavros_msgs.msg import DebugValue, PositionTarget, State
from nav_msgs.msg import Odometry
from rclpy.experimental import EventsExecutor
from rclpy.exceptions import ParameterUninitializedException
from rclpy.node import Node
from rclpy.parameter import Parameter
from sensor_msgs.msg import BatteryState, Range
from std_msgs.msg import Bool, Float32, Int32
from std_srvs.srv import Trigger

from drone_interfaces.msg import (EkfHealth, FailsafeEvent, GripperCommand, GripperStatus, MissionPlan, MissionPlanAck, MissionState)
from drone_interfaces.srv import Arm, FcSimpleCommand, GotoWaypoint, Takeoff
from drone_mission import fc_link, mission_fsm
from drone_mission.fc_command_bridge_node import NAMED_VALUE_QOS
from drone_mission.landing_detector import LandingDetector
from drone_mission.qos import EVENT_QOS, SENSOR_QOS

# ODOMETRY cua FC: covariance vz = 1e6 nghia la van toc dung khong hop le (giao uoc 11.2).
COVARIANCE_INVALID = 1e5
VZ_COVARIANCE_INDEX = 2 * 6 + 2
RANGE_STALE_S = 0.5             # laser 20 Hz
TARGET_STALE_S = 0.5            # pose tag ~25 Hz tu landing_target_bridge_node
ODOM_STALE_S = 0.5              # EKF 30 Hz


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
        # Ban do tag dung chung (config/tags.yaml). Khong co GPS: vi tri waypoint suy tu day.
        self.declare_parameter('known_tags', Parameter.Type.DOUBLE_ARRAY)

        self.fsm = mission_fsm.MissionFsm(params=mission_fsm.Params(
            search_timeout_s=self.get_parameter('search_timeout_s').value,
            max_retries=self.get_parameter('max_retries').value,
            acceptance_radius_m=self.get_parameter('acceptance_radius_m').value,
            pre_dropoff_settle_s=self.get_parameter('pre_dropoff_settle_s').value,
            takeoff_alt_m=self.get_parameter('takeoff_alt_m').value))

        self.snapshot = mission_fsm.Snapshot()
        try:
            self.known_tags = mission_fsm.parse_known_tags(self.get_parameter('known_tags').value)
        except ParameterUninitializedException:
            self.known_tags = {}
            self.get_logger().warning(
                'chua nap config/tags.yaml (known_tags) - moi ke hoach se bi tu choi')
        self.plan = None
        self.gripper_seq = 0
        self.landing_detector = LandingDetector()
        self.fc_vz_mps = None
        self.cmd_vz_mps = None
        self.fc_status = fc_link.FcStatus()
        self.range_m = None
        self.range_stamp_s = None
        self.failsafe_active = {}         # FailsafeEvent.type -> escalate_to dang bat
        self.target_offset_m = None
        self.target_stamp_s = None
        self.position = None
        self.position_stamp_s = None
        self.arm_future = None           # moi luc chi mot lenh ARM/DISARM dang cho ACK
        self.last_detail = ''

        self.create_subscription(MissionPlan, '/mission/plan', self.on_plan, EVENT_QOS)
        self.create_subscription(State, '/mavros/state', self.on_fc_state, EVENT_QOS)
        self.create_subscription(BatteryState, '/mavros/battery', self.on_battery, SENSOR_QOS)
        self.create_subscription(Odometry, '/odometry/filtered', self.on_odom, SENSOR_QOS)
        self.create_subscription(Bool, '/landing_target/lost', self.on_target_lost, EVENT_QOS)
        # Pose tag da xac thuc ID (base_link) - FSM chi can do lech ngang de quyet dinh xuong.
        self.create_subscription(
            PoseStamped, '/landing_target/pose', self.on_landing_target_pose, SENSOR_QOS)
        self.create_subscription(GripperStatus, '/gripper/status', self.on_gripper, EVENT_QOS)
        # Chi de biet odom da neo theo bang tag chua - FSM can truoc khi chot "nha" cho RTH.
        self.create_subscription(EkfHealth, '/ekf/health', self.on_ekf_health, EVENT_QOS)
        self.create_subscription(FailsafeEvent, '/failsafe_event', self.on_failsafe, EVENT_QOS)
        # Tieu chi cham dat 11.1 #12e: laser, vz cua FC, vz Pi dang ra lenh (chi doc, khong publish).
        self.create_subscription(Range, '/mavros/mtf01p', self.on_range, SENSOR_QOS)
        self.create_subscription(Odometry, '/mavros/odometry/in', self.on_fc_odom, SENSOR_QOS)
        self.create_subscription(
            PositionTarget, '/mavros/setpoint_raw/local', self.on_cmd_setpoint, SENSOR_QOS)
        self.create_subscription(DebugValue, '/mavros/debug_value/named_value_int',
                                 self.on_named_value, NAMED_VALUE_QOS)

        self.pub_state = self.create_publisher(MissionState, '/mission/state', EVENT_QOS)
        self.pub_expected_id = self.create_publisher(Int32, '/mission/expected_marker_id', EVENT_QOS)
        self.pub_setpoint = self.create_publisher(PoseStamped, '/mission/setpoint', EVENT_QOS)
        # Tran toc do ngang cua waypoint dang bay toi, di kem /mission/setpoint. Tach topic
        # rieng thay vi them truong vao MissionState: MissionState nam trong giao uoc FC/GCS.
        self.pub_max_vel = self.create_publisher(Float32, '/mission/max_vel', EVENT_QOS)
        # Phan quyet ve ke hoach, cho gcs_link_node dich thanh DRONE_MISSION_ACK (muc 3.2).
        self.pub_plan_ack = self.create_publisher(
            MissionPlanAck, '/mission/plan_ack', EVENT_QOS)
        self.pub_gripper = self.create_publisher(GripperCommand, '/gripper/command', EVENT_QOS)
        # Lenh van toc FLU cho position_controller_node (cat/ha canh khong can gain PID vi tri).
        self.pub_velocity = self.create_publisher(
            TwistStamped, '/mission/velocity_setpoint', SENSOR_QOS)

        # Cat canh = ARM roi leo; KHONG co lenh cat canh o FC (5.1). Ha canh xong tu DISARM thuong.
        self.create_service(Trigger, '~/start', self.on_start)
        self.create_service(Trigger, '~/land', self.on_land)
        # Hai service cho gcs_link_node dich MAV_CMD 20 va 42100 (giao uoc GCS muc 4.1).
        self.create_service(Trigger, '~/rth', self.on_rth)
        self.create_service(Trigger, '~/abort', self.on_abort)

        # Moi lenh xuong FC deu di qua fc_command_bridge_node, khong goi MAVROS truc tiep.
        self.cli_arm = self.create_client(Arm, '/fc_command_bridge_node/arm')
        self.cli_takeoff = self.create_client(Takeoff, '/fc_command_bridge_node/takeoff')
        self.cli_goto = self.create_client(GotoWaypoint, '/fc_command_bridge_node/goto_waypoint')
        self.cli_simple = self.create_client(
            FcSimpleCommand, '/fc_command_bridge_node/simple_command')

        self.create_timer(1.0 / self.get_parameter('state_publish_rate_hz').value, self.tick)

    def on_plan(self, msg):
        """Kiem tra va nap ke hoach vao FSM (chi khi IDLE). KHONG tu cat canh - van can ~/start.

        Ket qua bao qua /mission/plan_ack (cho gcs_link_node dich len GCS), /mission/state
        (mission_id + detail) va log.
        """
        raw = [dict(seq=w.seq, marker_id=w.expected_marker_id, action=w.action, alt_m=w.alt_m,
                    acceptance_radius_m=w.acceptance_radius_m, max_vel_mps=w.max_vel_mps,
                    loiter_s=w.loiter_s) for w in msg.waypoints]
        refusal = self.fsm.load_plan(msg.mission_id, raw, msg.max_retries, msg.search_timeout_s,
                                     self.known_tags)
        ack = MissionPlanAck()
        ack.mission_id = msg.mission_id
        ack.accepted = not refusal
        ack.reason = refusal
        ack.stamp = self.get_clock().now().to_msg()
        self.pub_plan_ack.publish(ack)
        if refusal:
            detail = f'tu choi ke hoach {msg.mission_id}: {refusal}'
            self.get_logger().warning(detail)
        else:
            markers = [w.marker_id for w in self.fsm.waypoints]
            detail = (f'nhan ke hoach {msg.mission_id} "{msg.plan_name}": {len(markers)} diem, '
                      f'tag {markers} - goi ~/start de bay')
            self.get_logger().info(detail)
        self.publish_mission_state(detail)

    def now_s(self):
        return self.get_clock().now().nanoseconds / 1e9

    def on_start(self, request, response):
        del request
        refusal = self.fsm.request_start(self.now_s())
        response.success = not refusal
        response.message = refusal or 'nhan yeu cau cat canh'
        self.get_logger().info(f'~/start: {response.message}')
        return response

    def on_land(self, request, response):
        del request
        self.fsm.request_land()
        response.success, response.message = True, 'nhan yeu cau ha canh'
        self.get_logger().info(f'~/land: {response.message}')
        return response

    def on_rth(self, request, response):
        """MAV_CMD_NAV_RETURN_TO_LAUNCH. Tu choi SOM neu chua biet nha, khong nhan roi im lang."""
        del request
        ly_do = self.fsm.request_rth()
        response.success = not ly_do
        response.message = ly_do or 'nhan yeu cau ve nha'
        (self.get_logger().warning if ly_do else self.get_logger().info)(f'~/rth: {response.message}')
        return response

    def on_abort(self, request, response):
        """MAV_CMD_DRONE_ABORT_MISSION. KHONG bao gio tu choi - huy phai luon di duoc."""
        del request
        self.fsm.request_abort()
        response.success = True
        response.message = 'nhan yeu cau huy nhiem vu'
        self.get_logger().warning('~/abort: nhan yeu cau huy nhiem vu')
        return response

    def on_landing_target_pose(self, msg):
        self.target_offset_m = math.hypot(msg.pose.position.x, msg.pose.position.y)
        self.target_stamp_s = self.now_s()

    def on_named_value(self, msg):
        self.fc_status.update(msg.name, msg.value_int, self.now_s())

    def on_fc_state(self, msg):
        self.snapshot.armed = msg.armed
        self.snapshot.fc_connected = msg.connected

    def on_battery(self, msg):
        self.snapshot.battery_pct = msg.percentage * 100.0

    def on_odom(self, msg):
        p = msg.pose.pose.position
        self.position = (p.x, p.y, p.z)
        self.position_stamp_s = self.now_s()

    def on_fc_odom(self, msg):
        valid = msg.twist.covariance[VZ_COVARIANCE_INDEX] < COVARIANCE_INVALID
        self.fc_vz_mps = msg.twist.twist.linear.z if valid else None

    def on_cmd_setpoint(self, msg):
        self.cmd_vz_mps = msg.velocity.z

    def on_range(self, msg):
        range_m = msg.range if msg.min_range <= msg.range <= msg.max_range else None
        now_s = self.now_s()
        self.range_m, self.range_stamp_s = range_m, now_s
        self.landing_detector.update_ground_ref(
            self.snapshot.armed, range_m, self.fc_vz_mps, now_s)
        self.snapshot.landed = self.landing_detector.step(
            range_m, self.fc_vz_mps, self.cmd_vz_mps, now_s)

    def on_target_lost(self, msg):
        self.snapshot.landing_target_lost = msg.data

    def on_ekf_health(self, msg):
        self.snapshot.pos_anchored = msg.anchored

    def on_gripper(self, msg):
        self.snapshot.gripper_sensor_confirmed = msg.sensor_confirmed

    def on_failsafe(self, msg):
        # Nhieu su co cung luc: giu tung loai, FSM nhan muc NANG NHAT; het mot loai khong xoa
        # loai khac.
        if msg.active:
            self.failsafe_active[msg.type] = msg.escalate_to
        else:
            self.failsafe_active.pop(msg.type, None)
        self.snapshot.failsafe_escalate_to = max(self.failsafe_active.values(), default=0)

    def tick(self):
        """Gom Snapshot, goi FSM, dich Action, publish MissionState deu dan ke ca khi khong doi."""
        now = self.now_s()
        snap = self.snapshot
        snap.now_s = now
        snap.ob_auth = self.fc_status.has_authority(now)
        snap.ob_state = self.fc_status.get('OB_STATE', now)
        snap.arm_ready = self.fc_status.arm_ready(now)
        snap.disarm_ready = self.fc_status.disarm_ready(now)
        fresh = self.range_stamp_s is not None and now - self.range_stamp_s <= RANGE_STALE_S
        snap.range_m = self.range_m if fresh else None
        target_fresh = (self.target_stamp_s is not None and not snap.landing_target_lost
                        and now - self.target_stamp_s <= TARGET_STALE_S)
        snap.target_offset_m = self.target_offset_m if target_fresh else None
        odom_fresh = self.position_stamp_s is not None and now - self.position_stamp_s <= ODOM_STALE_S
        snap.position = self.position if odom_fresh else None

        prev_state = self.fsm.state
        action = self.fsm.step(snap)
        if self.fsm.state != prev_state:
            self.get_logger().info(f'{prev_state} -> {self.fsm.state}: {action.detail}')
        elif action.detail != self.last_detail and action.detail:
            self.get_logger().info(f'{self.fsm.state}: {action.detail}')
        self.last_detail = action.detail

        if action.fc_command in ('arm', 'disarm'):
            self.send_arm(action.fc_command == 'arm')
        if action.gripper_command in ('open', 'close'):
            self.send_gripper(action.gripper_command)
        self.pub_expected_id.publish(Int32(data=action.expected_marker_id))
        if action.velocity_up_mps is not None:
            msg = TwistStamped()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = 'base_link'
            msg.twist.linear.z = float(action.velocity_up_mps)
            self.pub_velocity.publish(msg)
        if action.position_target is not None:
            msg = PoseStamped()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = 'odom'
            msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = action.position_target
            msg.pose.orientation.w = 1.0
            self.pub_setpoint.publish(msg)
            if action.max_vel_mps is not None:
                self.pub_max_vel.publish(Float32(data=float(action.max_vel_mps)))
        self.publish_mission_state(action.detail, action.expected_marker_id)

    def send_gripper(self, lenh):
        """Phat GripperCommand. FSM tu gian nhip phat lai; o day chi dich lenh va danh seq."""
        self.gripper_seq += 1
        msg = GripperCommand()
        msg.seq = self.gripper_seq
        msg.command = (GripperCommand.GRIPPER_CLOSE if lenh == 'close'
                       else GripperCommand.GRIPPER_OPEN)
        self.pub_gripper.publish(msg)
        self.get_logger().info(f'gripper {lenh.upper()} (seq {msg.seq})')

    def send_arm(self, arm):
        """Goi fc_command_bridge_node ~/arm khong chan; lenh truoc chua xong thi bo lan nay."""
        if self.arm_future is not None and not self.arm_future.done():
            return
        if not self.cli_arm.service_is_ready():
            self.get_logger().warning('fc_command_bridge_node ~/arm chua san sang')
            return
        self.arm_future = self.cli_arm.call_async(Arm.Request(arm=arm))
        self.arm_future.add_done_callback(self.on_arm_done)

    def on_arm_done(self, future):
        resp = future.result()
        if resp is None:
            return
        text = f'[seq {resp.seq}] {resp.message} (result {resp.result})'
        if resp.success:
            self.get_logger().info(text)
        else:
            self.get_logger().warning(text)

    def publish_mission_state(self, detail, expected_marker_id=-1):
        msg = MissionState()
        msg.state = getattr(MissionState, self.fsm.state)
        msg.mission_result = self.fsm.mission_result
        msg.mission_id = self.fsm.mission_id
        msg.current_wp_index = self.fsm.current_wp_index
        msg.expected_marker_id = expected_marker_id
        msg.retry_count = self.fsm.retry_count
        msg.wp_total = len(self.fsm.waypoints)
        # home = None khi chua cat canh, hoac khi cat canh luc odom chua neo theo bang tag -
        # luc do KHONG duoc dien 0, vi 0 la toa do that cua pad_home (giao uoc GCS 5.2b, P21).
        msg.home_valid = self.fsm.home is not None
        if msg.home_valid:
            msg.home_odom = [float(v) for v in self.fsm.home]
        msg.state_entered_stamp.sec = int(self.fsm.state_entered_s)
        msg.state_entered_stamp.nanosec = int((self.fsm.state_entered_s % 1.0) * 1e9)
        msg.detail = detail
        self.pub_state.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = MissionManagerNode()
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
