"""position_controller_node - vong PID vi tri/van toc cap companion computer.

KHAC TANG voi cascade PID goc/toc-do-goc da chay san tren STM32H743: node nay chi noi
"muon o dau / nhanh co nao", FC van tu lo giu goc nghieng on dinh.

Yeu cau bat buoc - chuyen setpoint muot: khi doi nguon setpoint (waypoint hanh trinh <->
landing target) KHONG duoc reset tich phan PID ve 0 dot ngot va KHONG duoc nhay setpoint
tuc thoi; ramp trong ramp_duration_s de tranh giat may bay.

CANH BAO AN TOAN: khong bao gio tune PID lan dau tren drone that. Tune trong Gazebo truoc,
bay that thi buoc day/long an toan, tang kp tu 0 den khi dao dong nhe roi lui lai 30-50%.

Hop dong voi FC (docs/GIAO_UOC_FC_ROS2.md 3.2, 5.2, 5.3, 6.2, 9.6):
- Node nay la publisher DUY NHAT cua /mavros/setpoint_raw/local.
- Phat DEU control_rate_hz ke ca khi chua co odom/setpoint (van toc 0 lam nhip giu cho) - FC
  chi vao OFFBOARD khi setpoint dang toi; node chet thi FC het han 500 ms va tu phanh.
- Tinh va publish theo FLU (x toi, y trai, z len), coordinate_frame = BODY_NED (8),
  type_mask = 0x07C7. MAVROS tu doi sang FRD - TUYET DOI khong tu doi dau.
- Tu kep o 95 % tran FC va xuong cham sat dat (setpoint_limits).
- Watchdog P6 (11.1 #8, 11.3): FC khong dung heartbeat Pi. /mission/state im qua
  mission_timeout_s (mission_manager_node treo/chet) -> NGUNG phat han, de FC het han 500 ms,
  phanh va lui POSHOLD. Khong phat van toc 0 thay: FC se khong bao gio het han.
"""

from concurrent.futures import ThreadPoolExecutor
import math
import signal
import threading

import rclpy
from geometry_msgs.msg import PoseStamped, TwistStamped
from mavros_msgs.msg import PositionTarget
from rcl_interfaces.msg import SetParametersResult
from nav_msgs.msg import Odometry
from rclpy.experimental import EventsExecutor
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from sensor_msgs.msg import Range
from std_msgs.msg import Float32

from drone_control.drift_comp import body_to_world, closing_speed, DriftCompensator, world_to_body
from drone_control.hold import HeadingHold, MIN_AIRBORNE_RANGE_M, PositionHold
from drone_control.pid import PID, check_gain_param
from drone_control.qos import EVENT_QOS, SENSOR_QOS
from drone_control.setpoint_limits import limit_velocity
from drone_control.setpoint_ramp import SetpointRamp
from drone_control.yaw_control import yaw_rate_command
from drone_interfaces.msg import EkfHealth, MissionState

FRAME_BODY_NED = 8
TYPE_MASK_VELOCITY_YAWRATE = 0x07C7
RANGE_STALE_S = 0.5             # laser 20 Hz; qua han coi nhu mat laser -> xuong cham
VELOCITY_STALE_S = 0.5          # mission_manager_node phat 5 Hz; qua han -> bo, ve duong vi tri/0
LANDING_STALE_S = 0.5           # pose tag ~25 Hz; bridge ngung phat khi mat tag -> qua han la bo
# mission_manager_node phat /mission/setpoint 5 Hz chi trong trang thai can bay toi diem; qua han
# -> bo, khong bam mai diem den cu sau khi nhiem vu da doi trang thai.
MISSION_SETPOINT_STALE_S = 0.5
DRIFT_LEARN_MAX_SPEED_MPS = 0.25  # bu troi chi hoc khi toc do ngang duoi muc nay
# Dich cruise phai dung yen (khong lech qua 5 mm so voi diem neo) chung nay moi duoc hoc bu troi.
# 10-10: so tung chu ky thi carrot PAD_ALIGN bo cham (< 5 mm/chu ky) van tinh la dung yen -> do tre
# bam dich (lon hon khi bat cruise.vel_damping) bi hoc thanh troi SAI DAU, day lech 17 cm luc ha.
DRIFT_LEARN_STILL_S = 1.0
# Dang tien VE dich nhanh hon muc nay thi sai so la quang duong con lai, khong phai troi - khong hoc.
# 10-11: PAD_ALIGN dung lui sau tam bai, PRECISION_LAND keo toi 0,1-0,2 m theo tag -> bu troi hoc
# nham "troi toi", cham dat lech truoc 2-4 cm (Gazebo FC moi). Troi that: dung yen lech hoac bi day
# ra xa dich (toc do tien <= 0) - van hoc.
DRIFT_LEARN_CLOSING_MPS = 0.03
ODOM_STALE_S = 0.5              # EKF 30 Hz - giu cho chi chot/bam khi odom con moi
YAW_SETPOINT_STALE_S = 0.5      # /mission/yaw 5 Hz; im = FSM khong muon quay mui -> yaw_rate 0

# Nguon van toc: truc ngang (xy) va truc dung + yaw (z) tach rieng vi tag chi thay xy.
SOURCE_NONE = 'none'
SOURCE_VELOCITY = 'velocity'
SOURCE_CRUISE = 'cruise'
SOURCE_LANDING = 'landing'
# Giu cho mac dinh (drone_control/hold.py): khong nguon nao ra lenh truc ngang -> chot vi tri EKF,
# bam bang chinh PID cruise.
SOURCE_HOLD = 'hold'
PID_PHASE = {SOURCE_CRUISE: 'cruise', SOURCE_LANDING: 'landing', SOURCE_HOLD: 'cruise'}


class PositionControllerNode(Node):

    def __init__(self):
        super().__init__('position_controller_node')

        # Gain rieng tung truc va rieng tung pha: pha ha canh thuong can dap ung nhanh hon.
        for axis in ('x', 'y', 'z'):
            for phase in ('cruise', 'landing'):
                self.declare_parameter(f'{phase}.{axis}.kp', 0.0)
                self.declare_parameter(f'{phase}.{axis}.ki', 0.0)
                self.declare_parameter(f'{phase}.{axis}.kd', 0.0)
                self.declare_parameter(f'{phase}.{axis}.i_limit', 1.0)
                self.declare_parameter(f'{phase}.{axis}.out_limit', 2.0)
        self.declare_parameter('control_rate_hz', 20.0)
        self.declare_parameter('ramp_duration_s', 0.8)
        # mission_manager_node phat /mission/state 5 Hz bang timer: 1 s = mat 5 ban lien tiep.
        self.declare_parameter('mission_timeout_s', 1.0)
        # Vong yaw (WP5): chi chay khi mission phat /mission/yaw (mission.yaml yaw_control).
        self.declare_parameter('yaw.kp', 1.0)                # 1/s
        self.declare_parameter('yaw.max_rate_dps', 30.0)
        # Giu cho mac dinh khi OFFBOARD (FC tat moc vi tri + giu huong cua no luc do - hold.py).
        # Giu vi tri dung lai PID cruise da bay. Giu huong gui yaw_rate != 0: chi bat sau khi thu
        # dau tren ban (WP5).
        self.declare_parameter('position_hold', True)
        self.declare_parameter('heading_hold', False)
        # Bu troi ngang dung chung (drift_comp.py): khau I MOT cho moi nguon truc ngang, khong xoa
        # khi doi nguon. ki = 0 tat han.
        self.declare_parameter('drift_comp.ki', 0.15)
        self.declare_parameter('drift_comp.limit_mps', 0.3)
        self.declare_parameter('drift_comp.window_m', 0.5)
        # Giam chan khi bam tag: tru landing.vel_damping x van toc than (EKF) khoi lenh vx, vy. FC
        # that dap ung van toc cham nen chi P + khau I bay vuot qua tam bai roi mat tag (10-09).
        # Dung van toc do chu khong dao ham sai so tag (nhieu, 30 Hz). 0 = tat.
        self.declare_parameter('landing.vel_damping', 0.0)
        # Cung y cho bay toi diem / giu cho (PID cruise): bay that 10-10 FC tre 1-1,5 s, PAD_ALIGN
        # vuot qua truc bai 0,4-0,5 m roi vong lai 28-60 s (tai hien Gazebo vel_delay 0,5 tau 1,5).
        self.declare_parameter('cruise.vel_damping', 0.0)

        self.pids = {phase: {axis: self._make_pid(phase, axis) for axis in ('x', 'y', 'z')}
                     for phase in ('cruise', 'landing')}
        # Doi gain khi dang chay (ros2 param set) - de tune trong Gazebo khong phai khoi dong lai.
        self.add_on_set_parameters_callback(self.on_set_parameters)

        self.odom = None
        self.odom_stamp_s = None
        self.anchored = False
        self.pos_hold = PositionHold()
        self.heading_hold = HeadingHold()
        self.drift = DriftCompensator()
        self.cruise_anchor = None           # (diem neo dich cruise, thoi diem dat neo)
        self.mission_setpoint = None
        self.mission_setpoint_stamp_s = None
        self.max_vel_mps = None
        self.max_vel_stamp_s = None
        self.landing_target = None
        self.landing_stamp_s = None
        self.active_source = (SOURCE_NONE, SOURCE_NONE)
        self.ramp = SetpointRamp(self.get_parameter('ramp_duration_s').value)
        self.mission_stamp_s = None
        self.watchdog_tripped = False
        self.range_m = None
        self.range_stamp_s = None
        self.velocity_setpoint = None
        self.velocity_stamp_s = None
        self.yaw_setpoint = None
        self.yaw_stamp_s = None

        self.create_subscription(Odometry, '/odometry/filtered', self.on_odom, SENSOR_QOS)
        self.create_subscription(Range, '/range/vertical', self.on_range, SENSOR_QOS)
        self.create_subscription(PoseStamped, '/mission/setpoint', self.on_mission_setpoint, EVENT_QOS)
        self.create_subscription(Float32, '/mission/max_vel', self.on_max_vel, EVENT_QOS)
        self.create_subscription(Float32, '/mission/yaw', self.on_yaw_setpoint, EVENT_QOS)
        # Lenh van toc FLU truc tiep (cat/ha canh) - uu tien hon duong vi tri khi con moi.
        self.create_subscription(
            TwistStamped, '/mission/velocity_setpoint', self.on_velocity_setpoint, SENSOR_QOS)
        # Pose marker da xac thuc ID tu landing_target_bridge_node (P1).
        self.create_subscription(
            PoseStamped, '/landing_target/pose', self.on_landing_target, SENSOR_QOS)

        self.create_subscription(MissionState, '/mission/state', self.on_mission_state, EVENT_QOS)
        # Chi chot moc vi tri khi odom da NEO theo bang tag - truoc do vi tri se nhay khi neo.
        self.create_subscription(EkfHealth, '/ekf/health', self.on_ekf_health, EVENT_QOS)

        self.pub_setpoint = self.create_publisher(
            PositionTarget, '/mavros/setpoint_raw/local', SENSOR_QOS)

        self.create_timer(1.0 / self.get_parameter('control_rate_hz').value, self.control_step)

    def _make_pid(self, phase, axis):
        g = lambda k: self.get_parameter(f'{phase}.{axis}.{k}').value  # noqa: E731
        return PID(g('kp'), g('ki'), g('kd'), g('i_limit'), g('out_limit'))

    def on_set_parameters(self, params):
        """Ap gain moi ngay vao PID, GIU tich phan (khong giat). Mot gia tri sai -> tu choi ca lo."""
        updates = []
        for param in params:
            pid, field, err = check_gain_param(self.pids, param.name, param.value)
            if err:
                return SetParametersResult(successful=False, reason=err)
            if pid is not None:
                updates.append((pid, field, float(param.value), param.name))
        for pid, field, value, name in updates:
            setattr(pid, field, value)
            self.get_logger().info(f'gain {name} = {value}')
        return SetParametersResult(successful=True)

    def on_odom(self, msg):
        self.odom = msg
        self.odom_stamp_s = self.get_clock().now().nanoseconds / 1e9

    def on_ekf_health(self, msg):
        self.anchored = msg.anchored

    def on_range(self, msg):
        self.range_m = msg.range if msg.min_range <= msg.range <= msg.max_range else None
        self.range_stamp_s = self.get_clock().now().nanoseconds / 1e9

    def current_range_m(self):
        now_s = self.get_clock().now().nanoseconds / 1e9
        if self.range_stamp_s is None or now_s - self.range_stamp_s > RANGE_STALE_S:
            return None
        return self.range_m

    def on_mission_setpoint(self, msg):
        self.mission_setpoint = msg
        self.mission_setpoint_stamp_s = self.get_clock().now().nanoseconds / 1e9

    def on_max_vel(self, msg):
        self.max_vel_mps = msg.data
        self.max_vel_stamp_s = self.get_clock().now().nanoseconds / 1e9

    def limit_waypoint_speed(self, now_s, vx, vy):
        """Kep toc do NGANG theo max_vel_mps cua waypoint dang bay toi.

        Kep theo DO LON vector chu khong tung truc: kep tung truc thi bay cheo van vuot gioi han
        1,41 lan. Qua han (mission ngung phat) thi bo gioi han rieng, chi con tran cua FC."""
        if (self.max_vel_mps is None or self.max_vel_mps <= 0.0
                or now_s - self.max_vel_stamp_s > MISSION_SETPOINT_STALE_S):
            return vx, vy
        speed = math.hypot(vx, vy)
        if speed <= self.max_vel_mps:
            return vx, vy
        k = self.max_vel_mps / speed
        return vx * k, vy * k

    def on_yaw_setpoint(self, msg):
        self.yaw_setpoint = msg.data
        self.yaw_stamp_s = self.get_clock().now().nanoseconds / 1e9

    def on_velocity_setpoint(self, msg):
        self.velocity_setpoint = msg
        self.velocity_stamp_s = self.get_clock().now().nanoseconds / 1e9

    def on_landing_target(self, msg):
        self.landing_target = msg
        self.landing_stamp_s = self.get_clock().now().nanoseconds / 1e9

    def on_mission_state(self, msg):
        del msg
        self.mission_stamp_s = self.get_clock().now().nanoseconds / 1e9

    def mission_alive(self, now_s):
        """Watchdog P6. Log mot lan khi mat / co lai."""
        timeout = self.get_parameter('mission_timeout_s').value
        alive = self.mission_stamp_s is not None and now_s - self.mission_stamp_s <= timeout
        if alive == self.watchdog_tripped:
            self.watchdog_tripped = not alive
            if alive:
                self.get_logger().info('co lai /mission/state - phat setpoint tro lai')
            else:
                self.get_logger().warning(
                    f'/mission/state im qua {timeout:.1f} s - NGUNG phat setpoint (FC het han 500 ms)')
        return alive

    def switch_source(self, source, errors):
        """Doi nguon: PID cua nguon vua vao reset (tich phan cu da cu, prev_error = sai so hien
        tai de khong giat D). Ngo ra khong nhay nho SetpointRamp tron tu ngo ra cu."""
        prev_xy, prev_z = self.active_source
        xy, z = source
        if xy != prev_xy and xy in PID_PHASE:
            self.pids[PID_PHASE[xy]]['x'].reset(errors[xy][0])
            self.pids[PID_PHASE[xy]]['y'].reset(errors[xy][1])
        if z != prev_z and z == SOURCE_CRUISE:
            self.pids['cruise']['z'].reset(errors['cruise'][2])
        self.active_source = source

    def control_step(self):
        """Tinh van toc FLU, tron khi doi nguon, kep, publish. Thieu moi nguon -> van toc 0.

        Thu tu: lenh van toc cua mission (con moi) > vong vi tri cruise. Rieng van toc NGANG: khi
        pose tag tu landing_target_bridge_node con moi (bridge chi phat khi mission dat
        expected_marker_id va thay dung ID) thi vx, vy = PID landing tren do lech tag trong
        base_link (SOURCE_LANDING, P1); vz van theo mission (vd PRECISION_LAND ra lenh xuong).
        Khong nguon nao ra lenh truc ngang (FSM chi ra vz) -> giu vi tri EKF (SOURCE_HOLD), khong
        ai ra lenh yaw -> giu huong (hold.py) - FC khong tu giu khi OFFBOARD.
        Chi PID cua nguon dang dung moi duoc update (khong tich phan ngam). Gain dang 0.0 - TUNE
        TRONG GAZEBO TRUOC.
        """
        now_s = self.get_clock().now().nanoseconds / 1e9
        if not self.mission_alive(now_s):
            return
        dt = 1.0 / self.get_parameter('control_rate_hz').value

        errors = {}
        target_still = False
        vel_fresh = (self.velocity_stamp_s is not None
                     and now_s - self.velocity_stamp_s <= VELOCITY_STALE_S)
        sp_fresh = (self.odom is not None and self.mission_setpoint_stamp_s is not None
                    and now_s - self.mission_setpoint_stamp_s <= MISSION_SETPOINT_STALE_S)
        if vel_fresh:
            xy = z = SOURCE_VELOCITY
            # Lenh chi co vz (PRECISION_LAND) kem diem den: truc ngang bam diem den theo EKF (tam bai
            # da biet) - mat tag ngan (bong che, 10-10) van giu tren tam ma ha tiep, khong chot cho.
            t = self.velocity_setpoint.twist
            if sp_fresh and t.linear.x == 0.0 and t.linear.y == 0.0:
                xy = SOURCE_CRUISE
        elif sp_fresh:
            xy = z = SOURCE_CRUISE
        else:
            xy = z = SOURCE_NONE
        if sp_fresh:
            p = self.odom.pose.pose
            target = self.mission_setpoint.pose.position
            tgt = (target.x, target.y)
            if self.cruise_anchor is None or math.dist(tgt, self.cruise_anchor[0]) >= 0.005:
                self.cruise_anchor = (tgt, now_s)
            target_still = now_s - self.cruise_anchor[1] >= DRIFT_LEARN_STILL_S
            # Sai so trong he odom (ENU) xoay ve he than FLU bang yaw.
            ex, ey = target.x - p.position.x, target.y - p.position.y
            q = p.orientation
            yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
            errors[SOURCE_CRUISE] = (math.cos(yaw) * ex + math.sin(yaw) * ey,
                                     -math.sin(yaw) * ex + math.cos(yaw) * ey,
                                     target.z - p.position.z)
        if self.landing_stamp_s is not None and now_s - self.landing_stamp_s <= LANDING_STALE_S:
            # Tag trong base_level (FLU, bo roll/pitch - landing_target_bridge_node): tag phia truoc
            # (+x) -> bay toi (+vx); sai so = vi tri tag.
            xy = SOURCE_LANDING
            tag = self.landing_target.pose.position
            errors[SOURCE_LANDING] = (tag.x, tag.y)

        # Giu cho mac dinh (hold.py): truc ngang / yaw khong ai ra lenh thi chot va giu.
        odom_ok = self.odom_stamp_s is not None and now_s - self.odom_stamp_s <= ODOM_STALE_S
        yaw = wz = pos = None
        if odom_ok:
            q, pp = self.odom.pose.pose.orientation, self.odom.pose.pose.position
            yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
            wz = self.odom.twist.twist.angular.z
            pos = (pp.x, pp.y)
        rng = self.current_range_m()
        airborne = rng is not None and rng > MIN_AIRBORNE_RANGE_M
        cmd = self.velocity_setpoint.twist if z == SOURCE_VELOCITY else None
        # Ranh = khong ai lai truc ngang. KHONG xet rieng lenh van toc: PRECISION_LAND gui vx = vy = 0
        # (chi vz) trong luc truc ngang dang bam tag - 10-09 giu cho de len bam tag, ha lech 12-15 cm.
        free_xy = (xy == SOURCE_NONE
                   or (xy == SOURCE_VELOCITY and cmd.linear.x == 0.0 and cmd.linear.y == 0.0))
        speed = (math.hypot(self.odom.twist.twist.linear.x, self.odom.twist.twist.linear.y)
                 if odom_ok else 0.0)
        hold = self.pos_hold.update(
            pos, speed,
            self.get_parameter('position_hold').value and free_xy and airborne and self.anchored)
        if hold is not None:
            xy = SOURCE_HOLD
            ex, ey = hold[0] - pos[0], hold[1] - pos[1]
            errors[SOURCE_HOLD] = (math.cos(yaw) * ex + math.sin(yaw) * ey,
                                   -math.sin(yaw) * ex + math.cos(yaw) * ey)

        if (xy, z) != self.active_source:
            self.switch_source((xy, z), errors)

        vx = vy = vz = yaw_rate = 0.0
        if z == SOURCE_VELOCITY:
            t = self.velocity_setpoint.twist
            vx, vy, vz, yaw_rate = t.linear.x, t.linear.y, t.linear.z, t.angular.z
        elif z == SOURCE_CRUISE:
            vz = self.pids['cruise']['z'].update(errors[SOURCE_CRUISE][2], dt)
        if xy in PID_PHASE:
            vx = self.pids[PID_PHASE[xy]]['x'].update(errors[xy][0], dt)
            vy = self.pids[PID_PHASE[xy]]['y'].update(errors[xy][1], dt)
        # Bu troi: sai so cua nguon dang lai truc ngang -> he ban do -> tich; cong bias vao lenh.
        if not airborne:
            self.drift.reset()
        elif xy in PID_PHASE and yaw is not None:
            g = self.get_parameter
            self.drift.ki = g('drift_comp.ki').value
            self.drift.limit_mps = g('drift_comp.limit_mps').value
            self.drift.window_m = g('drift_comp.window_m').value
            # Chi hoc khi DICH DUNG YEN va may bay gan nhu dung yen. Dich dang chay (carrot PAD_ALIGN di
            # truoc 0,5 m) thi sai so la do DAN DUONG, khong phai troi: Gazebo 10-09 hoc nham thanh
            # +0,3 m/s ve phia truoc, sang bam tag thi bias day may bay lech 15-23 cm.
            v = self.odom.twist.twist.linear              # he than, cung he voi errors[xy]
            learn = (speed <= DRIFT_LEARN_MAX_SPEED_MPS and (xy != SOURCE_CRUISE or target_still)
                     and closing_speed(errors[xy][:2], (v.x, v.y)) <= DRIFT_LEARN_CLOSING_MPS)
            bias_w = (self.drift.update(body_to_world(errors[xy][:2], yaw), dt) if learn
                      else self.drift.bias)
            bias = world_to_body(bias_w, yaw)
            vx, vy = vx + bias[0], vy + bias[1]
        if xy in PID_PHASE and odom_ok:
            kv = self.get_parameter(f'{PID_PHASE[xy]}.vel_damping').value
            v = self.odom.twist.twist.linear              # he than base_link (child frame)
            vx, vy = vx - kv * v.x, vy - kv * v.y
        if xy == SOURCE_CRUISE:
            vx, vy = self.limit_waypoint_speed(now_s, vx, vy)
        # Yaw: FSM ra lenh (/mission/yaw) > giu huong mac dinh > yaw_rate cua lenh van toc / 0.
        if (yaw is not None and self.yaw_stamp_s is not None
                and now_s - self.yaw_stamp_s <= YAW_SETPOINT_STALE_S):
            yaw_sp = self.yaw_setpoint
            self.heading_hold.reset()               # FSM thoi ra lenh thi chot lai cho dang dung
        else:
            free_yaw = cmd is None or cmd.angular.z == 0.0
            yaw_sp = self.heading_hold.update(
                yaw, wz or 0.0, self.get_parameter('heading_hold').value and free_yaw and airborne,
                dt)
        if yaw_sp is not None:
            yaw_rate = yaw_rate_command(yaw_sp, yaw, self.get_parameter('yaw.kp').value,
                                        math.radians(self.get_parameter('yaw.max_rate_dps').value))

        vx, vy, vz, yaw_rate = self.ramp.apply(now_s, (xy, z), (vx, vy, vz, yaw_rate))
        vx, vy, vz, yaw_rate = limit_velocity(vx, vy, vz, yaw_rate, self.current_range_m())

        msg = PositionTarget()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.coordinate_frame = FRAME_BODY_NED
        msg.type_mask = TYPE_MASK_VELOCITY_YAWRATE
        msg.velocity.x, msg.velocity.y, msg.velocity.z = vx, vy, vz
        msg.yaw_rate = yaw_rate
        self.pub_setpoint.publish(msg)


def main(args=None):
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = PositionControllerNode()
    # EventsExecutor: tren Pi 4 executor mac dinh cua rclpy ton phan lon CPU de dung lai wait-set
    # moi lan thuc day (do 09-14: mission_manager_node 45-50 % -> 13,5 %).
    executor = EventsExecutor()
    executor.add_node(node)
    # Tu bat SIGINT/SIGTERM (init voi SignalHandlerOptions.NO): handler mac dinh cua rclpy tat
    # context ngay trong luc EventsExecutor con chay callback -> publish vao context da chet,
    # spin() nem loi, node thoat code 1 moi lan tat dich vu (2026-10-08). Dung executor TRUOC roi
    # moi tat context. Loi trong callback van lam node chet nhu cu (spin.result() nem lai).
    # Cho CO timeout: tin hieu roi vao luong khac thi wait() vo han khong bao gio thuc de
    # chay handler -> node treo toi khi bi SIGKILL (gap 1 lan khi tat dich vu).
    stop = threading.Event()
    for sig in (signal.SIGINT, signal.SIGTERM):
        signal.signal(sig, lambda *_: stop.set())
    spin = ThreadPoolExecutor(1).submit(executor.spin)
    spin.add_done_callback(lambda _: stop.set())
    while not stop.wait(0.5):
        pass
    executor.shutdown()
    try:
        spin.result()
    finally:
        node.destroy_node()
        rclpy.shutdown()
