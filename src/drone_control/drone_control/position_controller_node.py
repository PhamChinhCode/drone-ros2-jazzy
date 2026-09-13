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
"""

import math

import rclpy
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import LandingTarget, PositionTarget
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import Range

from drone_control.pid import PID
from drone_control.qos import EVENT_QOS, SENSOR_QOS
from drone_control.setpoint_limits import limit_velocity

FRAME_BODY_NED = 8
TYPE_MASK_VELOCITY_YAWRATE = 0x07C7
RANGE_STALE_S = 0.5             # laser 20 Hz; qua han coi nhu mat laser -> xuong cham

SOURCE_MISSION = 'mission'
SOURCE_LANDING = 'landing'


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

        self.pids = {phase: {axis: self._make_pid(phase, axis) for axis in ('x', 'y', 'z')}
                     for phase in ('cruise', 'landing')}

        self.odom = None
        self.mission_setpoint = None
        self.landing_target = None
        self.active_source = SOURCE_MISSION
        self.ramp_started_at = None
        self.range_m = None
        self.range_stamp_s = None

        self.create_subscription(Odometry, '/odometry/filtered', self.on_odom, SENSOR_QOS)
        self.create_subscription(Range, '/mavros/mtf01p', self.on_range, SENSOR_QOS)
        self.create_subscription(PoseStamped, '/mission/setpoint', self.on_mission_setpoint, EVENT_QOS)
        self.create_subscription(
            LandingTarget, '/mavros/landing_target/raw', self.on_landing_target, SENSOR_QOS)

        self.pub_setpoint = self.create_publisher(
            PositionTarget, '/mavros/setpoint_raw/local', SENSOR_QOS)

        self.create_timer(1.0 / self.get_parameter('control_rate_hz').value, self.control_step)

    def _make_pid(self, phase, axis):
        g = lambda k: self.get_parameter(f'{phase}.{axis}.{k}').value  # noqa: E731
        return PID(g('kp'), g('ki'), g('kd'), g('i_limit'), g('out_limit'))

    def on_odom(self, msg):
        self.odom = msg

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

    def on_landing_target(self, msg):
        self.landing_target = msg

    def switch_source(self, source):
        """TODO: bat dau ramp thay vi doi setpoint tuc thoi; GIU nguyen tich phan PID neu
        sai so truoc/sau chuyen doi khong lon, chi reset khi lech qua nhieu."""
        del source

    def control_step(self):
        """Tinh van toc FLU tu sai so vi tri, kep, publish. Thieu odom/setpoint -> van toc 0.

        TODO: nguon SOURCE_LANDING (P1 - Pi dong vong van toc theo marker) va ramp khi doi nguon.
        """
        vx = vy = vz = 0.0
        if self.odom is not None and self.mission_setpoint is not None:
            dt = 1.0 / self.get_parameter('control_rate_hz').value
            p = self.odom.pose.pose
            target = self.mission_setpoint.pose.position
            # Sai so trong he odom (ENU) xoay ve he than FLU bang yaw.
            ex, ey = target.x - p.position.x, target.y - p.position.y
            q = p.orientation
            yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
            ex_body = math.cos(yaw) * ex + math.sin(yaw) * ey
            ey_body = -math.sin(yaw) * ex + math.cos(yaw) * ey
            pids = self.pids['cruise']
            vx = pids['x'].update(ex_body, dt)
            vy = pids['y'].update(ey_body, dt)
            vz = pids['z'].update(target.z - p.position.z, dt)

        vx, vy, vz, yaw_rate = limit_velocity(vx, vy, vz, 0.0, self.current_range_m())

        msg = PositionTarget()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.coordinate_frame = FRAME_BODY_NED
        msg.type_mask = TYPE_MASK_VELOCITY_YAWRATE
        msg.velocity.x, msg.velocity.y, msg.velocity.z = vx, vy, vz
        msg.yaw_rate = yaw_rate
        self.pub_setpoint.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = PositionControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
