"""position_controller_node - vong PID vi tri/van toc cap companion computer.

KHAC TANG voi cascade PID goc/toc-do-goc da chay san tren STM32H743: node nay chi noi
"muon o dau / nhanh co nao", FC van tu lo giu goc nghieng on dinh.

Yeu cau bat buoc - chuyen setpoint muot: khi doi nguon setpoint (waypoint hanh trinh <->
landing target) KHONG duoc reset tich phan PID ve 0 dot ngot va KHONG duoc nhay setpoint
tuc thoi; ramp trong ramp_duration_s de tranh giat may bay.

CANH BAO AN TOAN: khong bao gio tune PID lan dau tren drone that. Tune trong Gazebo truoc,
bay that thi buoc day/long an toan, tang kp tu 0 den khi dao dong nhe roi lui lai 30-50%.
"""

import rclpy
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import LandingTarget, PositionTarget
from nav_msgs.msg import Odometry
from rclpy.node import Node

from drone_control.pid import PID
from drone_control.qos import EVENT_QOS, SENSOR_QOS

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

        self.create_subscription(Odometry, '/odometry/filtered', self.on_odom, SENSOR_QOS)
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

    def on_mission_setpoint(self, msg):
        self.mission_setpoint = msg

    def on_landing_target(self, msg):
        self.landing_target = msg

    def switch_source(self, source):
        """TODO: bat dau ramp thay vi doi setpoint tuc thoi; GIU nguyen tich phan PID neu
        sai so truoc/sau chuyen doi khong lon, chi reset khi lech qua nhieu."""
        del source

    def control_step(self):
        """TODO: chon nguon setpoint dang hoat dong, tinh sai so tren tung truc tu odom,
        chay PID cua pha tuong ung, dien PositionTarget (FRAME_LOCAL_NED, type_mask van toc)
        va publish. Chua co odom hoac chua co setpoint thi khong publish."""


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
