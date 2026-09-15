"""sim_fc_bridge_node - thay MAVROS + FC + EKF khi chay Gazebo. CHI dung trong mo phong.

Noi cac node Pi that (position_controller_node, mission_manager_node, fc_command_bridge_node...)
voi drone X3 cua Gazebo qua ros_gz_bridge, giu nguyen ten topic/service MAVROS de khong phai sua
node nao. Logic FC nam trong sim_fc.py.

  /mavros/setpoint_raw/local  -> SimFc -> /X3/gazebo/command/twist, /X3/enable
  /model/X3/odometry          -> /odometry/filtered (thay EKF: tre + nhieu tuy chon),
                                 /mavros/odometry/in, /mavros/mtf01p
  SimFc                       -> /mavros/state, /mavros/debug_value/named_value_int,
                                 service /mavros/cmd/command
"""

import random
from collections import deque

import rclpy
from geometry_msgs.msg import Twist
from mavros_msgs.msg import DebugValue, PositionTarget, State
from mavros_msgs.srv import CommandLong
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import Range
from std_msgs.msg import Bool

from drone_interfaces.msg import MissionState
from drone_sim import sim_fc

SENSOR_QOS = QoSProfile(depth=1, reliability=QoSReliabilityPolicy.BEST_EFFORT)
RELIABLE_QOS = QoSProfile(depth=10, reliability=QoSReliabilityPolicy.RELIABLE)
NAMED_VALUE_QOS = QoSProfile(depth=20, reliability=QoSReliabilityPolicy.BEST_EFFORT)


class SimFcBridgeNode(Node):

    def __init__(self):
        super().__init__('sim_fc_bridge_node')

        self.declare_parameter('cmd_topic', '/X3/gazebo/command/twist')
        self.declare_parameter('enable_topic', '/X3/enable')
        self.declare_parameter('odom_topic', '/model/X3/odometry')
        self.declare_parameter('control_rate_hz', 50.0)
        # Tune khong can mission_manager_node: arm ngay va tu phat /mission/state cho watchdog P6.
        self.declare_parameter('auto_arm', False)
        self.declare_parameter('publish_mission_heartbeat', False)
        # Gia lap EKF xau hon ground truth: tre (s) va nhieu vi tri Gauss (sigma, m).
        self.declare_parameter('odom_delay_s', 0.0)
        self.declare_parameter('odom_noise_m', 0.0)

        g = lambda name: self.get_parameter(name).value  # noqa: E731
        self.fc = sim_fc.SimFc(auto_arm=g('auto_arm'))
        self.odom_queue = deque()
        self.last_enable = None
        self.rx_ok = self.rx_rej = 0

        self.create_subscription(
            PositionTarget, '/mavros/setpoint_raw/local', self.on_setpoint, SENSOR_QOS)
        self.create_subscription(Odometry, g('odom_topic'), self.on_odometry, SENSOR_QOS)
        self.create_service(CommandLong, '/mavros/cmd/command', self.on_command)

        self.pub_twist = self.create_publisher(Twist, g('cmd_topic'), RELIABLE_QOS)
        self.pub_enable = self.create_publisher(Bool, g('enable_topic'), RELIABLE_QOS)
        self.pub_odom = self.create_publisher(Odometry, '/odometry/filtered', SENSOR_QOS)
        self.pub_fc_odom = self.create_publisher(Odometry, '/mavros/odometry/in', SENSOR_QOS)
        self.pub_range = self.create_publisher(Range, '/mavros/mtf01p', SENSOR_QOS)
        self.pub_state = self.create_publisher(State, '/mavros/state', RELIABLE_QOS)
        self.pub_named = self.create_publisher(
            DebugValue, '/mavros/debug_value/named_value_int', NAMED_VALUE_QOS)
        if g('publish_mission_heartbeat'):
            self.pub_mission = self.create_publisher(MissionState, '/mission/state', RELIABLE_QOS)
            self.create_timer(0.2, lambda: self.pub_mission.publish(MissionState(detail='sim')))

        self.create_timer(1.0 / g('control_rate_hz'), self.control_step)
        self.create_timer(0.2, self.publish_state)          # /mavros/state 5 Hz
        self.create_timer(0.5, self.publish_named_values)   # FC phat NAMED_VALUE_INT 2 Hz

    def now_s(self):
        return self.get_clock().now().nanoseconds / 1e9

    def on_setpoint(self, msg):
        ok = self.fc.on_setpoint(self.now_s(), msg.coordinate_frame, msg.type_mask,
                                 msg.velocity.x, msg.velocity.y, msg.velocity.z, msg.yaw_rate)
        if ok:
            self.rx_ok += 1
        else:
            self.rx_rej += 1

    def on_command(self, request, response):
        armed_before = self.fc.armed
        response.result = self.fc.command(request.command, request.param1, request.param2)
        response.success = response.result == sim_fc.MAV_RESULT_ACCEPTED
        self.get_logger().info(
            f'COMMAND_LONG {request.command} p1={request.param1:.0f} p2={request.param2:.0f} '
            f'-> result {response.result} (armed {armed_before} -> {self.fc.armed})')
        return response

    def on_odometry(self, msg):
        self.fc.on_odometry(msg.pose.pose.position.z)
        now = self.now_s()
        stamp = self.get_clock().now().to_msg()

        fc_odom = Odometry()
        fc_odom.header.stamp = stamp
        fc_odom.header.frame_id, fc_odom.child_frame_id = 'odom', 'base_link'
        fc_odom.pose = msg.pose
        fc_odom.twist = msg.twist
        # covariance van toc hop le (< 1e5): tieu chi cham dat dung vz nay (11.1 #12e).
        fc_odom.twist.covariance = [0.0] * 36
        for i in range(6):
            fc_odom.twist.covariance[i * 7] = 0.01
        self.pub_fc_odom.publish(fc_odom)

        rng = Range()
        rng.header.stamp = stamp
        rng.header.frame_id = 'mtf01p'
        rng.radiation_type = Range.INFRARED
        rng.min_range, rng.max_range = sim_fc.RANGE_MIN_M, sim_fc.RANGE_MAX_M
        r = self.fc.range_m()
        rng.range = float('inf') if r is None else r
        self.pub_range.publish(rng)

        # "EKF": ground truth + nhieu, phat tre odom_delay_s.
        out = Odometry()
        out.header.stamp = stamp
        out.header.frame_id, out.child_frame_id = 'odom', 'base_link'
        out.pose.pose.orientation = msg.pose.pose.orientation
        noise = self.get_parameter('odom_noise_m').value
        p = msg.pose.pose.position
        out.pose.pose.position.x = p.x + random.gauss(0.0, noise) if noise > 0 else p.x
        out.pose.pose.position.y = p.y + random.gauss(0.0, noise) if noise > 0 else p.y
        out.pose.pose.position.z = p.z + random.gauss(0.0, noise) if noise > 0 else p.z
        var = max(noise * noise, 1e-4)
        cov = [0.0] * 36
        for i in range(6):
            cov[i * 7] = var if i < 3 else 1e-4
        out.pose.covariance = cov
        out.twist = msg.twist
        self.odom_queue.append((now + self.get_parameter('odom_delay_s').value, out))
        while self.odom_queue and self.odom_queue[0][0] <= now:
            self.pub_odom.publish(self.odom_queue.popleft()[1])

    def control_step(self):
        enable, (vx, vy, vz, yaw_rate) = self.fc.step(self.now_s())
        if enable != self.last_enable:
            self.pub_enable.publish(Bool(data=enable))
            self.get_logger().info(f'dong co {"BAT" if enable else "TAT"}')
            self.last_enable = enable
        twist = Twist()
        twist.linear.x, twist.linear.y, twist.linear.z = vx, vy, vz
        twist.angular.z = yaw_rate
        self.pub_twist.publish(twist)

    def publish_state(self):
        msg = State()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.connected = True
        msg.armed = self.fc.armed
        msg.mode = 'OFFBOARD' if self.fc.running else 'POSHOLD'
        self.pub_state.publish(msg)
        # Bat lai dong co dinh ky: Gazebo co the chua subscribe khi ban tin dau duoc gui.
        if self.last_enable is not None:
            self.pub_enable.publish(Bool(data=self.last_enable))

    def publish_named_values(self):
        values = self.fc.named_values()
        values['OB_RX_OK'], values['OB_RX_REJ'] = self.rx_ok, self.rx_rej
        stamp = self.get_clock().now().to_msg()
        for name, value in values.items():
            msg = DebugValue()
            msg.header.stamp = stamp
            msg.index, msg.array_id = -1, -1
            msg.name, msg.value_int = name, int(value)
            msg.type = DebugValue.TYPE_NAMED_VALUE_INT
            self.pub_named.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = SimFcBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
