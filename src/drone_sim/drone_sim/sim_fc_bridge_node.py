"""sim_fc_bridge_node - thay MAVROS + FC + EKF khi chay Gazebo. CHI dung trong mo phong.

Noi cac node Pi that (position_controller_node, mission_manager_node, fc_command_bridge_node...)
voi drone X3 cua Gazebo qua ros_gz_bridge, giu nguyen ten topic/service MAVROS de khong phai sua
node nao. Logic FC nam trong sim_fc.py.

  /mavros/setpoint_raw/local  -> SimFc -> /X3/gazebo/command/twist, /X3/enable
  /model/X3/odometry          -> /odometry/filtered (thay EKF: tre + nhieu tuy chon),
                                 /mavros/odometry/in, /mavros/mtf01p, /range/vertical,
                                 TF odom -> base_link
  SimFc                       -> /mavros/state, /mavros/debug_value/named_value_int,
                                 service /mavros/cmd/command
"""

import math
import random
from collections import deque

import rclpy
from geometry_msgs.msg import Twist
from mavros_msgs.msg import DebugValue, PositionTarget, State
from mavros_msgs.srv import CommandLong
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import Imu, Range
from std_msgs.msg import Bool, Float64
from tf2_ros import TransformBroadcaster

from drone_interfaces.msg import MissionState
from drone_mission.camera_tilt import TiltTable
from drone_sim import sim_fc
from drone_sim.sim_tof import quat_to_mat, tof_range

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
        # Gia lap sai so uoc luong van toc cua FC that (flow, gyro): FC tuong da bam dung lenh
        # nhung than may that su troi them chung nay (he than FLU), chi khi da roi dat. De thu giu
        # cho cua Pi (drone_control/hold.py) - FC khong keo ve khi OFFBOARD.
        self.declare_parameter('drift_vx_mps', 0.0)
        self.declare_parameter('drift_vy_mps', 0.0)
        self.declare_parameter('drift_yaw_dps', 0.0)
        # Gia lap FC THAT dap ung van toc ngang cham (vong POSHOLD + flow): lenh vx, vy tre thuan
        # vel_delay_s roi qua quan tinh bac 1 vel_tau_s. X3 cua Gazebo bam gan nhu tuc thi nen gain
        # tune trong sim (drift_comp ki 0,15) dao dong +-0,25 m/s khi bay that 10-09. 0 = tat.
        self.declare_parameter('vel_delay_s', 0.0)
        self.declare_parameter('vel_tau_s', 0.0)
        # Laser doc khi nam dat (m): X3 0,17 nhu drone cu; drone_v2 0,196 (URDF 10-29: chan 0,205 duoi
        # FC, mat MTF-01 0,009 duoi FC).
        self.declare_parameter('ground_range_m', sim_fc.GROUND_RANGE_M)
        # Servo camera (lenh 183 servo 2): xung -> goc theo CUNG bang voi camera_tilt_node, phat goc
        # (rad) cho JointPositionController cua camera_tilt_joint. Topic rong = model khong co servo.
        self.declare_parameter('camera_joint_topic', '')
        # Laser theo TIA that (sim_tof): vi tri ToF trong base_link + hop hang [x, y, yaw, ...] tren
        # dat (nua canh, cao nap chung). tof_xyz rong = laser cu (do cao than + ground_range_m).
        self.declare_parameter('tof_xyz', [0.0])
        self.declare_parameter('boxes', [0.0])
        self.declare_parameter('box_half_m', 0.06)
        self.declare_parameter('box_height_m', 0.12)
        # false: chi phat laser THO (/mavros/mtf01p) - range_vertical_node that tinh /range/vertical
        # (bu nghieng, bu hop hang) nhu tren drone.
        self.declare_parameter('publish_range_vertical', True)
        # Khau I van toc ngang nhu vong van toc FC that (kp 16 ki 3, 10-10): plugin Gazebo khong co
        # khau I nen trong tam lech (drone_v2: 5 mm + camera) thanh troi tinh ~3 cm/s ma Pi phai tu
        # bu cham. 0 = tat (X3, giu nguyen cac lan tune cu).
        self.declare_parameter('vel_ki', 0.0)
        self.declare_parameter('vel_i_limit_mps', 0.3)
        # Khau I van toc DUNG (FC that giu do cao co khau I): MulticopterVelocityControl chi co P va lay
        # luc day theo khoi luong model - mang them hop 120 g (DetachableJoint) thi lenh vz 0 van chim
        # ~0,3-0,4 m/s cho toi khi hop cham dat (mo phong 10-25).
        self.declare_parameter('vel_ki_z', 0.0)
        self.declare_parameter('vel_i_limit_z_mps', 0.5)
        # FC that khi Pi mat quyen (KHOA) ve che do ch6 = POSHOLD (GIAO_UOC_FC_ROS2 6.2): tu giu vi tri.
        # Truoc 10-25 FC gia chi lenh van toc 0 - gio 4 m/s day drone troi ra ngoai khu bay mai khong
        # dung. Gia lap POSHOLD: P vi tri tu luc vao KHOA (ground truth thay flow). 0 = tat.
        self.declare_parameter('poshold_kp', 1.0)
        self.declare_parameter('poshold_max_mps', 1.0)
        self.declare_parameter('tilt_deg', [-20.0, 0.0, 20.0])
        self.declare_parameter('pulse_us', [1244, 1514, 1762])

        g = lambda name: self.get_parameter(name).value  # noqa: E731
        self.fc = sim_fc.SimFc(auto_arm=g('auto_arm'), ground_range_m=g('ground_range_m'))
        self.tilt_table = TiltTable(g('tilt_deg'), g('pulse_us'))
        self.tof_xyz = tuple(g('tof_xyz')) if len(g('tof_xyz')) == 3 else None
        b = g('boxes')
        self.boxes = [(b[i], b[i + 1], b[i + 2], g('box_half_m'), g('box_height_m'))
                      for i in range(0, len(b) - 2, 3)]
        self.pub_cam_joint = (self.create_publisher(Float64, g('camera_joint_topic'), RELIABLE_QOS)
                              if g('camera_joint_topic') else None)
        self.odom_queue = deque()
        self.vel_queue = deque()        # (t, vx, vy) cho tre thuan
        self.vel_lag = (0.0, 0.0)
        self.v_body = (0.0, 0.0)        # van toc ngang that (khung than) tu odometry Gazebo
        self.v_up = 0.0                 # van toc dung that (khung than ~ len)
        self.vel_i = (0.0, 0.0)
        self.vel_i_z = 0.0
        self.pose = None                # (x, y, yaw) that tu odometry Gazebo
        self.hold_xy = None             # diem POSHOLD khi KHOA
        self.last_enable = None
        self.rx_ok = self.rx_rej = 0

        self.create_subscription(
            PositionTarget, '/mavros/setpoint_raw/local', self.on_setpoint, SENSOR_QOS)
        self.create_subscription(Odometry, g('odom_topic'), self.on_odometry, SENSOR_QOS)
        self.create_service(CommandLong, '/mavros/cmd/command', self.on_command)

        self.pub_twist = self.create_publisher(Twist, g('cmd_topic'), RELIABLE_QOS)
        self.pub_enable = self.create_publisher(Bool, g('enable_topic'), RELIABLE_QOS)
        self.pub_odom = self.create_publisher(Odometry, '/odometry/filtered', SENSOR_QOS)
        # TF odom -> base_link: that thi robot_localization phat. Thieu no thi
        # marker_pose_republisher_node khong lookup duoc va /marker/pose_odom im.
        self.tf_bc = TransformBroadcaster(self)
        self.pub_fc_odom = self.create_publisher(Odometry, '/mavros/odometry/in', SENSOR_QOS)
        self.pub_range = self.create_publisher(Range, '/mavros/mtf01p', SENSOR_QOS)
        # Laser gia da la do cao thang dung va sim khong co IMU: thay range_vertical_node.
        self.pub_range_v = self.create_publisher(Range, '/range/vertical', SENSOR_QOS)
        # Tu the + toc do goc nhu /mavros/imu/data cua FC that: optical_flow_node can de chieu tia
        # nhin xuong dat theo tu the luc chup (camera nghieng theo than).
        self.pub_imu = self.create_publisher(Imu, '/mavros/imu/data', SENSOR_QOS)
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
        if (response.success and self.pub_cam_joint is not None
                and request.command == sim_fc.MAV_CMD_DO_SET_SERVO and round(request.param1) == 2):
            deg = self.tilt_table.to_deg(request.param2)
            self.pub_cam_joint.publish(Float64(data=math.radians(deg)))
        self.get_logger().info(
            f'COMMAND_LONG {request.command} p1={request.param1:.0f} p2={request.param2:.0f} '
            f'-> result {response.result} (armed {armed_before} -> {self.fc.armed})')
        return response

    def on_odometry(self, msg):
        self.fc.on_odometry(msg.pose.pose.position.z)
        self.v_body = (msg.twist.twist.linear.x, msg.twist.twist.linear.y)
        self.v_up = msg.twist.twist.linear.z
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        self.pose = (p.x, p.y, math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                                          1.0 - 2.0 * (q.y * q.y + q.z * q.z)))
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
        if self.tof_xyz is not None:
            p, q = msg.pose.pose.position, msg.pose.pose.orientation
            r = tof_range((p.x, p.y, p.z), quat_to_mat(q.x, q.y, q.z, q.w), self.tof_xyz, self.boxes)
            if r is not None and not sim_fc.RANGE_MIN_M <= r <= sim_fc.RANGE_MAX_M:
                r = None
        rng.range = 0.0 if r is None else r      # hop dong 1.6: FC gui 0 khi mat laser
        self.pub_range.publish(rng)
        if self.get_parameter('publish_range_vertical').value:
            self.pub_range_v.publish(rng)

        imu = Imu()
        imu.header.stamp = stamp
        imu.header.frame_id = 'base_link'
        imu.orientation = msg.pose.pose.orientation
        imu.angular_velocity = msg.twist.twist.angular        # Gazebo: he than (child frame)
        self.pub_imu.publish(imu)

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
            phat = self.odom_queue.popleft()[1]
            self.pub_odom.publish(phat)
            self.publish_tf(phat)

    def publish_tf(self, odom):
        """TF odom -> base_link tu chinh ban tin vua phat, de TF va topic khong lech nhau."""
        tf = TransformStamped()
        tf.header = odom.header
        tf.child_frame_id = odom.child_frame_id
        p = odom.pose.pose.position
        tf.transform.translation.x, tf.transform.translation.y, tf.transform.translation.z = p.x, p.y, p.z
        tf.transform.rotation = odom.pose.pose.orientation
        self.tf_bc.sendTransform(tf)

    def control_step(self):
        enable, (vx, vy, vz, yaw_rate) = self.fc.step(self.now_s())
        if enable != self.last_enable:
            self.pub_enable.publish(Bool(data=enable))
            self.get_logger().info(f'dong co {"BAT" if enable else "TAT"}')
            self.last_enable = enable
        vx, vy = self.poshold(enable, vx, vy)
        vx, vy = self.lag_horizontal(enable, vx, vy)
        h = self.fc.height_m()
        ki = self.get_parameter('vel_ki').value
        if ki > 0.0 and enable and h is not None and h > 0.05:
            dt = 1.0 / self.get_parameter('control_rate_hz').value
            lim = self.get_parameter('vel_i_limit_mps').value
            self.vel_i = tuple(max(-lim, min(lim, i + ki * (c - v) * dt))
                               for i, c, v in zip(self.vel_i, (vx, vy), self.v_body))
            vx, vy = vx + self.vel_i[0], vy + self.vel_i[1]
        elif not enable:
            self.vel_i = (0.0, 0.0)
        kz = self.get_parameter('vel_ki_z').value
        if kz > 0.0 and enable and h is not None and h > 0.05:
            dt = 1.0 / self.get_parameter('control_rate_hz').value
            lim = self.get_parameter('vel_i_limit_z_mps').value
            self.vel_i_z = max(-lim, min(lim, self.vel_i_z + kz * (vz - self.v_up) * dt))
            vz += self.vel_i_z
        elif not enable:
            self.vel_i_z = 0.0
        # Gio/troi tac dung toi sat dat (cang ~5 cm). Truoc 10-11 la > 0,3 m: gio tat dot ngot luc ha
        # mu, bu troi Pi da hoc (-0,16 m/s) day lui 19 cm - gio that khong tat o 0,3 m.
        if enable and h is not None and h > 0.1:
            g = self.get_parameter
            vx += g('drift_vx_mps').value
            vy += g('drift_vy_mps').value
            yaw_rate += math.radians(g('drift_yaw_dps').value)
        twist = Twist()
        twist.linear.x, twist.linear.y, twist.linear.z = vx, vy, vz
        twist.angular.z = yaw_rate
        self.pub_twist.publish(twist)

    def poshold(self, enable, vx, vy):
        """Dang arm ma OFFBOARD khong chay (KHOA / TAT): giu vi tri nhu POSHOLD cua FC that."""
        kp = self.get_parameter('poshold_kp').value
        if not enable or self.fc.running or kp <= 0.0 or self.pose is None:
            self.hold_xy = None
            return vx, vy
        x, y, yaw = self.pose
        if self.hold_xy is None:
            self.hold_xy = (x, y)
            self.get_logger().warning(f'OFFBOARD khong chay - POSHOLD tai ({x:.2f}, {y:.2f})')
        ex, ey = self.hold_xy[0] - x, self.hold_xy[1] - y
        bx = kp * (math.cos(yaw) * ex + math.sin(yaw) * ey)
        by = kp * (-math.sin(yaw) * ex + math.cos(yaw) * ey)
        lim = self.get_parameter('poshold_max_mps').value
        k = min(1.0, lim / max(math.hypot(bx, by), 1e-9))
        return bx * k, by * k

    def lag_horizontal(self, enable, vx, vy):
        """Tre thuan + quan tinh bac 1 cho lenh van toc ngang (vel_delay_s, vel_tau_s)."""
        g = self.get_parameter
        if not enable:
            self.vel_queue.clear()
            self.vel_lag = (0.0, 0.0)
            return vx, vy
        now = self.now_s()
        self.vel_queue.append((now, vx, vy))
        delay = g('vel_delay_s').value
        while len(self.vel_queue) > 1 and now - self.vel_queue[1][0] >= delay:
            self.vel_queue.popleft()
        _, dvx, dvy = self.vel_queue[0]
        tau = g('vel_tau_s').value
        dt = 1.0 / g('control_rate_hz').value
        a = 1.0 if tau <= 0.0 else dt / (tau + dt)
        lx, ly = self.vel_lag
        self.vel_lag = (lx + a * (dvx - lx), ly + a * (dvy - ly))
        return self.vel_lag

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
