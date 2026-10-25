"""sim_gripper_node - co cau kep MG90 cua drone_v2 trong Gazebo. CHI dung trong mo phong.

  /gripper/command (CLOSE/OPEN, gripper_controller_node van la nguon trang thai) ->
    servo chay toi -120 do (mo) / 0 (dong) theo toc do MG90, 5 joint kep di theo loi giai 4 khau
    CHINH XAC (gripper_sim) -> JointPositionController /X3/gripper/<joint>/cmd.
  Dong xong ma hop nam giua hai ham (gripper_sim.box_in_jaws, vi tri that tu odometry Gazebo):
    xoa hop tinh hop_<n>, sinh hop dong hang_<n> 120 g tai cho (canh giua ham, xoay theo ham nhu
    ham ep that) -> plugin DetachableJoint trong worlds/drone_v2.sdf tu gan vao drone (plugin luon
    cho gan khi model con xuat hien), drone mang tai that.
  Mo: tach moi hop dang kep (/X3/gripper/detach_<n>) -> hop roi xuong.
Hop dang kep khong kep lai duoc (hop dong khong con trong danh sach hop tinh).
"""

import math
import os
import subprocess
import tempfile
import threading

from ament_index_python.packages import get_package_share_directory
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from std_msgs.msg import Empty, Float64

from drone_control.qos import EVENT_QOS, SENSOR_QOS
from drone_interfaces.msg import GripperCommand
from drone_sim.gripper_sim import CLOSED_RAD, OPEN_RAD, box_in_jaws, joint_targets, step_servo
from drone_sim.sim_tof import quat_to_mat

RELIABLE_QOS = QoSProfile(depth=10, reliability=QoSReliabilityPolicy.RELIABLE)
BOX_HALF, BOX_HEIGHT, BOX_MASS = 0.06, 0.12, 0.12

HOP_SDF = """<sdf version="1.9"><model name="{name}"><link name="link">
  <inertial><pose>0 0 0.06 0 0 0</pose><mass>{m}</mass>
    <inertia><ixx>{i}</ixx><iyy>{i}</iyy><izz>{i}</izz></inertia></inertial>
  <collision name="hop"><pose>0 0 0.06 0 0 0</pose>
    <geometry><box><size>0.12 0.12 0.12</size></box></geometry>
    <surface><contact><collide_bitmask>0x02</collide_bitmask></contact></surface></collision>
  <visual name="hop"><pose>0 0 0.06 0 0 0</pose>
    <geometry><box><size>0.12 0.12 0.12</size></box></geometry>
    <material><ambient>0.6 0.4 0.2 1</ambient><diffuse>0.6 0.4 0.2 1</diffuse></material></visual>
  <visual name="tag_hop"><pose>0 0 0.1205 0 0 -1.5708</pose>
    <geometry><plane><normal>0 0 1</normal><size>0.125 0.125</size></plane></geometry>
    <material><ambient>1 1 1 1</ambient><diffuse>1 1 1 1</diffuse><specular>0 0 0 1</specular>
      <pbr><metal><albedo_map>{tex}</albedo_map>
        <roughness>1.0</roughness><metalness>0.0</metalness></metal></pbr></material></visual>
</link></model></sdf>"""


class SimGripperNode(Node):

    def __init__(self):
        super().__init__('sim_gripper_node')
        self.declare_parameter('world', 'drone_tune')
        self.declare_parameter('rate_hz', 50.0)
        # [x, y, yaw] tam day hop (world) - PHAI khop hop_<ten> trong worlds/drone_v2.sdf.
        self.declare_parameter('boxes', [10.0, -0.30, 1.5708])
        self.declare_parameter('box_names', ['a'])
        self.declare_parameter('box_tag_ids', [11])
        b = self.get_parameter('boxes').value
        names = self.get_parameter('box_names').value
        tags = self.get_parameter('box_tag_ids').value
        self.boxes = {n: (b[3 * i], b[3 * i + 1], b[3 * i + 2], t)
                      for i, (n, t) in enumerate(zip(names, tags))}
        self.held = []
        self.servo = OPEN_RAD           # gripper_controller_node khoi dong o trang thai MO
        self.target = OPEN_RAD
        self.checked = True             # da xet kep cho lan dong nay chua
        self.pose = None
        self.pub_joint = {j: self.create_publisher(Float64, f'/X3/gripper/{j}/cmd', RELIABLE_QOS)
                          for j in joint_targets(0.0)}
        self.pub_detach = {n: self.create_publisher(Empty, f'/X3/gripper/detach_{n}', RELIABLE_QOS)
                           for n in self.boxes}
        self.create_subscription(GripperCommand, '/gripper/command', self.on_command, EVENT_QOS)
        self.create_subscription(Odometry, '/model/X3/odometry', self.on_odom, SENSOR_QOS)
        self.dt = 1.0 / self.get_parameter('rate_hz').value
        self.create_timer(self.dt, self.step)

    def on_odom(self, msg):
        self.pose = msg.pose.pose

    def on_command(self, msg):
        target = CLOSED_RAD if msg.command == GripperCommand.GRIPPER_CLOSE else OPEN_RAD
        if target == self.target:
            return
        self.target = target
        if target == OPEN_RAD:
            for n in self.held:
                self.pub_detach[n].publish(Empty())
                p = self.pose.position if self.pose is not None else None
                o = f' tai drone ({p.x:.3f}, {p.y:.3f}, {p.z:.3f})' if p is not None else ''
                self.get_logger().info(f'tha hop {n}{o}')
            self.held = []
        else:
            self.checked = False

    def step(self):
        self.servo = step_servo(self.servo, self.target, self.dt)
        for j, a in joint_targets(self.servo).items():
            self.pub_joint[j].publish(Float64(data=a))
        if not self.checked and self.servo == CLOSED_RAD and self.pose is not None:
            self.checked = True
            self.try_grip()

    def try_grip(self):
        p, q = self.pose.position, self.pose.orientation
        r = quat_to_mat(q.x, q.y, q.z, q.w)
        yaw = math.atan2(r[1][0], r[0][0])
        for n, (bx, by, byaw, tag) in self.boxes.items():
            d = (bx - p.x, by - p.y, -p.z)
            body = tuple(sum(r[k][i] * d[k] for k in range(3)) for i in range(3))
            if box_in_jaws(body, byaw - yaw, BOX_HALF, BOX_HEIGHT):
                # Ham ep hop vao giua: y khung than = 0, mat hop song song ham.
                c, s = math.cos(yaw), math.sin(yaw)
                x, y = p.x + c * body[0], p.y + s * body[0]
                del self.boxes[n]
                self.held.append(n)
                self.get_logger().info(f'kep hop {n}: lech ngang {body[1] * 100:.1f} cm, '
                                       f'doc {body[0] * 100:.1f} cm')
                threading.Thread(target=self.swap_box, args=(n, tag, x, y, yaw),
                                 daemon=True).start()
                return
        ly_do = ''
        for n, (bx, by, byaw, _) in self.boxes.items():
            d = (bx - p.x, by - p.y, -p.z)
            body = tuple(sum(r[k][i] * d[k] for k in range(3)) for i in range(3))
            ly_do += (f' | hop {n}: than ({body[0] * 100:.1f}, {body[1] * 100:.1f}, '
                      f'{body[2] * 100:.1f}) cm, lech goc {math.degrees(byaw - yaw):.0f} do')
        self.get_logger().warn(f'dong kep nhung khong co hop giua hai ham{ly_do}')

    def gz(self, service, reqtype, req):
        world = self.get_parameter('world').value
        subprocess.run(['gz', 'service', '-s', f'/world/{world}/{service}',
                        '--reqtype', reqtype, '--reptype', 'gz.msgs.Boolean', '--timeout', '3000',
                        '--req', req], check=False, capture_output=True)

    def swap_box(self, n, tag, x, y, yaw):
        tex = os.path.join(get_package_share_directory('drone_sim'), 'worlds', 'textures',
                           f'tag36h11_{tag:02d}.png')
        i = BOX_MASS * (2 * (2 * BOX_HALF) ** 2) / 12
        with tempfile.NamedTemporaryFile('w', suffix='.sdf', delete=False) as f:
            f.write(HOP_SDF.format(name=f'hang_{n}', m=BOX_MASS, i=f'{i:.3e}', tex=tex))
        self.gz('remove', 'gz.msgs.Entity', f'name: "hop_{n}" type: MODEL')
        self.gz('create', 'gz.msgs.EntityFactory',
                f'sdf_filename: "{f.name}" name: "hang_{n}"'
                f' pose: {{position: {{x: {x} y: {y} z: 0}}'
                f' orientation: {{z: {math.sin(yaw / 2)} w: {math.cos(yaw / 2)}}}}}')


def main(args=None):
    rclpy.init(args=args)
    node = SimGripperNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
