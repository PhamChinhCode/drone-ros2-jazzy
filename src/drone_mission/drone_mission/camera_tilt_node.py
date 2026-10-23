"""camera_tilt_node - nghieng camera bang servo tren FC va phat TF lap dat camera THEO goc do.

Nguon DUY NHAT cua TF base_link -> camera_link (thay static TF 20 do cu trong estimation.launch.py)
va cua /camera/tilt - goc camera HIEN TAI da on dinh (do, + ve mui, 0 = thang xuong); NaN khi servo
dang quay / chua chac servo o dau. Moi ben dung anh de do (tag, optical flow) PHAI bo du lieu khi
/camera/tilt la NaN: anh nhoe va TF chua dung goc that.

Lenh: /mission/camera_tilt (Float32, do) tu mission_manager_node, phat lai deu - node chi hanh dong
khi goc doi. Doi goc -> bang tra us (camera_tilt.py) -> fc_command_bridge_node ~/servo (CAMERA, lenh
183 servo 2), tien toi tu phia us thap de an ro, cho settle_s roi moi bao goc moi. FC tu choi (Pi
khong co quyen) -> thu lai moi retry_s, /camera/tilt giu NaN.

Servo khong co phan hoi vi tri: luc khoi dong coi nhu camera o initial_tilt_deg (Pi chua co quyen thi
RC giu servo - nguoi lai dat nhin thang xuong truoc khi bay).
"""

from concurrent.futures import ThreadPoolExecutor
import math
import signal
import threading

import rclpy
from geometry_msgs.msg import TransformStamped
from rclpy.experimental import EventsExecutor
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from std_msgs.msg import Float32
from tf2_ros import TransformBroadcaster

from drone_interfaces.srv import SetServo
from drone_mission.camera_tilt import approach_moves, camera_link_pitch, camera_position, TiltTable
from drone_mission.qos import EVENT_QOS

# Goc lenh moi khac goc dang theo duoi qua muc nay moi tinh la doi (lenh phat lai 5 Hz).
CHANGE_DEG = 0.2


class CameraTiltNode(Node):

    def __init__(self):
        super().__init__('camera_tilt_node')
        # Bang hieu chinh (mission.yaml), do 2026-10-22.
        self.declare_parameter('tilt_deg', [-20.0, 0.0, 20.0])
        self.declare_parameter('pulse_us', [1244, 1514, 1762])
        self.declare_parameter('overshoot_us', 100)
        self.declare_parameter('overshoot_wait_s', 0.3)
        self.declare_parameter('settle_s', 0.8)
        self.declare_parameter('retry_s', 1.0)
        self.declare_parameter('initial_tilt_deg', 0.0)
        # Truc servo camera tren than (m, base_link). z = 0: ngang cam bien ToF (do 10-08).
        self.declare_parameter('camera_xyz', [0.09, 0.0, 0.0])
        # Ong kinh so voi truc servo (m, khung camera_tilt_link o goc 0 - URDF drone_v2). 0 = ong kinh
        # nam ngay tren truc, chi xoay tai cho.
        self.declare_parameter('lens_xyz', [0.0, 0.0, 0.0])
        self.declare_parameter('tf_rate_hz', 30.0)

        g = self.get_parameter
        self.table = TiltTable(g('tilt_deg').value, g('pulse_us').value)
        self.pivot = list(g('camera_xyz').value)
        self.lens = list(g('lens_xyz').value)
        self.current_deg = float(g('initial_tilt_deg').value)   # goc TF dang phat
        self.current_us = None          # xung cuoi FC da nhan; None = chua tung gui
        self.settled = True
        self.wanted_deg = self.current_deg
        self.moves = []                 # xung con phai gui cho lan doi goc dang chay
        self.next_s = None              # luc duoc gui xung ke / bao on dinh
        self.busy = False               # dang cho ACK cua ~/servo

        self.cli = self.create_client(SetServo, '/fc_command_bridge_node/servo')
        self.tf_bc = TransformBroadcaster(self)
        self.pub_tilt = self.create_publisher(Float32, '/camera/tilt', EVENT_QOS)
        self.create_subscription(Float32, '/mission/camera_tilt', self.on_command, EVENT_QOS)
        self.create_timer(1.0 / g('tf_rate_hz').value, self.tick)

    def now_s(self):
        return self.get_clock().now().nanoseconds / 1e9

    def on_command(self, msg):
        deg = float(msg.data)
        if not math.isfinite(deg):
            return
        same = abs(deg - self.wanted_deg) < CHANGE_DEG
        # Cung goc nhung servo chua tung duoc lenh (moi khoi dong): van gui mot lan cho servo ve dung
        # goc da gia dinh, nhung chua bao NaN - FC con tu choi (Pi chua co quyen) thi optical flow
        # van chay voi goc gia dinh nhu truoc khi co servo.
        if same and (self.current_us is not None or self.moves):
            return
        self.wanted_deg = deg
        us = self.table.to_us(deg)
        self.moves = approach_moves(self.current_us, us, self.get_parameter('overshoot_us').value)
        self.settled = same
        self.next_s = self.now_s()
        self.get_logger().info(f'camera -> {deg:+.1f} do ({us} us, gui {self.moves})')

    def tick(self):
        now = self.now_s()
        if (self.moves or not self.settled) and not self.busy and self.next_s is not None \
                and now >= self.next_s:
            if self.moves:
                self.send(self.moves[0])
            else:
                self.current_deg = self.wanted_deg
                self.settled = True
                self.get_logger().info(f'camera on dinh o {self.current_deg:+.1f} do')
        self.publish(now)

    def send(self, us):
        if not self.cli.service_is_ready():
            self.next_s = self.now_s() + self.get_parameter('retry_s').value
            return
        self.busy = True
        future = self.cli.call_async(SetServo.Request(target=SetServo.Request.CAMERA, value=us))
        future.add_done_callback(lambda f: self.on_reply(f, us))

    def on_reply(self, future, us):
        self.busy = False
        resp = future.result()
        g = self.get_parameter
        if resp is None or not resp.success:
            why = resp.message if resp is not None else 'khong co tra loi'
            self.get_logger().warning(f'servo camera {us} us bi tu choi: {why} - thu lai',
                                      throttle_duration_sec=5.0)
            self.next_s = self.now_s() + g('retry_s').value
            return
        self.current_us = us
        self.settled = False            # servo dang chay that (ke ca lan dong bo dau)
        self.moves.pop(0)
        self.next_s = self.now_s() + (g('overshoot_wait_s').value if self.moves
                                      else g('settle_s').value)

    def publish(self, now):
        tf = TransformStamped()
        tf.header.stamp = self.get_clock().now().to_msg()
        tf.header.frame_id, tf.child_frame_id = 'base_link', 'camera_link'
        tf.transform.translation.x, tf.transform.translation.y, tf.transform.translation.z = \
            camera_position(self.pivot, self.lens, self.current_deg)
        half = camera_link_pitch(self.current_deg) / 2.0
        tf.transform.rotation.y, tf.transform.rotation.w = math.sin(half), math.cos(half)
        self.tf_bc.sendTransform(tf)
        self.pub_tilt.publish(Float32(data=self.current_deg if self.settled else math.nan))


def main(args=None):
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = CameraTiltNode()
    # Cung khung dung/tat nhu mission_manager_node (EventsExecutor + tu bat SIGINT/SIGTERM).
    executor = EventsExecutor()
    executor.add_node(node)
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
