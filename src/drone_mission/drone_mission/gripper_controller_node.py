"""gripper_controller_node - dieu khien co cau gap/tha va la NGUON XAC NHAN DUY NHAT rang hang da
gap/tha. mission_manager_node khong duoc phep suy doan qua timeout.

Servo kep cam vao FC (PD14), khong cam vao Pi: node gui lenh qua fc_command_bridge_node ~/servo
(MAV_CMD_DO_GRIPPER). Khong co cong tac hanh trinh / cam bien luc (chu du an chot 2026-10-21):
FC ACCEPTED roi cho travel_s (servo chay het hanh trinh) thi coi nhu da kep/nha -
sensor_confirmed o day la "da ra lenh va da cho du", KHONG phai do that.

FC tra DENIED khi Pi khong co quyen (giong quyen bay) -> state ERROR, lenh lap lai ke tiep cua
mission_manager se gui lai.

Chay duoc khi khong co phan cung: simulate=true (mac dinh) bo qua FC, chi gia lap hanh trinh, du
de chay het chuoi ACTUATE_GRIPPER trong Gazebo. Muon thu duong THAT BAI (gripper khong xac nhan
-> failsafe FS_GRIP_CONFIRM_FAIL) thi khong chay node nay.
"""

from concurrent.futures import ThreadPoolExecutor
import signal
import threading

import rclpy
from rclpy.experimental import EventsExecutor
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions

from drone_interfaces.msg import GripperCommand, GripperStatus
from drone_interfaces.srv import SetServo
from drone_mission.qos import EVENT_QOS

SIM_FORCE_N = 2.5      # luc gia lap khi da gap chac; khong co y nghia do luong


class GripperControllerNode(Node):

    def __init__(self):
        super().__init__('gripper_controller_node')

        self.declare_parameter('simulate', True)
        self.declare_parameter('status_rate_hz', 10.0)
        # Cho servo chay het hanh trinh sau khi FC nhan lenh (ES08MD II ~0,5 s).
        self.declare_parameter('travel_s', 0.5)

        self.cli_servo = self.create_client(SetServo, '/fc_command_bridge_node/servo')

        # Khoi dong o trang thai MO: an toan hon: khong giu hang khi chua ai ra lenh.
        self.state = GripperStatus.GRIP_STATE_OPEN
        self.target_state = GripperStatus.GRIP_STATE_OPEN
        self.move_done_s = None

        self.create_subscription(GripperCommand, '/gripper/command', self.on_command, EVENT_QOS)
        self.pub_status = self.create_publisher(GripperStatus, '/gripper/status', EVENT_QOS)
        self.create_timer(1.0 / self.get_parameter('status_rate_hz').value, self.publish_status)

    def now_s(self):
        return self.get_clock().now().nanoseconds / 1e9

    def set_servo(self, muon):
        """Gui lenh kep/nha xuong FC. Khong chan: ket qua ve on_servo_reply."""
        if not self.cli_servo.service_is_ready():
            self.on_failed(muon, 'fc_command_bridge_node chua san sang')
            return
        req = SetServo.Request()
        req.target = SetServo.Request.GRIPPER
        req.value = 1 if muon == GripperStatus.GRIP_STATE_CLOSED else 0
        future = self.cli_servo.call_async(req)
        future.add_done_callback(lambda f: self.on_servo_reply(f, muon))

    def on_servo_reply(self, future, muon):
        if muon != self.target_state:
            return      # da co lenh moi hon
        resp = future.result()
        if not resp.success:
            self.on_failed(muon, resp.message)
            return
        # Chi bat dau dem hanh trinh khi FC da doi xung.
        self.move_done_s = self.now_s() + self.get_parameter('travel_s').value

    def on_failed(self, muon, ly_do):
        self.get_logger().warning(
            f'{"KEP" if muon == GripperStatus.GRIP_STATE_CLOSED else "NHA"} that bai: {ly_do}')
        self.state = GripperStatus.GRIP_STATE_ERROR
        self.target_state = None    # lenh lap lai ke tiep se gui lai
        self.move_done_s = None

    def on_command(self, msg):
        muon = (GripperStatus.GRIP_STATE_CLOSED if msg.command == GripperCommand.GRIPPER_CLOSE
                else GripperStatus.GRIP_STATE_OPEN)
        if muon == self.target_state:
            return      # mission_manager phat lai deu dan - khong khoi dong lai hanh trinh
        self.target_state = muon
        self.state = GripperStatus.GRIP_STATE_MOVING
        self.move_done_s = None
        if self.get_parameter('simulate').value:
            self.move_done_s = self.now_s() + self.get_parameter('travel_s').value
        else:
            self.set_servo(muon)
        self.get_logger().info(
            f'lenh {"CLOSE" if muon == GripperStatus.GRIP_STATE_CLOSED else "OPEN"} (seq {msg.seq})')

    def publish_status(self):
        if (self.state == GripperStatus.GRIP_STATE_MOVING and self.move_done_s is not None
                and self.now_s() >= self.move_done_s):
            self.state = self.target_state
            self.move_done_s = None

        msg = GripperStatus()
        msg.stamp = self.get_clock().now().to_msg()
        msg.state = self.state
        # Khong co cam bien: "da kep" = FC nhan lenh kep va da cho het travel_s (xem dau file).
        msg.sensor_confirmed = self.state == GripperStatus.GRIP_STATE_CLOSED
        if self.get_parameter('simulate').value and msg.sensor_confirmed:
            msg.force_reading_n = SIM_FORCE_N
        else:
            msg.force_reading_n = 0.0
        self.pub_status.publish(msg)


def main(args=None):
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = GripperControllerNode()
    # EventsExecutor: executor mac dinh cua rclpy dung lai wait-set moi lan thuc day, ton phan
    # lon CPU tren Pi 4 (xem mission_manager_node).
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
