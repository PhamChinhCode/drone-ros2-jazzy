"""gripper_controller_node - dieu khien co cau gap/tha va la NGUON XAC NHAN CUNG DUY NHAT
rang hang da gap/tha. mission_manager_node khong duoc phep suy doan qua timeout.

Diem de sai: hang nhe hoac gap lech co the khong kich hoat cong tac hanh trinh du da gap -
vi vay status mang ca force_reading_n chu khong chi mot co boolean.

Chay duoc khi khong co phan cung: simulate=true (mac dinh) gia lap hanh trinh servo va cong tac
xac nhan, du de chay het chuoi ACTUATE_GRIPPER trong Gazebo. Muon thu duong THAT BAI (gripper
khong xac nhan -> failsafe FS_GRIP_CONFIRM_FAIL) thi khong chay node nay.
"""

import rclpy
from rclpy.experimental import EventsExecutor
from rclpy.node import Node

from drone_interfaces.msg import GripperCommand, GripperStatus
from drone_mission.qos import EVENT_QOS

SIM_FORCE_N = 2.5      # luc gia lap khi da gap chac; khong co y nghia do luong


class GripperControllerNode(Node):

    def __init__(self):
        super().__init__('gripper_controller_node')

        self.declare_parameter('simulate', True)
        self.declare_parameter('servo_gpio', 18)
        self.declare_parameter('confirm_switch_gpio', 23)
        # Do tay theo co cau co khi that truoc khi chot hai gia tri nay.
        self.declare_parameter('open_pulse_us', 1000)
        self.declare_parameter('close_pulse_us', 2000)
        self.declare_parameter('confirm_debounce_ms', 50)
        self.declare_parameter('force_threshold_n', 1.0)
        self.declare_parameter('status_rate_hz', 10.0)
        # Thoi gian co cau chay het hanh trinh khi simulate. Servo that do lai roi sua.
        self.declare_parameter('sim_travel_s', 0.8)

        self.pi = None
        if not self.get_parameter('simulate').value:
            self.setup_gpio()

        # Khoi dong o trang thai MO: an toan hon: khong giu hang khi chua ai ra lenh.
        self.state = GripperStatus.GRIP_STATE_OPEN
        self.target_state = GripperStatus.GRIP_STATE_OPEN
        self.move_done_s = None

        self.create_subscription(GripperCommand, '/gripper/command', self.on_command, EVENT_QOS)
        self.pub_status = self.create_publisher(GripperStatus, '/gripper/status', EVENT_QOS)
        self.create_timer(1.0 / self.get_parameter('status_rate_hz').value, self.publish_status)

    def now_s(self):
        return self.get_clock().now().nanoseconds / 1e9

    def setup_gpio(self):
        """TODO: khoi tao pigpio (can pigpiod dang chay), dat che do INPUT + pull-up cho
        confirm_switch_gpio. Bao loi ro rang neu khong ket noi duoc pigpiod."""
        self.get_logger().warn('Che do phan cung chua duoc hien thuc, dang chay nhu simulate')

    def set_servo(self, dong):
        """TODO: dat servo pulse (close_pulse_us / open_pulse_us) qua pigpio."""

    def on_command(self, msg):
        muon = (GripperStatus.GRIP_STATE_CLOSED if msg.command == GripperCommand.GRIPPER_CLOSE
                else GripperStatus.GRIP_STATE_OPEN)
        if muon == self.target_state:
            return      # mission_manager phat lai deu dan - khong khoi dong lai hanh trinh
        self.target_state = muon
        self.state = GripperStatus.GRIP_STATE_MOVING
        self.move_done_s = self.now_s() + self.get_parameter('sim_travel_s').value
        if not self.get_parameter('simulate').value:
            self.set_servo(muon == GripperStatus.GRIP_STATE_CLOSED)
        self.get_logger().info(
            f'lenh {"CLOSE" if muon == GripperStatus.GRIP_STATE_CLOSED else "OPEN"} (seq {msg.seq})')

    def read_confirm_switch(self):
        """TODO: doc cong tac hanh trinh (active-low) co debounce confirm_debounce_ms."""
        return False

    def read_force_sensor(self):
        """TODO: doc cam bien luc qua ADC; tra 0.0 neu khong lap cam bien."""
        return 0.0

    def publish_status(self):
        if (self.state == GripperStatus.GRIP_STATE_MOVING and self.move_done_s is not None
                and self.now_s() >= self.move_done_s):
            self.state = self.target_state
            self.move_done_s = None

        msg = GripperStatus()
        msg.stamp = self.get_clock().now().to_msg()
        msg.state = self.state
        if self.get_parameter('simulate').value:
            # Gia lap: co cau luon chay dung. sensor_confirmed = dang giu hang.
            msg.sensor_confirmed = self.state == GripperStatus.GRIP_STATE_CLOSED
            msg.force_reading_n = SIM_FORCE_N if msg.sensor_confirmed else 0.0
        else:
            # Cong tac hanh trinh la nguon chinh; cam bien luc do them de bat truong hop gap lech
            # ma cong tac van dong (xem chu thich dau file).
            msg.force_reading_n = self.read_force_sensor()
            msg.sensor_confirmed = (self.read_confirm_switch()
                                    or msg.force_reading_n
                                    >= self.get_parameter('force_threshold_n').value)
            if msg.sensor_confirmed and self.state != GripperStatus.GRIP_STATE_MOVING:
                msg.state = self.state = GripperStatus.GRIP_STATE_CLOSED
        self.pub_status.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = GripperControllerNode()
    # EventsExecutor: executor mac dinh cua rclpy dung lai wait-set moi lan thuc day, ton phan
    # lon CPU tren Pi 4 (xem mission_manager_node).
    executor = EventsExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
