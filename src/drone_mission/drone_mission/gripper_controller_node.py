"""gripper_controller_node - dieu khien co cau gap/tha va la NGUON XAC NHAN CUNG DUY NHAT
rang hang da gap/tha. mission_manager_node khong duoc phep suy doan qua timeout.

Diem de sai: hang nhe hoac gap lech co the khong kich hoat cong tac hanh trinh du da gap -
vi vay status mang ca force_reading_n chu khong chi mot co boolean.

Chay duoc khi khong co phan cung: dat tham so simulate=true (mac dinh) de test phan ROS
truoc khi lap servo that.
"""

import rclpy
from rclpy.node import Node

from drone_interfaces.msg import GripperCommand, GripperStatus
from drone_mission.qos import EVENT_QOS


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

        self.pi = None
        if not self.get_parameter('simulate').value:
            self.setup_gpio()

        self.target_state = GripperStatus.GRIP_STATE_OPEN

        self.create_subscription(GripperCommand, '/gripper/command', self.on_command, EVENT_QOS)
        self.pub_status = self.create_publisher(GripperStatus, '/gripper/status', EVENT_QOS)
        self.create_timer(1.0 / self.get_parameter('status_rate_hz').value, self.publish_status)

    def setup_gpio(self):
        """TODO: khoi tao pigpio (can pigpiod dang chay), dat che do INPUT + pull-up cho
        confirm_switch_gpio. Bao loi ro rang neu khong ket noi duoc pigpiod."""
        self.get_logger().warn('Che do phan cung chua duoc hien thuc, dang chay nhu simulate')

    def on_command(self, msg):
        """TODO: dat servo pulse theo open/close, chuyen target_state sang GRIP_STATE_MOVING."""
        del msg

    def read_confirm_switch(self):
        """TODO: doc cong tac hanh trinh (active-low) co debounce confirm_debounce_ms."""
        return False

    def read_force_sensor(self):
        """TODO: doc cam bien luc qua ADC; tra 0.0 neu khong lap cam bien."""
        return 0.0

    def publish_status(self):
        """TODO: gop confirm switch + force sensor thanh state, publish GripperStatus."""


def main(args=None):
    rclpy.init(args=args)
    node = GripperControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
