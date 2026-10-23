"""log_activity_node - danh dau luc drone HOAT DONG de prune_logs giu bag doan do.

Moi giay dang hoat dong (FC arm / Pi co quyen / co nhiem vu) ghi them mot moc thoi gian (giay
epoch) vao ~/drone_logs/hoat_dong.txt. Logic giu/xoa: log_retention.py.
"""

import os
import time

import rclpy
from mavros_msgs.msg import DebugValue, State
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy

from drone_interfaces.msg import MissionState
from drone_safety.log_retention import ACTIVE_FILE, la_hoat_dong
from drone_safety.qos import EVENT_QOS

NAMED_VALUE_QOS = QoSProfile(depth=20, reliability=QoSReliabilityPolicy.BEST_EFFORT)
STALE_S = 1.5              # nhu fc_link.DEFAULT_STALE_S: OB_AUTH phat 2 Hz
KHONG_NHIEM_VU = (MissionState.IDLE, MissionState.MISSION_COMPLETE)


class LogActivityNode(Node):

    def __init__(self):
        super().__init__('log_activity_node')
        self.declare_parameter('log_dir', os.path.expanduser('~/drone_logs'))
        self.path = os.path.join(self.get_parameter('log_dir').value, ACTIVE_FILE)
        os.makedirs(os.path.dirname(self.path), exist_ok=True)
        self.armed = False
        self.auth = None            # (gia tri OB_AUTH, luc nhan)
        self.mission_state = None
        self.active = False
        self.create_subscription(State, '/mavros/state', self.on_state, EVENT_QOS)
        self.create_subscription(DebugValue, '/mavros/debug_value/named_value_int',
                                 self.on_named_value, NAMED_VALUE_QOS)
        self.create_subscription(MissionState, '/mission/state', self.on_mission, EVENT_QOS)
        self.create_timer(1.0, self.tick)

    def on_state(self, msg):
        self.armed = msg.armed

    def on_named_value(self, msg):
        if msg.name == 'OB_AUTH':
            self.auth = (msg.value_int, time.monotonic())

    def on_mission(self, msg):
        self.mission_state = msg.state

    def tick(self):
        auth = None
        if self.auth is not None and time.monotonic() - self.auth[1] <= STALE_S:
            auth = self.auth[0] == 1
        mission = self.mission_state is not None and self.mission_state not in KHONG_NHIEM_VU
        active = la_hoat_dong(self.armed, auth, mission)
        if active != self.active:
            self.active = active
            self.get_logger().info(
                f'{"BAT" if active else "HET"} hoat dong (arm={self.armed} quyen={auth} '
                f'nhiem_vu={mission}) - bag {"giu" if active else "giu them 2 phut"}')
        if active:
            with open(self.path, 'a', encoding='ascii') as f:
                f.write(f'{time.time():.0f}\n')


def main(args=None):
    rclpy.init(args=args)
    node = LogActivityNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
