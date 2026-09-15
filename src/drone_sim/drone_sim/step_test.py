"""step_test - thu dap ung buoc cua position_controller_node trong Gazebo va in chi so.

Chup vi tri hien tai lam diem dau, giu diem dau settle_s giay, doi setpoint mot buoc tren mot
truc, ghi /odometry/filtered duration_s giay, in chi so; --back thi buoc nguoc ve va do lan nua.
Phat /mission/setpoint 5 Hz (position_controller_node bo setpoint cu hon 0,5 s).

  ros2 run drone_sim step_test --axis z --step 2.0
  ros2 run drone_sim step_test --axis x --step 2.0 --back
"""

import argparse
import time

import rclpy
from rclpy.parameter import Parameter
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import PositionTarget
from nav_msgs.msg import Odometry
from rclpy.qos import QoSProfile, QoSReliabilityPolicy

from drone_sim.step_response import analyze

AXES = {'x': 0, 'y': 1, 'z': 2}
SAT_MPS = 1.88                  # position_controller_node kep ngang 1,9 m/s (95 % tran FC)


class StepTest:

    def __init__(self, node):
        self.node = node
        self.pos = None
        self.cmd = None
        self.record = None
        best_effort = QoSProfile(depth=1, reliability=QoSReliabilityPolicy.BEST_EFFORT)
        node.create_subscription(Odometry, '/odometry/filtered', self.on_odom, best_effort)
        node.create_subscription(
            PositionTarget, '/mavros/setpoint_raw/local', self.on_cmd, best_effort)
        self.pub = node.create_publisher(
            PoseStamped, '/mission/setpoint', QoSProfile(depth=10))

    def now_s(self):
        return self.node.get_clock().now().nanoseconds / 1e9

    def on_odom(self, msg):
        p = msg.pose.pose.position
        self.pos = (p.x, p.y, p.z)
        if self.record is not None:
            self.record['pos'].append((self.now_s(), self.pos))

    def on_cmd(self, msg):
        self.cmd = (msg.velocity.x, msg.velocity.y, msg.velocity.z)
        if self.record is not None:
            self.record['cmd'].append(self.cmd)

    def hold(self, target, seconds):
        """Phat target 5 Hz trong seconds giay (tinh theo dong ho node - sim time neu co)."""
        end = self.now_s() + seconds
        next_pub = 0.0
        while rclpy.ok() and self.now_s() < end:
            if self.now_s() >= next_pub:
                msg = PoseStamped()
                msg.header.stamp = self.node.get_clock().now().to_msg()
                msg.header.frame_id = 'odom'
                msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = target
                msg.pose.orientation.w = 1.0
                self.pub.publish(msg)
                next_pub = self.now_s() + 0.2
            rclpy.spin_once(self.node, timeout_sec=0.02)

    def step(self, start, target, axis, duration_s, band_m):
        self.record = {'pos': [], 'cmd': []}
        t0 = self.now_s()
        self.hold(target, duration_s)
        rec, self.record = self.record, None
        i = AXES[axis]
        samples = [(t - t0, p[i]) for t, p in rec['pos']]
        if not samples:
            print('KHONG nhan /odometry/filtered trong luc do - kiem sim_fc_bridge_node')
            return
        r = analyze(samples, start[i], target[i], band_m)
        sat = sum(1 for c in rec['cmd'] if max(abs(c[0]), abs(c[1])) >= SAT_MPS)
        sat_pct = 100.0 * sat / max(1, len(rec['cmd']))
        fmt = lambda v, unit: '-' if v is None else f'{v:.2f} {unit}'  # noqa: E731
        print(f'buoc {axis}: {start[i]:+.2f} -> {target[i]:+.2f} m | '
              f'rise {fmt(r["rise_s"], "s")} | vuot {r["overshoot_pct"]:.1f} % | '
              f'on dinh (+-{band_m} m) {fmt(r["settle_s"], "s")} | '
              f'sai so cuoi {r["final_error_m"]:.3f} m | lenh ngang bao hoa {sat_pct:.0f} %')


def main(args=None):
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('--axis', choices=AXES, required=True)
    parser.add_argument('--step', type=float, required=True, help='do dai buoc (m), co dau')
    parser.add_argument('--settle', type=float, default=3.0, help='giu diem dau truoc buoc (s)')
    parser.add_argument('--duration', type=float, default=10.0, help='ghi sau buoc (s)')
    parser.add_argument('--band', type=float, default=0.1, help='dai coi la on dinh (m)')
    parser.add_argument('--back', action='store_true', help='buoc nguoc ve va do lan nua')
    opts, ros_args = parser.parse_known_args()

    rclpy.init(args=ros_args)
    # Thoi gian Gazebo (/clock): PC cham (real time factor < 1) thi chi so van dung giay mo phong.
    node = rclpy.create_node('step_test',
                             parameter_overrides=[Parameter('use_sim_time', value=True)])
    test = StepTest(node)
    try:
        deadline = time.monotonic() + 10.0
        waiting = lambda: test.pos is None or test.now_s() == 0.0  # noqa: E731
        while rclpy.ok() and waiting() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
        if waiting():
            print('KHONG nhan /odometry/filtered hoac /clock sau 10 s - Gazebo/sim_fc_bridge_node '
                  'chua chay?')
            return
        start = test.pos
        target = list(start)
        target[AXES[opts.axis]] += opts.step
        target = tuple(target)
        print(f'giu {start[0]:+.2f} {start[1]:+.2f} {start[2]:+.2f} trong {opts.settle:.0f} s')
        test.hold(start, opts.settle)
        test.step(start, target, opts.axis, opts.duration, opts.band)
        if opts.back:
            test.step(target, start, opts.axis, opts.duration, opts.band)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
