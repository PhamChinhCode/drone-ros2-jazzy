"""range_vertical_node - laser MTF01P (doc truc than) -> do cao thang dung, bu nghieng bang IMU.

Laser gan theo than: nghieng goc a thi so doc dai ra 1/cos(a) (25 do -> +10 %). /range/vertical la
nguon do cao laser DUY NHAT cho cac node Pi (optical flow, dieu khien, nhiem vu).

Khong hop le (FC bao mat laser - hop dong 1.6 gui 0 -, nghieng qua nguong, hoac IMU qua han) ->
range = NaN: moi node dung kiem min_range <= range <= max_range nen tu loai, khong phai sua them.

/range/pose_z (2026-10-08): cung do cao do, quy thanh z trong khung ban do cho EKF (pose2, CHI z).
Truoc do EKF khong co nguon z tuyet doi nao giua hai bai: chi vz cua FC + IMU nen z troi dan.
Gia dinh MAT DAT PHANG tai z = ground_z_m (ban do tag dat tren nen phang, tag ~ z 0). Bay qua vat
cao (ban, thung, bac) thi laser bao thap hon that - pose2_rejection_threshold chan buoc nhay dot
ngot, nhung doan dai tren vat cao van keo z xuong. Vi tri lap laser chua do: coi nhu o tam than.
"""

from concurrent.futures import ThreadPoolExecutor
import math
import signal
import threading

import rclpy
from rclpy.experimental import EventsExecutor
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from geometry_msgs.msg import PoseWithCovarianceStamped
from sensor_msgs.msg import Imu, Range

from drone_estimation.estimation_math import tilt_cos_from_quaternion, vertical_range
from drone_estimation.qos import SENSOR_QOS


class RangeVerticalNode(Node):

    def __init__(self):
        super().__init__('range_vertical_node')
        # Cung nguong voi ekf_altitude cua FC (EST_RANGE_MAX_TILT_DEG, EST_RANGE_TILT_HYST_DEG).
        self.declare_parameter('max_tilt_deg', 25.0)
        self.declare_parameter('tilt_hyst_deg', 3.0)
        self.declare_parameter('imu_timeout_s', 0.2)
        # z ban do = ground_z_m + do cao laser. Lech chuan: luong tu 1 cm + sai so do ~1 cm + mat
        # dat khong phang hoan toan -> 3 cm (EKF tin hon z marker, phuong sai 0,01 m^2).
        self.declare_parameter('ground_z_m', 0.0)
        self.declare_parameter('z_stdev_m', 0.03)
        self.declare_parameter('map_frame', 'odom')

        self.tilt_cos = None
        self.imu_stamp_s = None
        self.blocked = False

        self.create_subscription(Imu, '/mavros/imu/data', self.on_imu, SENSOR_QOS)
        self.create_subscription(Range, '/mavros/mtf01p', self.on_range, SENSOR_QOS)
        self.pub = self.create_publisher(Range, '/range/vertical', SENSOR_QOS)
        self.pub_z = self.create_publisher(PoseWithCovarianceStamped, '/range/pose_z', SENSOR_QOS)

    def now_s(self):
        return self.get_clock().now().nanoseconds / 1e9

    def on_imu(self, msg):
        q = msg.orientation
        self.tilt_cos = tilt_cos_from_quaternion((q.x, q.y, q.z, q.w))
        self.imu_stamp_s = self.now_s()

    def on_range(self, msg):
        out = Range()
        out.header = msg.header
        out.radiation_type = msg.radiation_type
        out.field_of_view = msg.field_of_view
        out.min_range, out.max_range = msg.min_range, msg.max_range
        out.range = math.nan

        imu_ok = (self.imu_stamp_s is not None and
                  self.now_s() - self.imu_stamp_s <= self.get_parameter('imu_timeout_s').value)
        if imu_ok:
            h, self.blocked = vertical_range(
                msg.range, msg.min_range, msg.max_range, self.tilt_cos, self.blocked,
                self.get_parameter('max_tilt_deg').value,
                self.get_parameter('tilt_hyst_deg').value)
            if h is not None:
                # Giu [min, max] cung ti le de mot so do hop le van nam trong dai sau khi bu.
                out.range = h
                out.min_range = msg.min_range * self.tilt_cos
                out.max_range = msg.max_range * self.tilt_cos
                self.publish_pose_z(msg.header.stamp, h)
        self.pub.publish(out)

    def publish_pose_z(self, stamp, h):
        """Chi z co nghia; x, y, huong de phuong sai rat lon (ekf.yaml pose2_config chi bat z)."""
        p = PoseWithCovarianceStamped()
        p.header.stamp = stamp
        p.header.frame_id = self.get_parameter('map_frame').value
        p.pose.pose.position.z = self.get_parameter('ground_z_m').value + h
        p.pose.pose.orientation.w = 1.0
        cov = [0.0] * 36
        for i in (0, 7, 21, 28, 35):
            cov[i] = 1e6
        cov[14] = self.get_parameter('z_stdev_m').value ** 2
        p.pose.covariance = cov
        self.pub_z.publish(p)


def main(args=None):
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = RangeVerticalNode()
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
