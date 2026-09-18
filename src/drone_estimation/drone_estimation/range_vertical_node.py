"""range_vertical_node - laser MTF01P (doc truc than) -> do cao thang dung, bu nghieng bang IMU.

Laser gan theo than: nghieng goc a thi so doc dai ra 1/cos(a) (25 do -> +10 %). /range/vertical la
nguon do cao laser DUY NHAT cho cac node Pi (optical flow, dieu khien, nhiem vu).

Khong hop le (FC bao mat laser - hop dong 1.6 gui 0 -, nghieng qua nguong, hoac IMU qua han) ->
range = NaN: moi node dung kiem min_range <= range <= max_range nen tu loai, khong phai sua them.
"""

import math

import rclpy
from rclpy.experimental import EventsExecutor
from rclpy.node import Node
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

        self.tilt_cos = None
        self.imu_stamp_s = None
        self.blocked = False

        self.create_subscription(Imu, '/mavros/imu/data', self.on_imu, SENSOR_QOS)
        self.create_subscription(Range, '/mavros/mtf01p', self.on_range, SENSOR_QOS)
        self.pub = self.create_publisher(Range, '/range/vertical', SENSOR_QOS)

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
        self.pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = RangeVerticalNode()
    executor = EventsExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
