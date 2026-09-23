"""gps_survey - do goc WGS84 cua ban do tag bang chinh GPS tren drone.

Dat drone NAM YEN dung tam pad_home (tag 0, goc ban do), ngoai troi thoang, cho GPS co fix roi chay:

    ros2 run drone_estimation gps_survey --ros-args -p seconds:=120

Lay trung binh cac mau fix 3D, in lat/lon/alt de nhap vao trang thiet ke khu vuc cua GCS (goc
Site), roi nap lai ban do tag. Chi doc, khong ghi gi.

Do lech chuan in ra la DO PHAN TAN trong luc do, khong phai sai so tuyet doi: GPS mot tan so troi
cham vai chuc cm trong vai phut. L1+L5 tot hon, nhung do 2 phut van chi cho goc dung ~0,5-1 m.
"""

import math
import statistics

import rclpy
from mavros_msgs.msg import GPSRAW
from rclpy.node import Node

from drone_estimation.geo import GeoOrigin, lla_to_map
from drone_estimation.qos import SENSOR_QOS


class GpsSurvey(Node):

    def __init__(self):
        super().__init__('gps_survey')
        self.declare_parameter('seconds', 120.0)
        self.declare_parameter('min_fix_type', 3)
        self.samples = []
        self.h_acc = []
        self.t0 = None
        self.create_subscription(GPSRAW, '/mavros/gpsstatus/gps1/raw', self.on_gps, SENSOR_QOS)

    def on_gps(self, msg):
        if msg.fix_type < self.get_parameter('min_fix_type').value:
            return
        now = self.get_clock().now().nanoseconds / 1e9
        if self.t0 is None:
            self.t0 = now
            self.get_logger().info('co fix - bat dau lay mau')
        self.samples.append((msg.lat * 1e-7, msg.lon * 1e-7, msg.alt * 1e-3))
        self.h_acc.append(msg.h_acc / 1000.0)

    def done(self):
        if self.t0 is None:
            return False
        return self.get_clock().now().nanoseconds / 1e9 - self.t0 >= \
            self.get_parameter('seconds').value


def main(args=None):
    rclpy.init(args=args)
    node = GpsSurvey()
    try:
        while rclpy.ok() and not node.done():
            rclpy.spin_once(node, timeout_sec=1.0)
            if node.t0 is None:
                node.get_logger().info('dang cho GPS fix 3D...', throttle_duration_sec=10.0)
    except KeyboardInterrupt:
        pass
    s = node.samples
    if len(s) < 10:
        print(f'Qua it mau ({len(s)}) - chua co fix on dinh.')
    else:
        lat = statistics.fmean(p[0] for p in s)
        lon = statistics.fmean(p[1] for p in s)
        alt = statistics.fmean(p[2] for p in s)
        o = GeoOrigin(lat, lon, alt)
        xy = [lla_to_map(p[0], p[1], p[2], o) for p in s]
        sx = statistics.pstdev(v[0] for v in xy)
        sy = statistics.pstdev(v[1] for v in xy)
        sz = statistics.pstdev(v[2] for v in xy)
        print(f'{len(s)} mau fix 3D, h_acc trung vi {statistics.median(node.h_acc):.2f} m')
        print(f'phan tan: E {sx:.2f} m, N {sy:.2f} m, cao {sz:.2f} m '
              f'(ngang {math.hypot(sx, sy):.2f} m)')
        print('Goc ban do (nhap vao Site tren GCS):')
        print(f'  origin_lat   = {lat:.7f}')
        print(f'  origin_lon   = {lon:.7f}')
        print(f'  origin_alt_m = {alt:.2f}')
    node.destroy_node()
    rclpy.try_shutdown()
