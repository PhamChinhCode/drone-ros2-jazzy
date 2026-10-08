"""gps_odom_node - dua vi tri GPS vao khung ban do tag (odom) cho EKF.

GPS MG-F10-A tren FC -> GPS_RAW_INT (giao uoc FC 1.8) -> MAVROS gps_status ->
/mavros/gpsstatus/gps1/raw. Node nay doi lat/lon sang (x, y) cua khung ban do theo goc WGS84 cua
ban do (tags.yaml: geo_origin_*, GCS gui kem ban do tag - giao uoc GCS 8.7) va phat
/gps/pose_odom cho pose1 cua robot_localization.

CHUA CO GOC BAN DO THI KHONG PHAT GI. Khong doan goc: goc sai la keo drone ve mot toa do sai mot
cach tu tin, te hon la khong co GPS.

Chi fuse x, y. Do cao GPS sai so gap 1,5-2 lan ngang va troi cham - laser + baro cua FC va z cua
marker da tot hon nhieu o do cao bay giao hang.
"""

from concurrent.futures import ThreadPoolExecutor
import math
import signal
import threading

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from mavros_msgs.msg import GPSRAW
from rclpy.experimental import EventsExecutor
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions

from drone_estimation.geo import gps_measurement, lla_to_map, origin_from_params
from drone_estimation.qos import SENSOR_QOS

LARGE_VARIANCE = 1e6   # truc khong do (z, goc) - EKF chi fuse x, y cua pose1


class GpsOdomNode(Node):

    def __init__(self):
        super().__init__('gps_odom_node')

        # Goc WGS84 cua ban do tag - nap tu tags.yaml (hoac tags_override.yaml do GCS gui).
        self.declare_parameter('geo_origin_valid', False)
        self.declare_parameter('geo_origin_lat', 0.0)
        self.declare_parameter('geo_origin_lon', 0.0)
        self.declare_parameter('geo_origin_alt', 0.0)
        self.declare_parameter('geo_north_yaw_deg', 0.0)
        # Cong loc mau. h_acc la sai so 1-sigma module tu bao (NAV-PVT); MG-F10 L1+L5 ngoai troi
        # thoang cho ~1-2 m. 3 m chan nghiem xau ma van con dung duoc canh nha cao.
        self.declare_parameter('min_fix_type', 3)
        self.declare_parameter('max_h_acc_m', 3.0)
        self.declare_parameter('min_sats', 6)
        # Nhan sai so module de phong h_acc lac quan (sai so GPS tuong quan theo thoi gian, EKF lai
        # coi moi mau doc lap). 1.5 -> phuong sai x2,25.
        self.declare_parameter('h_acc_scale', 1.5)
        self.declare_parameter('target_frame', 'odom')

        g = self.get_parameter
        self.origin = origin_from_params(g('geo_origin_valid').value, g('geo_origin_lat').value,
                                         g('geo_origin_lon').value, g('geo_origin_alt').value,
                                         g('geo_north_yaw_deg').value)
        if self.origin is None:
            self.get_logger().warning(
                'CHUA CO GOC BAN DO (geo_origin_valid = false) - GPS KHONG vao EKF. '
                'Dat goc tren GCS (trang thiet ke khu vuc) roi nap ban do tag.')
        else:
            o = self.origin
            self.get_logger().info(f'goc ban do: {o.lat_deg:.7f}, {o.lon_deg:.7f}, {o.alt_m:.1f} m, '
                                   f'truc N lech Bac that {o.north_yaw_deg:.1f} do')

        self.last_reason = None
        self.create_subscription(GPSRAW, '/mavros/gpsstatus/gps1/raw', self.on_gps, SENSOR_QOS)
        self.pub_pose = self.create_publisher(PoseWithCovarianceStamped, '/gps/pose_odom', SENSOR_QOS)

    def on_gps(self, msg):
        if self.origin is None:
            return
        g = self.get_parameter
        h_acc_m = msg.h_acc / 1000.0
        ok, reason = gps_measurement(msg.fix_type, h_acc_m, msg.satellites_visible,
                                     g('min_fix_type').value, g('max_h_acc_m').value,
                                     g('min_sats').value)
        # Chi log khi doi trang thai - khong spam 5 Hz.
        if reason != self.last_reason:
            if ok:
                self.get_logger().info(f'GPS vao EKF: fix {msg.fix_type}, '
                                       f'{msg.satellites_visible} ve tinh, h_acc {h_acc_m:.2f} m')
            else:
                self.get_logger().warning(f'GPS KHONG vao EKF: {reason}')
            self.last_reason = reason
        if not ok:
            return

        x, y, z = lla_to_map(msg.lat * 1e-7, msg.lon * 1e-7, msg.alt * 1e-3, self.origin)
        var = (h_acc_m * g('h_acc_scale').value) ** 2

        out = PoseWithCovarianceStamped()
        out.header.stamp = msg.header.stamp
        out.header.frame_id = g('target_frame').value
        out.pose.pose.position.x = x
        out.pose.pose.position.y = y
        out.pose.pose.position.z = z
        out.pose.pose.orientation.w = 1.0
        cov = [0.0] * 36
        cov[0] = cov[7] = var
        for i in range(2, 6):
            cov[i * 7] = LARGE_VARIANCE
        out.pose.covariance = cov
        if all(math.isfinite(v) for v in (x, y)):
            self.pub_pose.publish(out)


def main(args=None):
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = GpsOdomNode()
    # EventsExecutor: executor mac dinh cua rclpy ton phan lon CPU tren Pi 4 (xem ekf_health_node).
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
