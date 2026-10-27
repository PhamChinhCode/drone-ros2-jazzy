#!/usr/bin/env python3
"""gps_survey - do goc WGS84 cua ban do tag: dat module GPS ngay tren tam tag home roi lay trung binh.

    source ~/ros2_ws/install/setup.bash
    python3 ~/ros2_ws/scripts/gps_survey.py --seconds 180

Doc /mavros/gpsstatus/gps1/raw (GPS_RAW_INT cua FC), chi lay mau fix 3D tro len, du so ve tinh va
h_acc du nho. In trung binh lat/lon/alt + do lech chuan (m) de nhap vao trang thiet ke khu vuc
cua GCS (goc ban do). Trong luc do KHONG di chuyen drone.
"""

import argparse
import math
import statistics
import time

import rclpy
from mavros_msgs.msg import GPSRAW
from rclpy.qos import qos_profile_sensor_data

M_PER_DEG_LAT = 111320.0


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--seconds', type=float, default=180.0)
    ap.add_argument('--min-sats', type=int, default=10)
    ap.add_argument('--max-hacc', type=float, default=1.5, help='m')
    a = ap.parse_args()

    rclpy.init()
    node = rclpy.create_node('gps_survey')
    lat, lon, alt, bo = [], [], [], [0]
    last = [0.0]

    def on_raw(m):
        h = m.h_acc / 1000.0
        ok = m.fix_type >= 3 and m.satellites_visible >= a.min_sats and 0 < h <= a.max_hacc
        if ok:
            lat.append(m.lat * 1e-7)
            lon.append(m.lon * 1e-7)
            alt.append(m.alt * 1e-3)
        else:
            bo[0] += 1
        now = time.monotonic()
        if now - last[0] >= 5.0:
            last[0] = now
            print(f'fix {m.fix_type} sv {m.satellites_visible} hacc {h:.2f} m | '
                  f'mau dat {len(lat)}, bo {bo[0]}', flush=True)

    node.create_subscription(GPSRAW, '/mavros/gpsstatus/gps1/raw', on_raw, qos_profile_sensor_data)
    het = time.monotonic() + a.seconds
    try:
        while time.monotonic() < het:
            rclpy.spin_once(node, timeout_sec=0.2)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.try_shutdown()

    if len(lat) < 20:
        raise SystemExit(f'Chi {len(lat)} mau dat - chua co fix tot (can fix 3D, >= {a.min_sats} sv, '
                         f'hacc <= {a.max_hacc} m).')
    la, lo, al = statistics.fmean(lat), statistics.fmean(lon), statistics.fmean(alt)
    sn = statistics.stdev(lat) * M_PER_DEG_LAT
    se = statistics.stdev(lon) * M_PER_DEG_LAT * math.cos(math.radians(la))
    print(f'\n{len(lat)} mau ({bo[0]} bi bo)')
    print(f'goc: lat {la:.7f}  lon {lo:.7f}  alt {al:.2f} m (AMSL)')
    print(f'do lech chuan: N {sn:.2f} m, E {se:.2f} m, cao {statistics.stdev(alt):.2f} m')


if __name__ == '__main__':
    main()
