#!/usr/bin/env python3
"""Chụp một khung hình kèm marker đã phát hiện, vẽ đè lên rồi ghi ra PNG.

Dùng khi máy không có GUI: mở file PNG kết quả bằng VS Code / SCP về máy khác.

Dùng:
  ./tools/snapshot_detections.py                    # ghi ra /tmp/apriltag_snapshot.png
  ./tools/snapshot_detections.py --out anh.png
  ./tools/snapshot_detections.py --topic /camera/image_raw
"""

import argparse

import cv2
import numpy as np
import rclpy
from apriltag_msgs.msg import AprilTagDetectionArray
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image

XANH = (0, 255, 0)
VANG = (0, 255, 255)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--out', default='/tmp/apriltag_snapshot.png')
    ap.add_argument('--topic', default='/camera/image_rect')
    ap.add_argument('--timeout', type=float, default=15.0)
    a = ap.parse_args()

    rclpy.init()
    node = rclpy.create_node('snapshot_detections')
    trang_thai = {'anh': None, 'xong': False}

    def on_image(msg):
        if msg.encoding != 'mono8':
            node.get_logger().error(f'chi ho tro mono8, nhan duoc {msg.encoding}')
            trang_thai['xong'] = True
            return
        khung = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width)
        trang_thai['anh'] = cv2.cvtColor(khung, cv2.COLOR_GRAY2BGR)

    def on_detections(msg):
        anh = trang_thai['anh']
        if anh is None or trang_thai['xong']:
            return
        anh = anh.copy()
        for d in msg.detections:
            goc = np.array([[c.x, c.y] for c in d.corners], dtype=np.int32)
            cv2.polylines(anh, [goc], True, XANH, 2)
            cv2.circle(anh, (int(d.centre.x), int(d.centre.y)), 4, VANG, -1)
            canh = [np.linalg.norm(goc[i] - goc[(i + 1) % 4]) for i in range(4)]
            cv2.putText(anh, f'id={d.id} margin={d.decision_margin:.0f}',
                        (int(d.centre.x) - 60, int(d.centre.y) - 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, XANH, 1, cv2.LINE_AA)
            node.get_logger().info(
                f'id={d.id} tam=({d.centre.x:.1f},{d.centre.y:.1f}) '
                f'canh_trung_binh={np.mean(canh):.1f}px margin={d.decision_margin:.0f}')
        cv2.putText(anh, f'{len(msg.detections)} marker', (8, 20),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, VANG, 1, cv2.LINE_AA)
        cv2.imwrite(a.out, anh)
        node.get_logger().info(f'da ghi {a.out}')
        trang_thai['xong'] = True

    node.create_subscription(Image, a.topic, on_image, qos_profile_sensor_data)
    node.create_subscription(AprilTagDetectionArray, '/apriltag/detections',
                             on_detections, qos_profile_sensor_data)

    het_gio = node.get_clock().now().nanoseconds + a.timeout * 1e9
    while rclpy.ok() and not trang_thai['xong']:
        rclpy.spin_once(node, timeout_sec=0.1)
        if node.get_clock().now().nanoseconds > het_gio:
            node.get_logger().error(f'het {a.timeout}s ma chua co detection nao')
            break
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
