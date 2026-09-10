#!/usr/bin/env python3
"""Kiểm chứng kết quả hiệu chỉnh bằng thước: đặt bàn cờ ở khoảng cách ĐÃ BIẾT rồi
so khoảng cách script tính ra với số đo thật.

Đây là phép thử end-to-end: script lấy K/D từ topic /camera_info (đúng file
~/.ros/camera_info/unicam.yaml mà node đang nạp) rồi chạy solvePnP y hệt cách
ArUco/AprilTag sẽ làm. Khớp ở đây nghĩa là đo pose khi bay cũng khớp.

Dùng (node camera phải ĐANG CHẠY ở terminal khác):

  # đặt bàn cờ vuông góc trục ống kính, cách 1.00 m, rồi:
  ./scripts/check_calibration.py --distance 1.0 --size 9x6 --square 0.025

  # lưu thêm ảnh đã khử méo để soi đường thẳng
  ./scripts/check_calibration.py --distance 1.0 --save-undistort /tmp/undist.png

Đọc kết quả: sai lệch < 2% là tốt, 2-5% tạm được, > 5% là có gì đó sai —
thường do bàn cờ in bị co (phải đo lại cạnh ô thật rồi truyền vào --square).

Xem thêm: docs/CAMERA.md muc 4.7
"""
import argparse
import os
import sys

import cv2
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from calibrate_camera import find_corners, parse_size, to_gray  # noqa: E402

import rclpy                                                    # noqa: E402
from rclpy.node import Node                                     # noqa: E402
from rclpy.qos import QoSProfile, ReliabilityPolicy             # noqa: E402
from sensor_msgs.msg import CameraInfo, Image                   # noqa: E402


def image_to_gray(msg):
    """Đổi sensor_msgs/Image sang ảnh xám, không cần cv_bridge."""
    buf = np.frombuffer(msg.data, dtype=np.uint8)
    if msg.encoding == "mono8":
        return buf.reshape(msg.height, msg.step)[:, :msg.width]
    if msg.encoding in ("bgr8", "rgb8"):
        img = buf.reshape(msg.height, msg.step // 3, 3)[:, :msg.width]
        return to_gray(img)
    sys.exit(f"[!] Chưa hỗ trợ encoding '{msg.encoding}' (cần mono8 hoặc bgr8/rgb8)")


class Grabber(Node):
    """Hứng /camera_info một lần và gom vài khung /image_raw."""

    def __init__(self, image_topic, info_topic, want):
        super().__init__("check_calibration")
        self.info = None
        self.frames = []
        self.want = want
        # BEST_EFFORT tương thích cả publisher RELIABLE lẫn BEST_EFFORT
        qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(CameraInfo, info_topic, self._on_info, qos)
        self.create_subscription(Image, image_topic, self._on_image, qos)

    def _on_info(self, msg):
        self.info = msg

    def _on_image(self, msg):
        if len(self.frames) < self.want:
            self.frames.append(msg)

    def done(self):
        return self.info is not None and len(self.frames) >= self.want


def collect(args):
    rclpy.init()
    node = Grabber(args.image_topic, args.info_topic, args.frames)
    print(f"[i] Chờ {args.image_topic} và {args.info_topic} ...")
    deadline = node.get_clock().now().nanoseconds + int(args.timeout * 1e9)
    try:
        while rclpy.ok() and not node.done():
            rclpy.spin_once(node, timeout_sec=0.2)
            if node.get_clock().now().nanoseconds > deadline:
                break
        info, frames = node.info, list(node.frames)
    finally:
        node.destroy_node()
        rclpy.shutdown()

    if info is None:
        sys.exit(f"[!] Không nhận được {args.info_topic}. Node camera đã chạy chưa?\n"
                 f"    ./scripts/run_camera_node.sh")
    if not frames:
        sys.exit(f"[!] Không nhận được khung hình nào trên {args.image_topic}.")
    return info, frames


def main():
    p = argparse.ArgumentParser(
        description="Kiểm chứng hiệu chỉnh camera bằng bàn cờ đặt ở khoảng cách đã biết.",
        formatter_class=argparse.RawDescriptionHelpFormatter, epilog=__doc__)
    p.add_argument("--distance", type=float, required=True,
                   help="khoảng cách THẬT từ ống kính tới mặt bàn cờ, tính bằng MÉT")
    p.add_argument("--size", type=parse_size, default=(9, 6),
                   help="số GÓC TRONG dạng NxM (mặc định 9x6)")
    p.add_argument("--square", type=float, default=0.025,
                   help="cạnh một ô, tính bằng mét (mặc định 0.025)")
    p.add_argument("--image-topic", default="/image_raw")
    p.add_argument("--info-topic", default="/camera_info")
    p.add_argument("--frames", type=int, default=5,
                   help="số khung để lấy trung bình (dò góc mất ~2-3 s/khung ở 1280x800)")
    p.add_argument("--timeout", type=float, default=15.0)
    p.add_argument("--save-undistort", default="",
                   help="ghi ảnh gốc|ảnh đã khử méo ra file này để soi bằng mắt")
    a = p.parse_args()

    info, frames = collect(a)
    K = np.array(info.k, dtype=np.float64).reshape(3, 3)
    D = np.array(info.d, dtype=np.float64)
    fx, fy, cx, cy = K[0, 0], K[1, 1], K[0, 2], K[1, 2]

    if abs(K).sum() == 0:
        sys.exit("[!] /camera_info còn rỗng — node chưa nạp được file hiệu chỉnh.")
    if (info.width, info.height) != (frames[0].width, frames[0].height):
        print(f"[!] camera_info {info.width}x{info.height} khác ảnh "
              f"{frames[0].width}x{frames[0].height} — kết quả sẽ SAI.")

    print(f"[i] camera_info: {info.width}x{info.height}  fx={fx:.2f} fy={fy:.2f} "
          f"cx={cx:.2f} cy={cy:.2f}")
    print(f"[i] Bàn cờ {a.size[0]}x{a.size[1]} góc trong, ô {a.square*1000:.1f} mm\n")

    objp = np.zeros((a.size[0] * a.size[1], 3), np.float32)
    objp[:, :2] = np.mgrid[0:a.size[0], 0:a.size[1]].T.reshape(-1, 2)
    objp *= a.square
    center = objp.mean(axis=0)

    zs, ranges, spans, gray = [], [], [], None
    print(f"[i] Dò góc bàn cờ (~2-3 s mỗi khung ở 1280x800, kiên nhẫn chút)...")
    for i, msg in enumerate(frames, 1):
        gray = image_to_gray(msg)
        found, corners = find_corners(gray, a.size)
        if not found:
            print(f"    khung {i}/{len(frames)}: không thấy bàn cờ")
            continue
        ok, rvec, tvec = cv2.solvePnP(objp, corners.reshape(-1, 1, 2).astype(np.float32),
                                      K, D, flags=cv2.SOLVEPNP_ITERATIVE)
        if not ok:
            print(f"    khung {i}/{len(frames)}: solvePnP thất bại")
            continue
        R, _ = cv2.Rodrigues(rvec)
        pc = (R @ center.reshape(3, 1) + tvec).ravel()   # tâm bàn cờ trong hệ camera
        zs.append(float(pc[2]))
        ranges.append(float(np.linalg.norm(pc)))
        c = corners.reshape(-1, 2)
        spans.append(float(c[:, 0].max() - c[:, 0].min()))
        print(f"    khung {i}/{len(frames)}: Z = {pc[2]*100:6.1f} cm")

    if not zs:
        sys.exit(f"[!] Không thấy bàn cờ trong {len(frames)} khung hình.\n"
                 f"    Kiểm tra: --size có đúng số GÓC TRONG không (bàn cờ 10x7 ô -> 9x6)?\n"
                 f"    Ảnh có đủ sáng không, bàn cờ có lọt trọn trong khung không?\n"
                 f"    Xem nhanh:  ros2 run image_view image_view --ros-args -r image:={a.image_topic}")

    z, rng = float(np.mean(zs)), float(np.mean(ranges))
    err = (z - a.distance) / a.distance * 100.0

    print(f"=== KẾT QUẢ ({len(zs)}/{len(frames)} khung thấy bàn cờ) ===")
    print(f"  Đo bằng thước       : {a.distance*100:7.1f} cm")
    print(f"  Script tính (trục Z): {z*100:7.1f} cm  (lệch {err:+.1f}%)")
    print(f"  Khoảng cách thẳng   : {rng*100:7.1f} cm  (nếu bàn cờ lệch khỏi tâm khung)")
    print(f"  Độ tản mát giữa các khung: ±{np.std(zs)*100:.2f} cm")

    # Phép thử tay: bề rộng bàn cờ trên ảnh phải xấp xỉ fx * W / Z
    board_w = (a.size[0] - 1) * a.square
    print(f"\n  Kiểm tra chéo bằng công thức w = fx*W/Z:")
    print(f"    bề rộng bàn cờ {board_w*100:.1f} cm ở {z*100:.1f} cm "
          f"-> lý thuyết {fx*board_w/z:6.1f} px, đo trên ảnh {np.mean(spans):6.1f} px")

    print()
    if abs(err) < 2:
        print("  -> TỐT. Hiệu chỉnh dùng được để đo pose.")
    elif abs(err) < 5:
        print("  -> TẠM ĐƯỢC. Đủ cho bám mục tiêu, hơi thô nếu cần đo khoảng cách chính xác.")
    else:
        print("  -> SAI LỆCH LỚN. Nguyên nhân hay gặp, theo thứ tự:")
        print("     1) --square không đúng: bản in bị co. Đo cạnh MỘT Ô bằng thước rồi truyền số thật.")
        print("     2) Đo thước từ chỗ khác: phải đo từ MẶT KÍNH ống kính tới mặt giấy.")
        print("     3) Hiệu chỉnh ở độ phân giải khác lúc đang chạy.")
        print("     4) Bàn cờ dán không phẳng (cong vênh là hỏng).")

    if a.save_undistort and gray is not None:
        und = cv2.undistort(gray, K, D)
        cv2.imwrite(a.save_undistort, np.hstack([gray, und]))
        print(f"\n[i] Đã ghi {a.save_undistort} (trái = gốc, phải = đã khử méo).")
        print("    Soi mép bàn/khung cửa ở RÌA ảnh: bên phải phải THẲNG. Còn cong là D chưa đúng.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
