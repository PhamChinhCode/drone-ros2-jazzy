#!/usr/bin/env python3
"""Đọc và hiển thị hình ảnh từ camera OV9281 bằng OpenCV.

Hỗ trợ 3 backend để chạy được trên nhiều máy:
  picamera2  - Raspberry Pi 4/5 (libcamera)          <- mặc định trên Pi
  v4l2       - Jetson Orin Nano, USB cam, /dev/videoN <- mặc định ngoài Pi
  gstreamer  - Jetson (nvarguscamerasrc) hoặc pipeline tuỳ ý

Phím tắt khi cửa sổ đang mở:  q / ESC = thoát,  s = lưu ảnh
"""

import argparse
import os
import sys
import time
from collections import deque

import cv2
import numpy as np

# ---------------------------------------------------------------- backends


class Picamera2Source:
    """Raspberry Pi + libcamera. OV9281 là sensor mono nên ISP trả về ảnh xám."""

    name = "picamera2"

    def __init__(self, args):
        from picamera2 import Picamera2

        self.cam = Picamera2(args.camera)
        cfg = self.cam.create_video_configuration(
            main={"size": (args.width, args.height), "format": "RGB888"},
            buffer_count=4,
        )
        self.cam.configure(cfg)

        controls = {}

        # BẮT BUỘC: ép ISP dùng toàn bộ khung sensor.
        # Nếu không đặt rõ, libcamera lấy giá trị NHỎ NHẤT của ScalerCrop
        # (64x64 ở góc trên-trái) rồi phóng to lên, làm ảnh chỉ còn một góc
        # bị kéo giãn nhoè. rpicam-apps luôn đặt control này nên không dính.
        crop = self.cam.camera_properties.get("ScalerCropMaximum")
        if crop and crop[2] > 0 and crop[3] > 0:
            controls["ScalerCrop"] = crop

        if args.fps:
            # FrameDurationLimits tính bằng micro-giây
            d = int(1_000_000 / args.fps)
            controls["FrameDurationLimits"] = (d, d)

        self.cam.start()
        if controls:
            self.cam.set_controls(controls)
        time.sleep(0.5)  # chờ AE/AGC và crop có hiệu lực

        applied = self.cam.capture_metadata().get("ScalerCrop")
        print(f"[i] Vùng cắt ISP (ScalerCrop): {applied}")

    def read(self):
        return True, self.cam.capture_array()

    def release(self):
        self.cam.stop()
        self.cam.close()


class VideoCaptureSource:
    """Backend chung cho V4L2 (/dev/videoN) và GStreamer."""

    def __init__(self, args, pipeline, api):
        self.name = "gstreamer" if api == cv2.CAP_GSTREAMER else "v4l2"
        self.cap = cv2.VideoCapture(pipeline, api)
        if not self.cap.isOpened():
            raise RuntimeError(f"Không mở được nguồn: {pipeline}")
        if api == cv2.CAP_V4L2:
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, args.width)
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, args.height)
            if args.fps:
                self.cap.set(cv2.CAP_PROP_FPS, args.fps)
            self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)  # giảm độ trễ

    def read(self):
        return self.cap.read()

    def release(self):
        self.cap.release()


def jetson_pipeline(args):
    """Pipeline nvarguscamerasrc cho Jetson (cần driver sensor đã nạp)."""
    return (
        f"nvarguscamerasrc sensor-id={args.camera} ! "
        f"video/x-raw(memory:NVMM),width={args.width},height={args.height},"
        f"framerate={int(args.fps or 60)}/1 ! "
        f"nvvidconv ! video/x-raw,format=BGRx ! "
        f"videoconvert ! video/x-raw,format=BGR ! "
        f"appsink drop=true max-buffers=1 sync=false"
    )


def open_source(args):
    backend = args.backend
    if backend == "auto":
        backend = "picamera2" if is_raspberry_pi() else "v4l2"

    if backend == "picamera2":
        return Picamera2Source(args)
    if backend == "gstreamer":
        pipe = args.pipeline or jetson_pipeline(args)
        return VideoCaptureSource(args, pipe, cv2.CAP_GSTREAMER)
    return VideoCaptureSource(args, args.device, cv2.CAP_V4L2)


def is_raspberry_pi():
    try:
        with open("/proc/device-tree/model", "rb") as f:
            return b"Raspberry Pi" in f.read()
    except OSError:
        return False


# ---------------------------------------------------------------- hiển thị


def setup_display(force_headless):
    """Trả về True nếu hiển thị được cửa sổ.

    Khi chạy qua SSH mà không có X11 forwarding, ta thử đẩy cửa sổ ra
    màn hình vật lý của máy (DISPLAY=:0).
    """
    if force_headless:
        return False
    if not os.environ.get("DISPLAY") and not os.environ.get("WAYLAND_DISPLAY"):
        if os.path.exists("/tmp/.X11-unix/X0"):
            os.environ["DISPLAY"] = ":0"
            print("[i] Không có DISPLAY -> dùng :0 (màn hình gắn trên máy)")
        else:
            print("[i] Không tìm thấy màn hình -> chạy chế độ headless")
            return False
    try:
        cv2.namedWindow("OV9281", cv2.WINDOW_NORMAL)
        return True
    except cv2.error as e:
        print(f"[i] Không mở được cửa sổ ({e.err.strip()}) -> headless")
        return False


def draw_hud(frame, fps, count):
    text = f"{frame.shape[1]}x{frame.shape[0]}  {fps:5.1f} FPS  #{count}"
    cv2.putText(frame, text, (10, 25), cv2.FONT_HERSHEY_SIMPLEX,
                0.7, (0, 0, 0), 4, cv2.LINE_AA)
    cv2.putText(frame, text, (10, 25), cv2.FONT_HERSHEY_SIMPLEX,
                0.7, (0, 255, 0), 1, cv2.LINE_AA)
    return frame


# ---------------------------------------------------------------- main


def main():
    p = argparse.ArgumentParser(description="Xem ảnh camera OV9281 bằng OpenCV")
    p.add_argument("--backend", default="auto",
                   choices=["auto", "picamera2", "v4l2", "gstreamer"])
    p.add_argument("--width", type=int, default=1280)
    p.add_argument("--height", type=int, default=800)
    p.add_argument("--fps", type=float, default=0, help="0 = để driver tự chọn")
    p.add_argument("--camera", type=int, default=0, help="chỉ số camera / sensor-id")
    p.add_argument("--device", default="/dev/video0", help="cho backend v4l2")
    p.add_argument("--pipeline", default="", help="pipeline GStreamer tuỳ ý")
    p.add_argument("--headless", action="store_true", help="không mở cửa sổ")
    p.add_argument("--save-dir", default="captures", help="thư mục lưu ảnh khi bấm 's'")
    p.add_argument("--frames", type=int, default=0, help="dừng sau N khung (0 = vô hạn)")
    args = p.parse_args()

    src = open_source(args)
    print(f"[+] Backend: {src.name}  |  yêu cầu {args.width}x{args.height}"
          f"{f' @ {args.fps:g}fps' if args.fps else ''}")

    show = setup_display(args.headless)
    os.makedirs(args.save_dir, exist_ok=True)

    times = deque(maxlen=30)
    count = saved = 0
    last_log = time.monotonic()

    try:
        while True:
            ok, frame = src.read()
            if not ok or frame is None:
                print("[!] Không đọc được khung hình", file=sys.stderr)
                break

            count += 1
            times.append(time.monotonic())
            fps = (len(times) - 1) / (times[-1] - times[0]) if len(times) > 1 else 0.0

            # Sensor mono có thể trả về ảnh 1 kênh -> đổi sang BGR để vẽ HUD màu
            if frame.ndim == 2:
                frame = cv2.cvtColor(frame, cv2.COLOR_GRAY2BGR)

            if show:
                cv2.imshow("OV9281", draw_hud(frame.copy(), fps, count))
                key = cv2.waitKey(1) & 0xFF
                if key in (ord("q"), 27):
                    break
                if key == ord("s"):
                    path = os.path.join(args.save_dir, f"frame_{saved:04d}.png")
                    cv2.imwrite(path, frame)
                    saved += 1
                    print(f"[+] Đã lưu {path}")
            else:
                now = time.monotonic()
                if now - last_log >= 1.0:
                    mean = float(np.mean(frame))
                    print(f"    khung {count:6d}  {fps:5.1f} FPS  độ sáng TB {mean:5.1f}")
                    last_log = now

            if args.frames and count >= args.frames:
                break
    except KeyboardInterrupt:
        print("\n[i] Dừng bởi người dùng")
    finally:
        src.release()
        cv2.destroyAllWindows()
        print(f"[i] Tổng cộng {count} khung hình")


if __name__ == "__main__":
    main()
