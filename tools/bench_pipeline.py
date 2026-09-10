#!/usr/bin/env python3
"""Đo hiệu năng đường ảnh trên companion computer: ROS 2 -> ArUco -> optical flow.

Trả lời đúng một câu hỏi: Pi 4 gánh nổi bao nhiêu khung hình mỗi giây khi vừa
dò marker vừa tính optical flow, và độ trễ từ lúc chụp tới lúc có kết quả là
bao nhiêu.

Node bám vào topic /image_raw đang có sẵn (không mở /dev/video0, nên chạy song
song với node camera được), xử lý khung hình mới nhất và bỏ khung hình đến trong
lúc còn bận — đúng cách một node điều khiển thật phải hành xử.

Dùng:
  ./tools/bench_pipeline.py                          # cả aruco + flow, 20 s
  ./tools/bench_pipeline.py --mode none              # chỉ đo đường truyền ROS
  ./tools/bench_pipeline.py --mode aruco --seconds 30
  ./tools/bench_pipeline.py --scale 0.5              # hạ 1280x800 -> 640x400
  ./tools/bench_pipeline.py --best-effort            # nếu publisher dùng QoS best effort

Đọc kết quả:
  - "nhận" là nhịp khung hình tới, "xử lý" là nhịp thực sự chạy hết pipeline.
    Hai số bằng nhau -> CPU còn dư. Xử lý thấp hơn nhiều -> đang nghẽn.
  - Độ trễ p95 mới là số dùng để thiết kế vòng điều khiển, không phải trung bình.
"""
import argparse
import os
import statistics
import sys
import time

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image

try:
    import psutil
except ImportError:
    psutil = None


def read_temp():
    for p in ("/sys/class/thermal/thermal_zone0/temp",):
        try:
            with open(p) as f:
                return int(f.read().strip()) / 1000.0
        except Exception:
            pass
    return float("nan")


def read_throttled():
    try:
        import subprocess
        out = subprocess.run(["vcgencmd", "get_throttled"], capture_output=True,
                             text=True, timeout=2).stdout.strip()
        return out.split("=")[-1]
    except Exception:
        return "n/a"


def pct(values, q):
    if not values:
        return float("nan")
    s = sorted(values)
    i = min(int(q / 100.0 * len(s)), len(s) - 1)
    return s[i]


def stats_line(name, values, unit="ms"):
    if not values:
        return f"  {name:<22} (không có mẫu)"
    return (f"  {name:<22} p50 {pct(values,50):6.2f}  p95 {pct(values,95):6.2f}  "
            f"max {max(values):6.2f} {unit}")


class Bench(Node):
    def __init__(self, a):
        super().__init__("bench_pipeline")
        self.a = a
        qos = QoSProfile(depth=1, history=HistoryPolicy.KEEP_LAST,
                         reliability=(ReliabilityPolicy.BEST_EFFORT if a.best_effort
                                      else ReliabilityPolicy.RELIABLE))
        self.sub = self.create_subscription(Image, a.topic, self.on_image, qos)

        # OpenCV 4.6 dùng API cũ; 4.7+ đổi sang lớp ArucoDetector.
        self.aruco_new = hasattr(cv2.aruco, "ArucoDetector")
        dict_id = getattr(cv2.aruco, "DICT_" + a.dict.upper())
        if self.aruco_new:
            d = cv2.aruco.getPredefinedDictionary(dict_id)
            self.detector = cv2.aruco.ArucoDetector(d, cv2.aruco.DetectorParameters())
        else:
            self.aruco_dict = cv2.aruco.Dictionary_get(dict_id)
            self.aruco_par = cv2.aruco.DetectorParameters_create()

        self.prev_gray = None
        self.prev_pts = None

        self.t_arrive = []      # mốc thời gian nhận, để tính nhịp tới
        self.t_done = []        # mốc xử lý xong, để tính nhịp xử lý
        self.lat = []           # stamp -> vào callback
        self.d_convert, self.d_aruco, self.d_flow, self.d_total = [], [], [], []
        self.n_marker, self.n_track, self.flow_px = [], [], []
        self.frame_stats = []
        self.busy = False
        self.dropped = 0
        self.skipped_rate = 0
        self.t_last_proc = 0.0
        self.min_period = (1.0 / a.rate) if a.rate > 0 else 0.0
        self.t0 = None
        self.proc = psutil.Process(os.getpid()) if psutil else None
        self.cam_proc = self._find_camera_proc()
        if self.proc:
            self.proc.cpu_percent(None)
        if self.cam_proc:
            self.cam_proc.cpu_percent(None)
        self.sys_cpu = []
        self.timer = self.create_timer(a.seconds, self.finish)
        # lấy mẫu CPU TRONG lúc chạy; đo sau khi xong sẽ ra số vô nghĩa
        if psutil:
            psutil.cpu_percent(None)
            self.cpu_timer = self.create_timer(1.0, self.sample_cpu)

    def sample_cpu(self):
        self.sys_cpu.append(psutil.cpu_percent(None))

    def _find_camera_proc(self):
        if not psutil:
            return None
        for p in psutil.process_iter(["name", "cmdline"]):
            try:
                if p.info["name"] and "v4l2_camera_node" in p.info["name"]:
                    return p
            except Exception:
                pass
        return None

    def on_image(self, msg):
        now = time.monotonic()
        if self.t0 is None:
            self.t0 = now
            self.get_logger().info(
                f"khung đầu tiên: {msg.width}x{msg.height} {msg.encoding} — đang đo "
                f"{self.a.seconds}s (mode={self.a.mode}, scale={self.a.scale})")
        self.t_arrive.append(now)

        if self.busy:                      # mô phỏng node thật: bỏ khung khi còn bận
            self.dropped += 1
            return
        if self.min_period and (now - self.t_last_proc) < self.min_period:
            self.skipped_rate += 1         # giữ nhịp cố định, bỏ phần thừa
            return
        self.t_last_proc = now
        self.busy = True
        try:
            stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            if stamp > 0:
                self.lat.append((self.get_clock().now().nanoseconds * 1e-9 - stamp) * 1e3)

            t = time.perf_counter()
            gray = self.to_gray(msg)
            if self.a.scale != 1.0:
                gray = cv2.resize(gray, None, fx=self.a.scale, fy=self.a.scale,
                                  interpolation=cv2.INTER_AREA)
            t_conv = time.perf_counter()
            self.d_convert.append((t_conv - t) * 1e3)

            if len(self.frame_stats) < 5:
                self.frame_stats.append((float(gray.mean()), float(gray.std())))

            if self.a.mode in ("aruco", "both"):
                self.run_aruco(gray)
            t_ar = time.perf_counter()

            if self.a.mode in ("flow", "both"):
                self.run_flow(gray)
            t_fl = time.perf_counter()

            self.d_aruco.append((t_ar - t_conv) * 1e3)
            self.d_flow.append((t_fl - t_ar) * 1e3)
            self.d_total.append((t_fl - t) * 1e3)
            self.t_done.append(time.monotonic())
        finally:
            self.busy = False

    def to_gray(self, msg):
        buf = np.frombuffer(msg.data, dtype=np.uint8)
        if msg.encoding == "mono8":
            return buf.reshape(msg.height, msg.step)[:, :msg.width]
        if msg.encoding in ("bgr8", "rgb8"):
            img = buf.reshape(msg.height, msg.step)[:, :msg.width * 3]
            return cv2.cvtColor(img.reshape(msg.height, msg.width, 3), cv2.COLOR_BGR2GRAY)
        raise RuntimeError(f"chưa hỗ trợ encoding {msg.encoding}")

    def run_aruco(self, gray):
        if self.aruco_new:
            corners, ids, _ = self.detector.detectMarkers(gray)
        else:
            corners, ids, _ = cv2.aruco.detectMarkers(
                gray, self.aruco_dict, parameters=self.aruco_par)
        self.n_marker.append(0 if ids is None else len(ids))

    def run_flow(self, gray):
        if self.prev_gray is None or self.prev_pts is None or len(self.prev_pts) < 20:
            self.prev_pts = cv2.goodFeaturesToTrack(
                gray, maxCorners=self.a.features, qualityLevel=0.01,
                minDistance=15, blockSize=7)
            self.prev_gray = gray
            self.n_track.append(0 if self.prev_pts is None else len(self.prev_pts))
            return
        nxt, st, _ = cv2.calcOpticalFlowPyrLK(
            self.prev_gray, gray, self.prev_pts, None,
            winSize=(21, 21), maxLevel=3,
            criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 20, 0.03))
        if nxt is not None and st is not None:
            good_new = nxt[st.flatten() == 1]
            good_old = self.prev_pts[st.flatten() == 1]
            self.n_track.append(len(good_new))
            if len(good_new) >= 5:
                d = np.linalg.norm(good_new - good_old, axis=1)
                self.flow_px.append(float(np.median(d)))
            self.prev_pts = good_new.reshape(-1, 1, 2)
        self.prev_gray = gray
        # định kỳ tìm lại đặc trưng, tránh trôi dần hết điểm
        if len(self.t_done) % self.a.redetect == 0:
            self.prev_pts = None

    def finish(self):
        self.timer.cancel()
        if psutil:
            self.cpu_timer.cancel()
        self.report()
        raise SystemExit(0)

    def report(self):
        a = self.a
        span_in = (self.t_arrive[-1] - self.t_arrive[0]) if len(self.t_arrive) > 1 else 0
        span_out = (self.t_done[-1] - self.t_done[0]) if len(self.t_done) > 1 else 0
        hz_in = (len(self.t_arrive) - 1) / span_in if span_in > 0 else 0
        hz_out = (len(self.t_done) - 1) / span_out if span_out > 0 else 0

        print("\n" + "=" * 66)
        print(f"  KẾT QUẢ  mode={a.mode}  scale={a.scale}  topic={a.topic}")
        print("=" * 66)
        if not self.t_arrive:
            print("  Không nhận được khung hình nào.")
            print("  - Node camera còn chạy không?  ros2 topic list")
            print("  - Publisher dùng QoS best effort? thử thêm --best-effort")
            return
        mean_std = self.frame_stats[0] if self.frame_stats else (0, 0)
        print(f"  Khung nhận      {len(self.t_arrive):5d}   -> {hz_in:6.2f} Hz")
        print(f"  Khung xử lý     {len(self.t_done):5d}   -> {hz_out:6.2f} Hz"
              f"   (bỏ vì bận {self.dropped}, bỏ theo --rate {self.skipped_rate})")
        print(f"  Ảnh mẫu         mean={mean_std[0]:.1f} std={mean_std[1]:.1f}"
              f"{'   [!] ẢNH TOÀN ĐEN — xem docs/CAMERA.md 4.6' if mean_std[1] < 2 else ''}")
        print()
        print(stats_line("Trễ chụp->callback", self.lat))
        print(stats_line("Giải mã + resize", self.d_convert))
        if a.mode in ("aruco", "both"):
            print(stats_line("ArUco detect", self.d_aruco))
        if a.mode in ("flow", "both"):
            print(stats_line("Optical flow LK", self.d_flow))
        print(stats_line("Tổng mỗi khung", self.d_total))
        if self.d_total:
            budget = 1000.0 / max(pct(self.d_total, 95), 1e-6)
            print(f"\n  Trần lý thuyết 1 luồng: {budget:6.1f} Hz  (theo p95 tổng)")
        if self.n_marker:
            print(f"  Marker thấy được: trung bình {statistics.mean(self.n_marker):.2f}/khung"
                  f", tối đa {max(self.n_marker)}")
        if self.n_track:
            print(f"  Điểm bám optical flow: trung bình {statistics.mean(self.n_track):.0f}")
        if self.flow_px:
            print(f"  Dịch chuyển trung vị: {statistics.mean(self.flow_px):.2f} px/khung")
        print()
        if self.proc:
            print(f"  CPU node đo này : {self.proc.cpu_percent(None):5.1f} % "
                  f"(1 lõi = 100 %, máy có {psutil.cpu_count()} lõi)")
        if self.cam_proc:
            try:
                print(f"  CPU node camera : {self.cam_proc.cpu_percent(None):5.1f} %")
            except Exception:
                pass
        if self.sys_cpu:
            print(f"  CPU toàn máy    : trung bình {statistics.mean(self.sys_cpu):5.1f} %"
                  f"  đỉnh {max(self.sys_cpu):5.1f} %  (4 lõi gộp = 100 %)")
        print(f"  Nhiệt độ        : {read_temp():.1f} °C   throttled={read_throttled()}")
        print("=" * 66)


def main():
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--topic", default="/image_raw")
    p.add_argument("--mode", default="both", choices=["none", "aruco", "flow", "both"])
    p.add_argument("--seconds", type=float, default=20.0)
    p.add_argument("--scale", type=float, default=1.0, help="hệ số thu nhỏ ảnh trước khi xử lý")
    p.add_argument("--dict", default="4X4_50", help="từ điển ArUco, khớp aruco_opencv")
    p.add_argument("--features", type=int, default=200, help="số điểm đặc trưng cho LK")
    p.add_argument("--redetect", type=int, default=30, help="cứ N khung thì tìm lại đặc trưng")
    p.add_argument("--rate", type=float, default=0.0,
                   help="chỉ xử lý tối đa N khung/giây (0 = hết sức). Dùng để đo tải "
                        "thực của vòng điều khiển chạy ở nhịp cố định")
    p.add_argument("--best-effort", action="store_true", help="QoS best effort")
    a = p.parse_args()

    rclpy.init()
    node = Bench(a)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        try:
            if node.t_arrive and not node.timer.is_canceled():
                node.report()
        except Exception:
            pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
