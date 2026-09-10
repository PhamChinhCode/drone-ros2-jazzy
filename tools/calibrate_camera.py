#!/usr/bin/env python3
"""Hiệu chỉnh camera OV9281 KHÔNG CẦN GUI — dùng được trên Ubuntu Server thuần SSH.

`ros2 run camera_calibration cameracalibrator` bắt buộc mở cửa sổ đồ hoạ nên không
chạy được trên máy không có desktop. Script này thay thế: chụp ảnh bàn cờ rồi tính
tham số bằng OpenCV, toàn bộ phản hồi in ra terminal.

Quy trình 2 bước:

  # 1) Chụp — cầm bàn cờ đi khắp khung hình, script tự lọc và lưu ảnh hợp lệ
  ./scripts/calibrate_camera.py capture --size 9x6 --square 0.025

  # 2) Tính + ghi file camera_info cho ROS
  ./scripts/calibrate_camera.py solve --size 9x6 --square 0.025 --write-ros

LƯU Ý: bước `capture` đọc trực tiếp /dev/video0 nên phải DỪNG node camera trước
(Ctrl+C ở terminal chạy run_camera_node.sh), vì thiết bị chỉ cho một tiến trình mở.

--size đếm SỐ GÓC BÊN TRONG, không đếm ô: bàn cờ 10x7 ô  ->  --size 9x6
--square là cạnh một ô tính bằng MÉT (đo bằng thước, ví dụ 25 mm -> 0.025)

Xem thêm: docs/CAMERA.md muc 4.7
"""
import argparse
import os
import sys
import glob

import cv2
import numpy as np

DEFAULT_DIR = os.path.expanduser("~/.ros/camera_calib_images")
DEFAULT_ROS_YAML = os.path.expanduser("~/.ros/camera_info/unicam.yaml")


def parse_size(s):
    try:
        w, h = s.lower().split("x")
        return int(w), int(h)
    except Exception:
        raise argparse.ArgumentTypeError("--size phải dạng NxM, ví dụ 9x6")


def open_camera(device, width, height):
    """Mở /dev/videoN ở chế độ mono8.

    Bắt buộc ép fourcc GREY và tắt CONVERT_RGB: nếu để OpenCV tự thương lượng,
    nó xin YUYV -> lệch với subdev đang ở Y8_1X8 -> mọi khung hình toàn số 0.
    Đây đúng là cái bẫy mô tả ở docs/CAMERA.md muc 4.6.
    """
    cap = cv2.VideoCapture(device, cv2.CAP_V4L2)
    if not cap.isOpened():
        sys.exit(f"[!] Không mở được {device}. Node camera còn đang chạy? "
                 f"Dừng nó rồi thử lại.")
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"GREY"))
    cap.set(cv2.CAP_PROP_CONVERT_RGB, 0)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
    return cap


def to_gray(frame):
    if frame is None:
        return None
    if frame.ndim == 2:
        return frame
    return frame[:, :, 0] if frame.shape[2] == 1 else cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)


def find_corners(gray, pattern):
    """Tìm góc bàn cờ. Ưu tiên findChessboardCornersSB (nhanh và khoẻ hơn)."""
    if hasattr(cv2, "findChessboardCornersSB"):
        ok, corners = cv2.findChessboardCornersSB(
            gray, pattern, flags=cv2.CALIB_CB_EXHAUSTIVE | cv2.CALIB_CB_ACCURACY)
        if ok:
            return True, corners
    flags = (cv2.CALIB_CB_ADAPTIVE_THRESH | cv2.CALIB_CB_NORMALIZE_IMAGE |
             cv2.CALIB_CB_FAST_CHECK)
    ok, corners = cv2.findChessboardCorners(gray, pattern, flags)
    if ok:
        corners = cv2.cornerSubPix(
            gray, corners, (11, 11), (-1, -1),
            (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001))
    return ok, corners


def coverage_grid(centers, w, h, cells=6):
    """Vẽ lưới ASCII cho biết bàn cờ đã phủ những vùng nào của khung hình."""
    grid = np.zeros((cells, cells), dtype=int)
    for cx, cy in centers:
        gx = min(int(cx / w * cells), cells - 1)
        gy = min(int(cy / h * cells), cells - 1)
        grid[gy, gx] += 1
    lines = []
    for row in grid:
        lines.append("    " + " ".join("#" if v else "." for v in row))
    filled = int((grid > 0).sum())
    lines.append(f"    phủ {filled}/{cells * cells} ô lưới")
    return "\n".join(lines)


PAPERS = {          # khổ giấy, mm, dạng (rộng, cao) khi để NGANG
    "a4": (297.0, 210.0),
    "a3": (420.0, 297.0),
    "letter": (279.4, 215.9),
}


def cmd_pattern(a):
    """Sinh file SVG bàn cờ đúng kích thước vật lý, sẵn sàng đem in."""
    nx, ny = a.size                      # số góc TRONG
    sx, sy = nx + 1, ny + 1              # số Ô
    sq = a.square * 1000.0               # mm
    bw, bh = sx * sq, sy * sq            # kích thước bàn cờ, mm

    if a.paper not in PAPERS:
        sys.exit(f"[!] --paper phải là một trong: {', '.join(PAPERS)}")
    pw, ph = PAPERS[a.paper]
    if a.portrait:
        pw, ph = ph, pw

    mx, my = (pw - bw) / 2.0, (ph - bh) / 2.0
    print(f"[i] Bàn cờ {sx}x{sy} ô, ô {sq:.1f} mm  ->  {bw:.1f} x {bh:.1f} mm")
    print(f"[i] Giấy {a.paper.upper()} {'dọc' if a.portrait else 'ngang'}: {pw:.1f} x {ph:.1f} mm")
    print(f"[i] Số góc TRONG = {nx}x{ny}  ->  dùng --size {nx}x{ny} khi capture/solve")

    if mx < 0 or my < 0:
        sys.exit(f"[!] Bàn cờ ({bw:.0f}x{bh:.0f} mm) LỚN HƠN khổ giấy. "
                 f"Giảm --square hoặc --size, hoặc dùng --paper a3.")
    print(f"[i] Lề trắng còn lại: {mx:.1f} mm ngang, {my:.1f} mm dọc", end="  ")
    if min(mx, my) < 12:
        print("-> HƠI HẸP")
        print("    Cần vành trắng quanh bàn cờ thì OpenCV mới dò được góc.")
        print("    Nên giảm --square, hoặc in khổ lớn hơn (--paper a3).")
    else:
        print("-> đủ rộng")

    parts = [
        f'<svg xmlns="http://www.w3.org/2000/svg" width="{pw}mm" height="{ph}mm" '
        f'viewBox="0 0 {pw} {ph}">',
        f'<rect x="0" y="0" width="{pw}" height="{ph}" fill="#ffffff"/>',
    ]
    # Ô đen: (hàng+cột) chẵn -> đen, để góc trên-trái là ô đen
    for r in range(sy):
        for c in range(sx):
            if (r + c) % 2 == 0:
                x, y = mx + c * sq, my + r * sq
                parts.append(f'<rect x="{x:.4f}" y="{y:.4f}" width="{sq:.4f}" '
                             f'height="{sq:.4f}" fill="#000000"/>')

    # Thước kiểm tra tỉ lệ in, đặt ở lề dưới
    ry = ph - 6.0
    rx = 10.0
    if ry > my + bh + 2:
        parts.append(f'<line x1="{rx}" y1="{ry}" x2="{rx+100}" y2="{ry}" '
                     f'stroke="#000" stroke-width="0.3"/>')
        for t in range(0, 101, 10):
            h = 2.0 if t % 50 == 0 else 1.2
            parts.append(f'<line x1="{rx+t}" y1="{ry-h}" x2="{rx+t}" y2="{ry+h}" '
                         f'stroke="#000" stroke-width="0.3"/>')
        parts.append(f'<text x="{rx+104}" y="{ry+1.6}" font-family="sans-serif" '
                     f'font-size="3.2" fill="#000">'
                     f'vach nay phai dai dung 100 mm sau khi in</text>')
        parts.append(f'<text x="{rx}" y="{ry-5}" font-family="sans-serif" '
                     f'font-size="3.2" fill="#000">'
                     f'chessboard {sx}x{sy} o | o = {sq:.1f} mm | '
                     f'goc trong = {nx}x{ny} | --size {nx}x{ny} --square {a.square}</text>')
    parts.append("</svg>")

    with open(a.out, "w") as f:
        f.write("\n".join(parts))
    print(f"\n[i] Đã ghi {a.out}")
    print("[i] IN: mở bằng trình duyệt -> Print -> đặt tỉ lệ 100% / 'Actual size',")
    print("    TUYỆT ĐỐI KHÔNG chọn 'Fit to page' (sẽ co ảnh và sai kích thước ô).")
    print("[i] In xong ĐO LẠI thước 100 mm bằng thước thật. Nếu lệch, đo cạnh một ô")
    print(f"    rồi truyền số đo thật vào --square (mét) thay vì {a.square}.")
    return 0


def cmd_capture(a):
    pattern = a.size
    need = a.count
    os.makedirs(a.outdir, exist_ok=True)
    for f in glob.glob(os.path.join(a.outdir, "calib_*.png")):
        os.remove(f)

    cap = open_camera(a.device, a.width, a.height)
    print(f"[i] Chụp vào {a.outdir}")
    print(f"[i] Bàn cờ {pattern[0]}x{pattern[1]} góc trong, ô {a.square*1000:.0f} mm")
    print(f"[i] Cần {need} ảnh hợp lệ. Ctrl+C để dừng sớm.\n")
    print("    Cầm bàn cờ đi KHẮP khung hình, đặc biệt là 4 GÓC (ống kính fisheye")
    print("    méo mạnh ở rìa), nghiêng trái/phải/trên/dưới, cả gần lẫn xa.\n")

    saved, centers = 0, []
    tried = blank = 0
    try:
        while saved < need:
            ok, frame = cap.read()
            tried += 1
            gray = to_gray(frame)
            if gray is None:
                continue
            if gray.max() == 0:
                blank += 1
                if blank == 30:
                    print("[!] Khung hình toàn số 0 — subdev chưa khớp format.")
                    print("    Chạy: ./scripts/camera_v4l2_setup.sh --vblank 110 "
                          "--exposure 800 --gain 120")
                    break
                continue

            found, corners = find_corners(gray, pattern)
            if not found:
                continue

            c = corners.reshape(-1, 2)
            cx, cy = float(c[:, 0].mean()), float(c[:, 1].mean())
            span = float(np.linalg.norm(c.max(axis=0) - c.min(axis=0)))

            # Chỉ nhận ảnh đủ KHÁC các ảnh đã lưu, để bộ ảnh có độ đa dạng
            too_close = any(
                abs(cx - px) < a.min_move and abs(cy - py) < a.min_move
                for px, py in centers)
            if too_close:
                continue

            path = os.path.join(a.outdir, f"calib_{saved:03d}.png")
            cv2.imwrite(path, gray)
            centers.append((cx, cy))
            saved += 1
            print(f"[{saved:2d}/{need}] tâm=({cx:6.1f},{cy:6.1f}) span={span:5.1f}px  -> {os.path.basename(path)}")
            print(coverage_grid(centers, a.width, a.height))
    except KeyboardInterrupt:
        print("\n[i] Dừng theo yêu cầu.")
    finally:
        cap.release()

    print(f"\n[i] Đã lưu {saved} ảnh (đọc {tried} khung).")
    if saved < 10:
        print("[!] Dưới 10 ảnh thì kết quả hiệu chỉnh sẽ kém. Nên chụp lại nhiều hơn.")
    else:
        print(f"[i] Bước tiếp theo:\n    {sys.argv[0]} solve "
              f"--size {pattern[0]}x{pattern[1]} --square {a.square} --write-ros")
    return 0 if saved else 1


def cmd_solve(a):
    pattern = a.size
    files = sorted(glob.glob(os.path.join(a.outdir, "calib_*.png")))
    if not files:
        sys.exit(f"[!] Không có ảnh nào trong {a.outdir}. Chạy lệnh 'capture' trước.")

    objp = np.zeros((pattern[0] * pattern[1], 3), np.float32)
    objp[:, :2] = np.mgrid[0:pattern[0], 0:pattern[1]].T.reshape(-1, 2)
    objp *= a.square

    objpoints, imgpoints, used = [], [], []
    shape = None
    print(f"[i] Đọc {len(files)} ảnh từ {a.outdir}")
    for f in files:
        gray = cv2.imread(f, cv2.IMREAD_GRAYSCALE)
        if gray is None:
            continue
        shape = gray.shape[::-1]
        found, corners = find_corners(gray, pattern)
        if found:
            objpoints.append(objp)
            imgpoints.append(corners.reshape(-1, 1, 2).astype(np.float32))
            used.append(f)
        print(f"    {'OK ' if found else 'BỎ'} {os.path.basename(f)}")

    if len(objpoints) < 5:
        sys.exit(f"[!] Chỉ {len(objpoints)} ảnh dùng được — quá ít. Chụp thêm.")

    print(f"\n[i] Dùng {len(objpoints)}/{len(files)} ảnh, độ phân giải {shape[0]}x{shape[1]}")

    flags = 0
    if a.k_coefficients <= 2:
        flags |= cv2.CALIB_FIX_K3
    if a.fix_principal_point:
        flags |= cv2.CALIB_FIX_PRINCIPAL_POINT

    rms, K, D, rvecs, tvecs = cv2.calibrateCamera(
        objpoints, imgpoints, shape, None, None, flags=flags)

    # Sai số tái chiếu trung bình từng ảnh — để chỉ ra tấm nào tệ
    per_view = []
    for i in range(len(objpoints)):
        proj, _ = cv2.projectPoints(objpoints[i], rvecs[i], tvecs[i], K, D)
        err = cv2.norm(imgpoints[i], proj, cv2.NORM_L2) / len(proj)
        per_view.append((err, os.path.basename(used[i])))

    print(f"\n=== KẾT QUẢ ===")
    print(f"  Sai số tái chiếu (RMS): {rms:.4f} px", end="  ")
    if rms < 0.5:
        print("-> TỐT")
    elif rms < 1.0:
        print("-> tạm được")
    else:
        print("-> KÉM, nên chụp lại (bàn cờ phải phẳng, đủ sáng, phủ kín khung)")

    fx, fy, cx, cy = K[0, 0], K[1, 1], K[0, 2], K[1, 2]
    print(f"  fx={fx:8.2f}  fy={fy:8.2f}")
    print(f"  cx={cx:8.2f}  cy={cy:8.2f}   (tâm ảnh danh nghĩa: {shape[0]/2:.1f}, {shape[1]/2:.1f})")
    print(f"  D = {np.round(D.ravel(), 5).tolist()}")

    fov_x = 2 * np.degrees(np.arctan(shape[0] / (2 * fx)))
    fov_y = 2 * np.degrees(np.arctan(shape[1] / (2 * fy)))
    print(f"  Góc nhìn ước lượng: {fov_x:.1f}° ngang x {fov_y:.1f}° dọc")

    worst = sorted(per_view, reverse=True)[:3]
    print(f"  3 ảnh sai số cao nhất: " + ", ".join(f"{n}({e:.3f})" for e, n in worst))

    print(f"\n  Kiểm chứng nhanh: marker cạnh {a.check_marker*100:.0f} cm ở cách 1 m sẽ rộng "
          f"~{fx * a.check_marker / 1.0:.0f} px trên ảnh.")

    if a.write_ros:
        write_ros_yaml(a.ros_yaml, a.camera_name, shape, K, D)
    else:
        print(f"\n[i] Thêm --write-ros để ghi thẳng ra {a.ros_yaml}")
    return 0


def write_ros_yaml(path, name, shape, K, D):
    d = list(np.array(D).ravel()[:5])
    while len(d) < 5:
        d.append(0.0)
    P = np.zeros((3, 4))
    P[:3, :3] = K
    os.makedirs(os.path.dirname(path), exist_ok=True)
    if os.path.exists(path):
        bak = path + ".bak"
        os.replace(path, bak)
        print(f"[i] File cũ được đổi tên thành {bak}")

    def rows(m):
        return ", ".join(f"{v:.10g}" for v in np.array(m).ravel())

    with open(path, "w") as f:
        f.write(f"""image_width: {shape[0]}
image_height: {shape[1]}
camera_name: {name}
camera_matrix:
  rows: 3
  cols: 3
  data: [{rows(K)}]
distortion_model: plumb_bob
distortion_coefficients:
  rows: 1
  cols: 5
  data: [{rows(d)}]
rectification_matrix:
  rows: 3
  cols: 3
  data: [1, 0, 0, 0, 1, 0, 0, 0, 1]
projection_matrix:
  rows: 3
  cols: 4
  data: [{rows(P)}]
""")
    print(f"\n[i] Đã ghi {path}")
    print("[i] Khởi động lại node camera — cảnh báo 'Unable to open camera calibration")
    print("    file' sẽ biến mất và /camera_info sẽ có số thật.")


def main():
    p = argparse.ArgumentParser(
        description="Hiệu chỉnh camera không cần GUI (dùng trên Ubuntu Server).",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__)
    sub = p.add_subparsers(dest="cmd", required=True)

    def common(sp):
        sp.add_argument("--size", type=parse_size, default=(9, 6),
                        help="số GÓC TRONG dạng NxM (mặc định 9x6, tức bàn cờ 10x7 ô)")
        sp.add_argument("--square", type=float, default=0.025,
                        help="cạnh một ô, tính bằng mét (mặc định 0.025)")
        sp.add_argument("--outdir", default=DEFAULT_DIR)

    g = sub.add_parser("pattern", help="sinh file SVG bàn cờ để in")
    common(g)
    g.add_argument("--paper", default="a4", choices=sorted(PAPERS),
                   help="khổ giấy (mặc định a4)")
    g.add_argument("--portrait", action="store_true", help="in dọc thay vì ngang")
    g.add_argument("--out", default="chessboard.svg")
    g.set_defaults(func=cmd_pattern)

    c = sub.add_parser("capture", help="chụp bộ ảnh bàn cờ")
    common(c)
    c.add_argument("--device", default="/dev/video0")
    c.add_argument("--width", type=int, default=1280)
    c.add_argument("--height", type=int, default=800)
    c.add_argument("--count", type=int, default=25, help="số ảnh cần (mặc định 25)")
    c.add_argument("--min-move", type=float, default=60.0,
                   help="khoảng cách tối thiểu giữa tâm 2 ảnh liên tiếp, px")
    c.set_defaults(func=cmd_capture)

    s = sub.add_parser("solve", help="tính tham số từ bộ ảnh đã chụp")
    common(s)
    s.add_argument("--k-coefficients", type=int, default=3,
                   help="số hệ số méo radial (2 hoặc 3, mặc định 3)")
    s.add_argument("--fix-principal-point", action="store_true")
    s.add_argument("--write-ros", action="store_true",
                   help="ghi thẳng ra file camera_info cho ROS")
    s.add_argument("--ros-yaml", default=DEFAULT_ROS_YAML)
    s.add_argument("--camera-name", default="unicam")
    s.add_argument("--check-marker", type=float, default=0.10,
                   help="cạnh marker (m) dùng cho dòng kiểm chứng cuối")
    s.set_defaults(func=cmd_solve)

    a = p.parse_args()
    # Tiến trình phải hiện ngay cả khi output bị pipe (tee vào file log chẳng hạn)
    try:
        sys.stdout.reconfigure(line_buffering=True)
    except Exception:
        pass
    return a.func(a)


if __name__ == "__main__":
    sys.exit(main())
