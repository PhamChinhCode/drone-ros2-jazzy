#!/usr/bin/env python3
"""Sinh tag ArUco / AprilTag sẵn sàng in, đúng kích thước vật lý.

Cùng nguyên tắc với `calibrate_camera.py pattern`: xuất SVG theo mm thật, kèm
thước 100 mm để kiểm tra máy in có co giấy không. Thêm PNG để xem nhanh trên màn hình.

  # một tag ArUco 4x4, id 0, cạnh đen 150 mm, in trên A4
  ./tools/make_tag.py tag --dict 4x4 --id 0 --size 150 --out assets/tags/aruco4x4_id0.svg

  # AprilTag 36h11 — loại apriltag_ros dùng mặc định
  ./tools/make_tag.py tag --dict apriltag36h11 --id 0 --size 150

  # bãi đáp: tag lớn để bắt từ xa + tag nhỏ để giữ lock lúc sát đất
  ./tools/make_tag.py pad --dict apriltag36h11 --size 150 --inner-size 30 --paper a4

QUAN TRỌNG — `--size` là cạnh HÌNH VUÔNG ĐEN BÊN NGOÀI, tính bằng mm. Đó đúng là
con số mà `aruco_opencv` (marker_size) và `apriltag_ros` (size) mong đợi. Đo nhầm
sang vành trắng thì mọi khoảng cách ước lượng sẽ sai theo tỉ lệ.

Xem thêm: docs/CAMERA.md muc 4.7
"""
import argparse
import os
import sys

import cv2
import cv2.aruco as aruco
import numpy as np

DICTS = {
    "4x4":            aruco.DICT_4X4_50,
    "5x5":            aruco.DICT_5X5_50,
    "6x6":            aruco.DICT_6X6_250,
    "7x7":            aruco.DICT_7X7_50,
    "original":       aruco.DICT_ARUCO_ORIGINAL,
    "apriltag16h5":   aruco.DICT_APRILTAG_16h5,
    "apriltag25h9":   aruco.DICT_APRILTAG_25h9,
    "apriltag36h10":  aruco.DICT_APRILTAG_36h10,
    "apriltag36h11":  aruco.DICT_APRILTAG_36h11,
}

PAPERS = {"a4": (297.0, 210.0), "a3": (420.0, 297.0), "letter": (279.4, 215.9)}


def grid_of(dict_name, tag_id):
    """Trả về lưới 0/1 của tag (1 = trắng), đã gồm vành đen 1 ô."""
    d = aruco.getPredefinedDictionary(DICTS[dict_name])
    cells = d.markerSize + 2
    n = int(d.bytesList.shape[0])
    if not 0 <= tag_id < n:
        sys.exit(f"[!] --id {tag_id} ngoài khoảng: từ điển '{dict_name}' chỉ có id 0..{n-1}")
    img = aruco.drawMarker(d, tag_id, cells)
    return (img // 255).astype(int), cells


def svg_tag(parts, grid, cells, x0, y0, size_mm):
    """Vẽ tag vào danh sách phần tử SVG, góc trên-trái tại (x0, y0)."""
    c = size_mm / cells
    for r in range(cells):
        for k in range(cells):
            if grid[r, k] == 0:                      # ô đen
                parts.append(
                    f'<rect x="{x0 + k * c:.4f}" y="{y0 + r * c:.4f}" '
                    f'width="{c:.4f}" height="{c:.4f}" fill="#000000"/>')


def svg_ruler(parts, x, y, note):
    parts.append(f'<line x1="{x}" y1="{y}" x2="{x+100}" y2="{y}" stroke="#000" stroke-width="0.3"/>')
    for t in range(0, 101, 10):
        h = 2.0 if t % 50 == 0 else 1.2
        parts.append(f'<line x1="{x+t}" y1="{y-h}" x2="{x+t}" y2="{y+h}" '
                     f'stroke="#000" stroke-width="0.3"/>')
    parts.append(f'<text x="{x+104}" y="{y+1.6}" font-family="sans-serif" font-size="3.2" '
                 f'fill="#000">vach nay phai dai dung 100 mm sau khi in</text>')
    parts.append(f'<text x="{x}" y="{y-5}" font-family="sans-serif" font-size="3.2" '
                 f'fill="#000">{note}</text>')


def paper_size(a):
    if a.paper not in PAPERS:
        sys.exit(f"[!] --paper phải là một trong: {', '.join(PAPERS)}")
    pw, ph = PAPERS[a.paper]
    return (ph, pw) if a.portrait else (pw, ph)


def write_svg(path, pw, ph, parts):
    os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
    head = [f'<svg xmlns="http://www.w3.org/2000/svg" width="{pw}mm" height="{ph}mm" '
            f'viewBox="0 0 {pw} {ph}">',
            f'<rect x="0" y="0" width="{pw}" height="{ph}" fill="#ffffff"/>']
    with open(path, "w") as f:
        f.write("\n".join(head + parts + ["</svg>"]))
    print(f"[i] Đã ghi {path}")


def write_png(path, grid, cells, px_per_cell=24, quiet_cells=2):
    """PNG để xem trên màn hình (KHÔNG dùng để in — in thì dùng file SVG)."""
    q = quiet_cells * px_per_cell
    img = np.kron(grid.astype(np.uint8) * 255, np.ones((px_per_cell, px_per_cell), np.uint8))
    out = np.full((img.shape[0] + 2 * q, img.shape[1] + 2 * q), 255, np.uint8)
    out[q:q + img.shape[0], q:q + img.shape[1]] = img
    os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
    cv2.imwrite(path, out)
    print(f"[i] Đã ghi {path}  ({out.shape[1]}x{out.shape[0]} px, chỉ để xem)")


def print_notes(dict_name, cells, size_mm, quiet_mm):
    cell = size_mm / cells
    print(f"[i] Từ điển {dict_name}: lưới {cells}x{cells} ô (gồm vành đen 1 ô)")
    print(f"[i] Cạnh đen ngoài {size_mm:.1f} mm  ->  mỗi ô {cell:.2f} mm")
    print(f"[i] Vành TRẮNG quanh tag: {quiet_mm:.1f} mm", end="  ")
    if quiet_mm < size_mm * 0.25:
        print(f"-> HẸP (nên >= {size_mm*0.25:.0f} mm = 25% cạnh tag)")
        print("    Thiếu vành trắng là bộ dò không tách được đường bao -> không thấy tag.")
    else:
        print("-> đủ rộng")
    fx = 683.29
    print(f"[i] Với fx={fx:.0f} px (bản hiệu chỉnh 1280x800 của bạn):")
    for w, label in ((40, "bắt ổn định"), (80, "pose đủ tốt để hạ cánh")):
        print(f"      {label:26s}: tới {fx * size_mm / 1000.0 / w:5.2f} m")


def cmd_tag(a):
    grid, cells = grid_of(a.dict, a.id)
    pw, ph = paper_size(a)
    if a.size > min(pw, ph):
        sys.exit(f"[!] Tag {a.size:.0f} mm không lọt khổ {a.paper.upper()} "
                 f"({pw:.0f}x{ph:.0f} mm). Giảm --size hoặc dùng --paper a3.")
    x0, y0 = (pw - a.size) / 2.0, (ph - a.size) / 2.0
    quiet = min(x0, y0)
    print_notes(a.dict, cells, a.size, quiet)

    parts = []
    svg_tag(parts, grid, cells, x0, y0, a.size)
    ry = ph - 6.0
    if ry > y0 + a.size + 2:
        svg_ruler(parts, 10.0, ry,
                  f"{a.dict} id={a.id} | canh den = {a.size:.0f} mm | "
                  f"marker_size = {a.size/1000.0:.3f} m")
    write_svg(a.out, pw, ph, parts)
    if a.png:
        write_png(a.png, grid, cells)
    print("[i] IN: mở SVG bằng trình duyệt -> Print -> tỉ lệ 100% / 'Actual size'.")
    print("    KHÔNG chọn 'Fit to page'. In xong đo lại thước 100 mm bằng thước thật.")
    return 0


def cmd_pad(a):
    """Bãi đáp: tag lớn bắt từ xa, tag nhỏ giữ lock lúc sát đất.

    KHÔNG lồng tag nhỏ vào giữa tag lớn — làm thế là phá dữ liệu của tag lớn.
    Hai tag đặt cạnh nhau, script in ra khoảng lệch tâm để khai báo tag bundle.
    """
    big, cells_b = grid_of(a.dict, a.id)
    small, cells_s = grid_of(a.dict, a.inner_id)
    pw, ph = paper_size(a)

    gap = max(a.size * 0.25, 10.0)                    # vành trắng giữa hai tag
    total_h = a.size + gap + a.inner_size
    if a.size > pw or total_h > ph:
        sys.exit(f"[!] Bố cục {a.size:.0f} + {gap:.0f} + {a.inner_size:.0f} = {total_h:.0f} mm "
                 f"không lọt khổ {a.paper.upper()} ({pw:.0f}x{ph:.0f} mm). "
                 f"Giảm --size hoặc dùng --paper a3 / --portrait.")

    y_big = (ph - total_h) / 2.0
    x_big = (pw - a.size) / 2.0
    y_small = y_big + a.size + gap
    x_small = (pw - a.inner_size) / 2.0

    print(f"[i] Tag lớn  id={a.id}  cạnh {a.size:.0f} mm")
    print_notes(a.dict, cells_b, a.size, min(x_big, y_big))
    print(f"\n[i] Tag nhỏ  id={a.inner_id}  cạnh {a.inner_size:.0f} mm")
    print_notes(a.dict, cells_s, a.inner_size, gap)

    dy = (y_small + a.inner_size / 2.0) - (y_big + a.size / 2.0)
    print(f"\n[i] Lệch tâm nhỏ so với tâm lớn: x = 0.000 m, y = {dy/1000.0:+.4f} m")
    print("    Dùng số này khi khai báo tag bundle cho apriltag_ros để hai tag")
    print("    cùng cho ra MỘT pose thống nhất, không nhảy khi chuyển tag.")

    parts = []
    svg_tag(parts, big, cells_b, x_big, y_big, a.size)
    svg_tag(parts, small, cells_s, x_small, y_small, a.inner_size)
    ry = ph - 6.0
    if ry > y_small + a.inner_size + 2:
        svg_ruler(parts, 10.0, ry,
                  f"{a.dict} pad | lon id={a.id} {a.size:.0f}mm | "
                  f"nho id={a.inner_id} {a.inner_size:.0f}mm | dy={dy:.1f}mm")
    write_svg(a.out, pw, ph, parts)
    print("[i] IN: tỉ lệ 100% / 'Actual size', KHÔNG 'Fit to page'.")
    return 0


def main():
    p = argparse.ArgumentParser(
        description="Sinh tag ArUco / AprilTag sẵn sàng in.",
        formatter_class=argparse.RawDescriptionHelpFormatter, epilog=__doc__)
    sub = p.add_subparsers(dest="cmd", required=True)

    def common(sp, default_out):
        sp.add_argument("--dict", default="apriltag36h11", choices=sorted(DICTS),
                        help="từ điển tag (mặc định apriltag36h11 — apriltag_ros dùng loại này)")
        sp.add_argument("--id", type=int, default=0, help="id của tag (mặc định 0)")
        sp.add_argument("--size", type=float, default=150.0,
                        help="cạnh HÌNH VUÔNG ĐEN, tính bằng mm (mặc định 150)")
        sp.add_argument("--paper", default="a4", choices=sorted(PAPERS))
        sp.add_argument("--portrait", action="store_true", help="in dọc thay vì ngang")
        sp.add_argument("--out", default=default_out)

    t = sub.add_parser("tag", help="một tag trên một tờ")
    common(t, "tags/tag.svg")
    t.add_argument("--png", default="", help="ghi thêm ảnh PNG để xem trên màn hình")
    t.set_defaults(func=cmd_tag)

    d = sub.add_parser("pad", help="bãi đáp: tag lớn + tag nhỏ trên cùng một tờ")
    common(d, "tags/pad.svg")
    d.add_argument("--inner-id", type=int, default=1, help="id tag nhỏ (mặc định 1)")
    d.add_argument("--inner-size", type=float, default=30.0,
                   help="cạnh đen của tag nhỏ, mm (mặc định 30)")
    d.set_defaults(func=cmd_pad)

    a = p.parse_args()
    return a.func(a)


if __name__ == "__main__":
    sys.exit(main())
