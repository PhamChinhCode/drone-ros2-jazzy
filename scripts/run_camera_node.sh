#!/usr/bin/env bash
# Cấu hình V4L2 rồi khởi động node ROS 2 xuất ảnh OV9281 ra topic /image_raw.
#
# Gộp 2 bước bắt buộc làm một (cấu hình V4L2 reset sau mỗi lần reboot):
#   1) scripts/camera_v4l2_setup.sh  — ép subdev sang Y8_1X8, đặt vblank/exposure/gain
#   2) ros2 run v4l2_camera v4l2_camera_node ...
# Bỏ bước 1 thì node vẫn chạy và 'ros2 topic hz' vẫn báo nhịp bình thường,
# nhưng MỌI KHUNG HÌNH TOÀN SỐ 0 (subdev còn ở Y10 trong khi node xin GREY 8-bit).
# Giải thích đầy đủ: docs/CAMERA.md mục 4.6
#
# Dùng:
#   ./scripts/run_camera_node.sh                                # 1280x800, exposure 1200, gain 100
#   ./scripts/run_camera_node.sh --exposure 2000 --gain 150
#   ./scripts/run_camera_node.sh --width 640 --height 400       # nhẹ hơn, FPS cao hơn
#   ./scripts/run_camera_node.sh --vblank 110                   # FPS tối đa (mặc định driver bóp còn ~23 Hz)
#   ./scripts/run_camera_node.sh --setup-only                   # chỉ cấu hình, không chạy node
#   ./scripts/run_camera_node.sh -- -p camera_frame_id:=cam     # tham số thừa đẩy thẳng vào node
#
# Kiểm tra ở terminal khác:
#   ros2 topic hz /image_raw
#   ros2 run image_view image_view --ros-args -r image:=/image_raw
set -uo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

WIDTH=1280
HEIGHT=800
EXPOSURE=1200
GAIN=100
VBLANK=""
BITS=8
SETUP_ONLY=0
EXTRA_ARGS=()

while [[ $# -gt 0 ]]; do
  case "$1" in
    --width)      WIDTH="${2:?thiếu giá trị cho --width}"; shift 2 ;;
    --height)     HEIGHT="${2:?thiếu giá trị cho --height}"; shift 2 ;;
    --exposure)   EXPOSURE="${2:?thiếu giá trị cho --exposure}"; shift 2 ;;
    --gain)       GAIN="${2:?thiếu giá trị cho --gain}"; shift 2 ;;
    --vblank)     VBLANK="${2:?thiếu giá trị cho --vblank}"; shift 2 ;;
    --bits)       BITS="${2:?thiếu giá trị cho --bits}"; shift 2 ;;
    --setup-only) SETUP_ONLY=1; shift ;;
    --)           shift; EXTRA_ARGS=("$@"); break ;;
    -h|--help)    sed -n '2,25p' "$0"; exit 0 ;;
    *) echo "[!] Tham số không nhận diện được: $1 (dùng -h để xem trợ giúp)" >&2; exit 1 ;;
  esac
done

case "$BITS" in
  8)  PIXFMT="GREY"; ENCODING="mono8" ;;
  10) PIXFMT="Y10P"; ENCODING="mono16"
      echo "[!] --bits 10: v4l2_camera có thể không hiểu Y10P. Mặc định 8-bit đã kiểm chứng." ;;
  *)  echo "[!] --bits chỉ nhận 8 hoặc 10." >&2; exit 1 ;;
esac

# ---------------------------------------------------------------------------
# Bước 1: cấu hình đường V4L2
# ---------------------------------------------------------------------------
SETUP="$SCRIPT_DIR/camera_v4l2_setup.sh"
[[ -x "$SETUP" ]] || { echo "[!] Không thấy $SETUP (hoặc chưa có quyền thực thi)."; exit 1; }

SETUP_ARGS=(--width "$WIDTH" --height "$HEIGHT" --bits "$BITS"
            --exposure "$EXPOSURE" --gain "$GAIN" --quiet)
[[ -n "$VBLANK" ]] && SETUP_ARGS+=(--vblank "$VBLANK")

"$SETUP" "${SETUP_ARGS[@]}" \
  || { echo "[!] Cấu hình V4L2 thất bại — dừng, không khởi động node."; exit 1; }

if [[ "$SETUP_ONLY" -eq 1 ]]; then
  echo "[i] --setup-only: bỏ qua bước khởi động node."
  exit 0
fi

# ---------------------------------------------------------------------------
# Bước 2: khởi động node ROS 2
# ---------------------------------------------------------------------------
if ! command -v ros2 >/dev/null 2>&1; then
  # Chưa source ROS — thử tự tìm bản đã cài
  for d in /opt/ros/*/setup.bash; do
    [[ -r "$d" ]] || continue
    echo "[i] Chưa source ROS, tự nạp: $d"
    # shellcheck disable=SC1090
    . "$d"
    break
  done
fi
command -v ros2 >/dev/null 2>&1 \
  || { echo "[!] Không tìm thấy lệnh ros2. Chạy: source /opt/ros/<distro>/setup.bash"; exit 1; }

if ! ros2 pkg prefix v4l2_camera >/dev/null 2>&1; then
  echo "[!] Chưa cài gói v4l2_camera. Chạy:"
  echo "    sudo apt install -y ros-\${ROS_DISTRO:-jazzy}-v4l2-camera ros-\${ROS_DISTRO:-jazzy}-image-transport-plugins"
  exit 1
fi

VIDEO_DEV="$(v4l2-ctl --list-devices 2>/dev/null \
             | awk '/unicam|rp1-cfe/{f=1;next} f&&/\/dev\/video/{gsub(/^[ \t]+/,"");print;exit}')"
VIDEO_DEV="${VIDEO_DEV:-/dev/video0}"

echo "[i] Khởi động v4l2_camera_node: ${VIDEO_DEV} ${PIXFMT} ${WIDTH}x${HEIGHT} -> /image_raw (${ENCODING})"
echo "[i] Ctrl+C để dừng. Kiểm tra ở terminal khác: ros2 topic hz /image_raw"
echo

# exec để Ctrl+C tác động thẳng vào node, không kẹt ở lớp shell bọc ngoài
exec ros2 run v4l2_camera v4l2_camera_node --ros-args \
  -p video_device:="$VIDEO_DEV" \
  -p pixel_format:="$PIXFMT" \
  -p output_encoding:="$ENCODING" \
  -p image_size:="[${WIDTH},${HEIGHT}]" \
  "${EXTRA_ARGS[@]}"
