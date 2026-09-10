#!/usr/bin/env bash
# Cấu hình đường V4L2 trực tiếp cho OV9281 (bỏ qua libcamera/ISP/IPA).
#
# Vì sao cần script này:
#   1) bcm2835-unicam KHÔNG đổi độ sâu bit. Sensor mặc định ở Y10_1X10 (10-bit),
#      nên nếu ứng dụng xin 'GREY' (8-bit) thì khung hình ra TOÀN SỐ 0.
#      Phải ép subdev của sensor sang Y8_1X8 cho khớp.
#   2) Đọc thẳng V4L2 thì KHÔNG có IPA làm auto-exposure. Sensor giữ nguyên
#      exposure/gain lần trước → thường rất tối. Phải đặt tay.
#   3) FPS bị chặn bởi vertical_blanking (mặc định của driver rất lớn).
#      Hạ --vblank để chạy nhanh hơn.
#   4) Mọi thứ trên RESET sau mỗi lần reboot / nạp lại overlay.
#
# Dùng:
#   ./scripts/camera_v4l2_setup.sh                          # 1280x800, 8-bit
#   ./scripts/camera_v4l2_setup.sh --width 640 --height 400
#   ./scripts/camera_v4l2_setup.sh --exposure 3000 --gain 200
#   ./scripts/camera_v4l2_setup.sh --vblank 110             # FPS tối đa
#   ./scripts/camera_v4l2_setup.sh --bits 10                # giữ Y10 (cần app đọc Y10P)
#
# Sau khi chạy script này:
#   ros2 run v4l2_camera v4l2_camera_node --ros-args \
#     -p video_device:=/dev/video0 -p pixel_format:=GREY \
#     -p output_encoding:=mono8 -p image_size:="[1280,800]"
set -uo pipefail

WIDTH=1280
HEIGHT=800
BITS=8
EXPOSURE=""
GAIN=""
VBLANK=""
QUIET=0

while [[ $# -gt 0 ]]; do
  case "$1" in
    --width)    WIDTH="${2:?thiếu giá trị cho --width}"; shift 2 ;;
    --height)   HEIGHT="${2:?thiếu giá trị cho --height}"; shift 2 ;;
    --bits)     BITS="${2:?thiếu giá trị cho --bits}"; shift 2 ;;
    --exposure) EXPOSURE="${2:?thiếu giá trị cho --exposure}"; shift 2 ;;
    --gain)     GAIN="${2:?thiếu giá trị cho --gain}"; shift 2 ;;
    --vblank)   VBLANK="${2:?thiếu giá trị cho --vblank}"; shift 2 ;;
    --quiet)    QUIET=1; shift ;;   # bỏ phần gợi ý cuối (dùng khi gọi từ script khác)
    -h|--help)  sed -n '2,28p' "$0"; exit 0 ;;
    *) echo "[!] Tham số không nhận diện được: $1" >&2; exit 1 ;;
  esac
done

case "$BITS" in
  8)  MBUS="Y8_1X8";   PIXFMT="GREY" ;;
  10) MBUS="Y10_1X10"; PIXFMT="Y10P" ;;
  *)  echo "[!] --bits chỉ nhận 8 hoặc 10." >&2; exit 1 ;;
esac

command -v media-ctl >/dev/null || { echo "[!] thiếu media-ctl — sudo apt install v4l-utils"; exit 1; }
command -v v4l2-ctl  >/dev/null || { echo "[!] thiếu v4l2-ctl — sudo apt install v4l-utils"; exit 1; }

# ---------------------------------------------------------------------------
# Tìm media device của tầng thu (unicam trên Pi 4, rp1-cfe trên Pi 5)
# ---------------------------------------------------------------------------
MEDIA_DEV=""
for m in /dev/media*; do
  [[ -e "$m" ]] || continue
  if media-ctl -d "$m" -p 2>/dev/null | grep -qE '^driver[[:space:]]+(unicam|rp1-cfe)'; then
    MEDIA_DEV="$m"; break
  fi
done
[[ -n "$MEDIA_DEV" ]] || { echo "[!] Không tìm thấy media device của unicam/rp1-cfe. Overlay ov9281 đã nạp chưa?"; exit 1; }

# Tên entity của sensor, ví dụ: "ov9281 10-0060"
ENTITY="$(media-ctl -d "$MEDIA_DEV" -p 2>/dev/null \
          | sed -n 's/^- entity [0-9]*: \(ov928[12] [0-9a-f]*-[0-9a-f]*\) .*/\1/p' | head -1)"
[[ -n "$ENTITY" ]] || { echo "[!] Không thấy entity ov9281 trong $MEDIA_DEV."; exit 1; }

SUBDEV="$(media-ctl -d "$MEDIA_DEV" -e "$ENTITY" 2>/dev/null)"
[[ -n "$SUBDEV" && -e "$SUBDEV" ]] || { echo "[!] Không xác định được subdev cho '$ENTITY'."; exit 1; }

echo "[i] media device : $MEDIA_DEV"
echo "[i] sensor entity: $ENTITY"
echo "[i] sensor subdev: $SUBDEV"

# ---------------------------------------------------------------------------
# 1) Ép định dạng subdev cho khớp với thứ ứng dụng sẽ xin ở /dev/videoN
# ---------------------------------------------------------------------------
echo "[1/3] Đặt subdev -> ${MBUS}/${WIDTH}x${HEIGHT}"
if ! media-ctl -d "$MEDIA_DEV" --set-v4l2 "\"${ENTITY}\":0[fmt:${MBUS}/${WIDTH}x${HEIGHT}]" 2>&1; then
  echo "[!] Đặt format thất bại — kiểm tra sensor có hỗ trợ ${MBUS} ở ${WIDTH}x${HEIGHT} không:"
  echo "    v4l2-ctl -d $SUBDEV --list-subdev-mbus-codes"
  exit 1
fi

ACTUAL="$(v4l2-ctl -d "$SUBDEV" --get-subdev-fmt 0 2>/dev/null | sed -n 's/.*Mediabus Code *: \(.*\)/\1/p')"
echo "    -> $(v4l2-ctl -d "$SUBDEV" --get-subdev-fmt 0 2>/dev/null | sed -n 's/.*Width\/Height *: \(.*\)/\1/p') ${ACTUAL}"

# ---------------------------------------------------------------------------
# 2) vertical_blanking (quyết định FPS) rồi mới tới exposure / gain
#
# THỨ TỰ QUAN TRỌNG: driver chốt trần exposure theo vts = height + vertical_blanking.
# Hạ vblank trước rồi mới đặt exposure, nếu không exposure sẽ bị cắt xuống im lặng.
#
# Thời gian 1 khung = (height + vblank) x (width + hblank) / pixel_rate
# ---------------------------------------------------------------------------
echo "[2/3] vertical_blanking / exposure / gain"
if [[ -n "$VBLANK" ]]; then
  v4l2-ctl -d "$SUBDEV" --set-ctrl "vertical_blanking=${VBLANK}" 2>&1 \
    || echo "    [!] đặt vertical_blanking thất bại — xem dải hợp lệ bằng: v4l2-ctl -d $SUBDEV --list-ctrls"
fi

if [[ -n "$EXPOSURE" || -n "$GAIN" ]]; then
  CTRLS=""
  [[ -n "$EXPOSURE" ]] && CTRLS="exposure=${EXPOSURE}"
  [[ -n "$GAIN" ]] && CTRLS="${CTRLS:+$CTRLS,}analogue_gain=${GAIN}"
  v4l2-ctl -d "$SUBDEV" --set-ctrl "$CTRLS" 2>&1 \
    || echo "    [!] đặt control thất bại — xem dải hợp lệ bằng: v4l2-ctl -d $SUBDEV --list-ctrls"
fi

v4l2-ctl -d "$SUBDEV" --list-ctrls 2>/dev/null \
  | grep -E 'vertical_blanking|exposure|analogue_gain' | sed 's/^/    /'

# Cảnh báo nếu exposure bị driver cắt so với giá trị yêu cầu
if [[ -n "$EXPOSURE" ]]; then
  ACTUAL_EXP="$(v4l2-ctl -d "$SUBDEV" --get-ctrl exposure 2>/dev/null | sed -n 's/.*: *//p')"
  if [[ -n "$ACTUAL_EXP" && "$ACTUAL_EXP" != "$EXPOSURE" ]]; then
    echo "    [!] exposure bị cắt: yêu cầu ${EXPOSURE} -> thực tế ${ACTUAL_EXP}"
    echo "        (trần exposure phụ thuộc vertical_blanking; tăng --vblank nếu cần phơi sáng lâu hơn)"
  fi
fi

# Ước lượng FPS trần từ các thông số hiện tại
VB_NOW="$(v4l2-ctl -d "$SUBDEV" --get-ctrl vertical_blanking 2>/dev/null | sed -n 's/.*: *//p')"
HB_NOW="$(v4l2-ctl -d "$SUBDEV" --get-ctrl horizontal_blanking 2>/dev/null | sed -n 's/.*: *//p')"
PR_NOW="$(v4l2-ctl -d "$SUBDEV" --get-ctrl pixel_rate 2>/dev/null | sed -n 's/.*: *//p')"
if [[ -n "$VB_NOW" && -n "$HB_NOW" && -n "$PR_NOW" ]]; then
  awk -v h="$HEIGHT" -v w="$WIDTH" -v vb="$VB_NOW" -v hb="$HB_NOW" -v pr="$PR_NOW" \
    'BEGIN { ft=(h+vb)*(w+hb)/pr; printf("    -> FPS trần lý thuyết: %.1f  (frame %.1f ms)\n", 1/ft, ft*1000) }'
fi

# ---------------------------------------------------------------------------
# 3) Gợi ý lệnh chạy
# ---------------------------------------------------------------------------
VIDEO_DEV="$(v4l2-ctl --list-devices 2>/dev/null \
             | awk '/unicam|rp1-cfe/{f=1;next} f&&/\/dev\/video/{gsub(/^[ \t]+/,"");print;exit}')"
VIDEO_DEV="${VIDEO_DEV:-/dev/video0}"

if [[ "$QUIET" -eq 1 ]]; then
  echo "[3/3] Sẵn sàng (${PIXFMT} ${WIDTH}x${HEIGHT} trên ${VIDEO_DEV})."
  exit 0
fi

echo "[3/3] Sẵn sàng. Chạy node ROS 2:"
echo
echo "    ros2 run v4l2_camera v4l2_camera_node --ros-args \\"
echo "      -p video_device:=${VIDEO_DEV} \\"
echo "      -p pixel_format:=${PIXFMT} \\"
echo "      -p output_encoding:=mono8 \\"
echo "      -p image_size:=\"[${WIDTH},${HEIGHT}]\""
echo
echo "  Kiểm tra nhanh không cần ROS:"
echo "    v4l2-ctl -d ${VIDEO_DEV} --set-fmt-video=width=${WIDTH},height=${HEIGHT},pixelformat=${PIXFMT} \\"
echo "             --stream-mmap --stream-count=5 --stream-to=/tmp/ov9281.raw"
echo
echo "  Nếu ảnh ra đen kịt (mean ~16, std <2): sensor không nhận được ánh sáng."
echo "  Tháo nắp ống kính, bật đèn, rồi tăng --exposure / --gain."
