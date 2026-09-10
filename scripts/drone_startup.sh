#!/usr/bin/env bash
# Khởi động hệ thống drone: cấu hình camera + nạp môi trường ROS 2.
#
# HAI CÁCH CHẠY — khác nhau ở chỗ biến môi trường có ở lại shell hay không:
#
#   source ~/ros2_ws/scripts/drone_startup.sh   Dùng khi ngồi gõ lệnh tay. Sau khi chạy,
#                                               shell hiện tại có sẵn lệnh `ros2` và các
#                                               gói của workspace.
#
#   ~/ros2_ws/scripts/drone_startup.sh          Dùng cho systemd lúc boot. Môi trường ROS
#                                               CHỈ có bên trong script — mọi lệnh cần nó
#                                               phải nằm ở MỤC 3 bên dưới.
#
# Thêm việc mới về sau: viết vào MỤC 3.

# ---------------------------------------------------------------------------
# Tham số camera — chỉnh ở đây, không sửa rải rác bên dưới
# ---------------------------------------------------------------------------
CAM_WIDTH=640
CAM_HEIGHT=400
CAM_VBLANK=1779       # 60 FPS. KHÔNG dùng 110 (246 FPS): Pi 4 bão hoà, vỡ đồng bộ apriltag
CAM_EXPOSURE=400      # không có auto-exposure — phòng tối thì tăng (trần 2154 ở vblank này)
CAM_GAIN=60

# Tự định vị: suy ra vị trí workspace từ chỗ script đang nằm, để clone về đâu
# cũng chạy được mà không phải sửa đường dẫn (giống scripts/run_camera_node.sh).
# BASH_SOURCE[0] đúng cả khi script được `source` lẫn khi chạy trực tiếp.
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_ROOT="$(dirname "$SCRIPT_DIR")"

ROS_SETUP=/opt/ros/jazzy/setup.bash          # ngoài workspace nên vẫn để tuyệt đối
WS_SETUP="$WS_ROOT/install/setup.bash"
CAM_SETUP="$SCRIPT_DIR/camera_v4l2_setup.sh"

# Được source hay chạy trực tiếp? Quyết định lệnh dừng và có bật `set -u` hay không
# (bật `set -u` trong shell tương tác của người dùng sẽ gây phiền).
SOURCED=0
[[ "${BASH_SOURCE[0]}" != "$0" ]] && SOURCED=1
# KHÔNG bật `set -u`: /opt/ros/jazzy/setup.bash đọc biến chưa đặt
# (AMENT_TRACE_SETUP_FILES) nên sẽ chết ngay ở MỤC 2.
if (( SOURCED )); then DUNG=return; else DUNG=exit; set -o pipefail; fi

# ---------------------------------------------------------------------------
# MỤC 1 — Cấu hình V4L2 cho camera (RESET sau mỗi lần reboot, phải chạy lại)
#
# Vòng chờ ở đây là để dùng được lúc boot: udev tạo /dev/media* SAU khi systemd
# khởi động service, và /dev/video0 không có TAG systemd nên không đợi bằng
# `After=dev-video0.device` được. Chạy tay thì vòng này qua ngay lần lặp đầu.
# ---------------------------------------------------------------------------
echo "[1/3] Cấu hình camera V4L2 (${CAM_WIDTH}x${CAM_HEIGHT}, vblank=${CAM_VBLANK})"

CAM_OK=0
for _ in $(seq 30); do
  for m in /dev/media*; do
    [[ -e "$m" ]] || continue
    if media-ctl -d "$m" -p 2>/dev/null | grep -q 'entity .*ov928'; then CAM_OK=1; break 2; fi
  done
  sleep 1
done

if (( ! CAM_OK )); then
  echo "[!] Không thấy sensor ov9281 sau 30 s — overlay đã nạp chưa? (dtoverlay=ov9281)" >&2
  $DUNG 1
fi

"$CAM_SETUP" --width "$CAM_WIDTH" --height "$CAM_HEIGHT" --vblank "$CAM_VBLANK" \
             --exposure "$CAM_EXPOSURE" --gain "$CAM_GAIN" --quiet \
  || { echo "[!] Cấu hình camera thất bại — dừng." >&2; $DUNG 1; }

# ---------------------------------------------------------------------------
# MỤC 2 — Nạp môi trường ROS 2
# ---------------------------------------------------------------------------
echo "[2/3] Nạp môi trường ROS 2"

# shellcheck disable=SC1090
source "$ROS_SETUP" || { echo "[!] Không nạp được $ROS_SETUP" >&2; $DUNG 1; }
# shellcheck disable=SC1090
source "$WS_SETUP"  || { echo "[!] Không nạp được $WS_SETUP — đã colcon build chưa?" >&2; $DUNG 1; }

echo "    ROS_DISTRO=${ROS_DISTRO}  workspace=$(dirname "$(dirname "$WS_SETUP")")"

# ---------------------------------------------------------------------------
# MỤC 3 — CHỖ THÊM LỆNH VỀ SAU (đang để trống có chủ ý)
#
# Viết lệnh vào đây. Lưu ý khi script được `source`: lệnh chạy nền (kết thúc bằng &)
# thì shell vẫn dùng tiếp được; lệnh chạy nền trước (như `ros2 launch`) sẽ GIỮ shell
# cho tới khi Ctrl+C.
#
# Ví dụ (bỏ dấu # để dùng):
#   ros2 launch drone_bringup perception.launch.py
#   ros2 launch drone_bringup full_system.launch.py
#   ros2 run foxglove_bridge foxglove_bridge &
# ---------------------------------------------------------------------------
echo "[3/3] Xong. Chưa có lệnh nào ở MỤC 3."
