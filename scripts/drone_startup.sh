#!/usr/bin/env bash
# Khởi động hệ thống drone: cấu hình camera + nạp môi trường ROS 2.
#
# HAI CÁCH CHẠY — khác nhau ở chỗ biến môi trường có ở lại shell hay không:
#
#   source ~/ros2_ws/scripts/drone_startup.sh   Dùng khi ngồi gõ lệnh tay. Sau khi chạy,
#                                               shell hiện tại có sẵn lệnh `ros2` và các
#                                               gói của workspace.
#
#   ~/ros2_ws/scripts/drone_startup.sh          Dùng cho systemd lúc boot: cấu hình camera,
#                                               nạp ROS rồi CHẠY CẢ STACK (MỤC 3) cho tới khi
#                                               service dừng.
#
# Thêm việc mới về sau: viết vào MỤC 3.

# ---------------------------------------------------------------------------
# Tham số camera — chỉnh ở đây, không sửa rải rác bên dưới
# ---------------------------------------------------------------------------
CAM_WIDTH=640
CAM_HEIGHT=400
CAM_VBLANK=3957       # 30 FPS (đo 09-14). Toàn hệ thống trên Pi 4 bão hoà CPU: ở 60 FPS apriltag chỉ
                      # 3,9 Hz, ở 30 FPS lên 15-16 Hz. KHÔNG dùng 110 (246 FPS).
CAM_EXPOSURE=300      # không có auto-exposure. Đo 09-14 trong phòng: 800/120 cháy sáng, tag không bắt
CAM_GAIN=32           # được; 300/32 bắt ổn định. Ngoài trời phải đo lại.

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
# MỤC 3 — Chạy cả stack (CHỈ khi chạy trực tiếp, tức từ systemd)
#
# `source` bằng tay thì dừng ở đây: shell có môi trường ROS và người dùng tự launch.
#
# Chạy stack KHÔNG làm drone tự cất cánh: cất cánh cần GCS gửi kế hoạch + MISSION_START,
# người lái bật công tắc cho phép OFFBOARD (ch8) và FC báo OB_ARM_RDY = 1.
#
# `exec`: tiến trình `ros2 launch` THAY chỗ script, để SIGINT của systemd (KillSignal)
# tới thẳng launch — tắt êm như Ctrl+C, bag kịp đóng file. Không exec thì SIGINT chỉ
# tới bash và launch bị SIGKILL khi hết TimeoutStopSec.
# ---------------------------------------------------------------------------
if (( SOURCED )); then
  echo "[3/3] Đã nạp môi trường. Chạy stack: ros2 launch drone_bringup full_system.launch.py"
  return 0
fi

# Phòng ngừa: đợi có IPv4 (không phải loopback) rồi mới launch. network-online.target tới khi
# wlan0 mới có carrier, CHƯA có IPv4, mà Fast DDS chọn giao diện lúc node khởi tạo.
# Lỗi "các node không thấy nhau" gặp 2026-09-18 rốt cuộc do logind RemoveIPC xoá bộ nhớ chia sẻ
# (xem systemd/10-drone-keep-ipc.conf), không chắc do thiếu IPv4.
# ĐỪNG dùng ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST: Fast DDS khi đó chỉ dò vài participant đầu
# trên localhost, stack >20 tiến trình thì phần lớn node không thấy nhau (đã thử, EKF trôi).
for _ in $(seq 60); do
  ip -4 -o addr show scope global | grep -q . && break
  sleep 1
done
if ! ip -4 -o addr show scope global | grep -q .; then
  echo "[!] Chưa có IPv4 sau 60 s - vẫn chạy, nhưng các node có thể không thấy nhau" >&2
fi

echo "[3/3] Chạy stack: ros2 launch drone_bringup full_system.launch.py"
exec ros2 launch drone_bringup full_system.launch.py
