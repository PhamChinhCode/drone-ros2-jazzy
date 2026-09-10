#!/usr/bin/env bash
# Cài đặt môi trường cho camera OV9281 (CSI) trên Raspberry Pi.
# Tự nhận diện Raspberry Pi OS vs Ubuntu để chọn đúng bộ gói.
# Tham khảo đầy đủ: docs/CAMERA.md
#
# Dùng:
#   ./scripts/install.sh                 # đầu camera mặc định (CAM1)
#   ./scripts/install.sh --cam0          # dùng đầu CAM0 (Compute Module / Pi 5)
#   ./scripts/install.sh --rotation 180  # lắp ngược
set -euo pipefail

OVERLAY_PARAMS=""
while [[ $# -gt 0 ]]; do
  case "$1" in
    --cam0)
      OVERLAY_PARAMS="${OVERLAY_PARAMS},cam0"
      shift
      ;;
    --rotation)
      OVERLAY_PARAMS="${OVERLAY_PARAMS},rotation=${2:?thiếu giá trị cho --rotation}"
      shift 2
      ;;
    *)
      echo "[!] Tham số không nhận diện được: $1" >&2
      exit 1
      ;;
  esac
done

# ---------------------------------------------------------------------------
# Nhận diện phần cứng và bản phân phối
# ---------------------------------------------------------------------------
if ! grep -q "Raspberry Pi" /proc/device-tree/model 2>/dev/null; then
  echo "[!] Máy này không phải Raspberry Pi (/proc/device-tree/model không khớp)."
  echo "    Bỏ qua phần cấu hình overlay ov9281, chỉ cài các gói Python chung."
  IS_PI=0
else
  IS_PI=1
  echo "[i] Phần cứng: $(tr -d '\0' < /proc/device-tree/model)"
fi

# DISTRO: rpi-os | ubuntu | other
DISTRO="other"
if [[ -r /etc/os-release ]]; then
  # shellcheck disable=SC1091
  . /etc/os-release
  case "${ID:-}" in
    raspbian) DISTRO="rpi-os" ;;
    debian)   DISTRO="rpi-os" ;;   # Raspberry Pi OS mới báo ID=debian
    ubuntu)   DISTRO="ubuntu" ;;
    *)
      if [[ "${ID_LIKE:-}" == *debian* && -f /etc/rpi-issue ]]; then
        DISTRO="rpi-os"
      elif [[ "${ID_LIKE:-}" == *ubuntu* ]]; then
        DISTRO="ubuntu"
      fi
      ;;
  esac
fi
# /etc/rpi-issue chỉ có trên Raspberry Pi OS, không có trên Ubuntu
if [[ "$DISTRO" == "other" && -f /etc/rpi-issue ]]; then
  DISTRO="rpi-os"
fi
echo "[i] Bản phân phối: ${PRETTY_NAME:-unknown} (nhánh gói: $DISTRO)"

# ---------------------------------------------------------------------------
# [1/3] Gói hệ thống
# ---------------------------------------------------------------------------
echo "[1/3] apt update & cài gói hệ thống..."
sudo apt update

COMMON_PKGS="python3-opencv v4l-utils python3-numpy git"
PKG_WARN=0

if [[ "$IS_PI" -eq 1 && "$DISTRO" == "rpi-os" ]]; then
  echo "    -> Raspberry Pi OS: rpicam-apps + picamera2 (stack rpicam)"
  sudo apt install -y rpicam-apps python3-picamera2 i2c-tools $COMMON_PKGS \
    || { echo "[!] apt install lỗi — kiểm tra lại bằng tay, script vẫn tiếp tục phần overlay."; PKG_WARN=1; }
elif [[ "$IS_PI" -eq 1 && "$DISTRO" == "ubuntu" ]]; then
  echo "    -> Ubuntu trên Pi: libcamera thuần (không có rpicam-*/python3-picamera2 qua apt)"
  # libcamera-ipa BẮT BUỘC: chứa ipa_rpi_vc4.so + file tuning (ov9281_mono.json).
  # Thiếu gói này thì 'cam -l' rỗng kèm lỗi "Failed to load a suitable IPA library".
  sudo apt install -y libcamera-tools libcamera-v4l2 libcamera-ipa python3-libcamera i2c-tools $COMMON_PKGS \
    || { echo "[!] apt install lỗi — kiểm tra lại bằng tay, script vẫn tiếp tục phần overlay."; PKG_WARN=1; }
elif [[ "$IS_PI" -eq 1 ]]; then
  echo "    -> Không xác định được bản phân phối trên Pi; thử bộ gói libcamera thuần."
  sudo apt install -y libcamera-tools libcamera-v4l2 libcamera-ipa python3-libcamera i2c-tools $COMMON_PKGS \
    || { echo "[!] apt install lỗi — kiểm tra lại bằng tay, script vẫn tiếp tục phần overlay."; PKG_WARN=1; }
else
  sudo apt install -y $COMMON_PKGS \
    || { echo "[!] apt install lỗi — kiểm tra lại bằng tay."; PKG_WARN=1; }
fi

# ---------------------------------------------------------------------------
# [2/3] Bật overlay ov9281 trong config.txt (mọi bản phân phối trên Pi)
# ---------------------------------------------------------------------------
if [[ "$IS_PI" -eq 1 ]]; then
  CONFIG_FILE="/boot/firmware/config.txt"
  [[ -f "$CONFIG_FILE" ]] || CONFIG_FILE="/boot/config.txt"
  echo "[2/3] Cấu hình $CONFIG_FILE cho overlay ov9281..."
  if [[ ! -f "$CONFIG_FILE" ]]; then
    echo "[!] Không thấy config.txt (/boot/firmware/config.txt hoặc /boot/config.txt) — bỏ qua."
  else
    ts=$(date +%Y%m%d-%H%M%S)
    backup="${CONFIG_FILE}.bak-${ts}"
    sudo cp "$CONFIG_FILE" "$backup"
    echo "    Đã sao lưu: $backup"

    if ! grep -q "^camera_auto_detect=0" "$CONFIG_FILE"; then
      if grep -q "^camera_auto_detect=" "$CONFIG_FILE"; then
        sudo sed -i 's/^camera_auto_detect=.*/camera_auto_detect=0/' "$CONFIG_FILE"
      else
        echo "camera_auto_detect=0" | sudo tee -a "$CONFIG_FILE" >/dev/null
      fi
      echo "    Đã đặt camera_auto_detect=0"
    fi

    overlay_line="dtoverlay=ov9281${OVERLAY_PARAMS}"
    if ! grep -q "^dtoverlay=ov9281" "$CONFIG_FILE"; then
      echo "$overlay_line" | sudo tee -a "$CONFIG_FILE" >/dev/null
      echo "    Đã thêm: $overlay_line"
    else
      echo "    Đã có dòng dtoverlay=ov9281 trong config.txt — kiểm tra thủ công nếu cần đổi tham số (cam0/rotation)."
    fi

    echo "[3/3] Thử nạp overlay ngay cho phiên hiện tại (nếu có lệnh dtoverlay)..."
    if command -v dtoverlay >/dev/null 2>&1; then
      sudo dtoverlay ov9281 ${OVERLAY_PARAMS//,/ } 2>/dev/null \
        || echo "    (không nạp live được, hãy reboot để áp dụng từ config.txt)"
    else
      echo "    (không có lệnh dtoverlay — bình thường trên Ubuntu; cần reboot để áp dụng)"
    fi
  fi
else
  echo "[2/3] Bỏ qua bước config.txt (không phải Raspberry Pi)."
fi

# ---------------------------------------------------------------------------
# Kết thúc
# ---------------------------------------------------------------------------
echo
if [[ "$IS_PI" -eq 1 && "$DISTRO" == "ubuntu" ]]; then
  echo "[Ubuntu] Không có python3-picamera2 qua apt. Nếu cần Picamera2:"
  echo "    pip install picamera2        # dựa trên python3-libcamera vừa cài"
  echo "  Hoặc đọc camera qua OpenCV/GStreamer libcamerasrc, hoặc build camera_ros từ source (xem mục 6C)."
  echo
fi
[[ "$PKG_WARN" -eq 1 ]] && echo "[!] Có bước apt bị lỗi ở trên — xem lại trước khi reboot." && echo
echo "Xong. Reboot một lần để nạp sạch overlay:"
echo "    sudo reboot"
echo "Sau reboot, kiểm tra:"
echo "    ls /dev/video*        # xuất hiện node cảm biến mới"
echo "    cam -l                # (Ubuntu) liệt kê camera qua libcamera"
echo "    v4l2-ctl --list-devices"
echo "    ./scripts/check_camera.sh"
