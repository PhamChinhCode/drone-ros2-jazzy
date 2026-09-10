#!/usr/bin/env bash
# Chẩn đoán nhanh camera OV9281. Xem giải thích chi tiết ở docs/CAMERA.md (mục 4.3, 9).
set -uo pipefail

echo "== liệt kê camera qua libcamera =="
if command -v rpicam-hello >/dev/null; then
  rpicam-hello --list-cameras
elif command -v cam >/dev/null; then
  cam -l          # Ubuntu: libcamera-tools, không có rpicam-*
else
  echo "(chưa có rpicam-hello lẫn cam — chạy scripts/install.sh trước)"
fi

echo
echo "== i2cdetect -y 10 (0x60 = sensor, UU = driver đã chiếm là BÌNH THƯỜNG) =="
if command -v i2cdetect >/dev/null; then
  sudo i2cdetect -y 10 2>&1 || echo "(bus i2c-10 không tồn tại trên máy này — số bus có thể khác, thử i2cdetect -l)"
else
  echo "(i2c-tools chưa cài)"
fi

echo
echo "== lsmod | grep -E 'ov9282|unicam|rp1_cfe' =="
lsmod | grep -E 'ov9282|unicam|rp1_cfe' || echo "(không thấy module nào — overlay có thể chưa nạp, xem config.txt)"

echo
echo "== IPA libcamera cho pipeline vc4 (Ubuntu cần gói libcamera-ipa) =="
IPA_DIR="$(ls -d /usr/lib/*/libcamera 2>/dev/null | head -1)"
if [[ -n "$IPA_DIR" && -e "$IPA_DIR/ipa_rpi_vc4.so" ]]; then
  echo "OK: $IPA_DIR/ipa_rpi_vc4.so"
  ls /usr/share/libcamera/ipa/rpi/vc4/ 2>/dev/null | grep -iE 'ov9281|uncalibrated' \
    || echo "(chưa thấy file tuning ov9281* — libcamera sẽ rơi về uncalibrated.json)"
else
  echo "THIẾU ipa_rpi_vc4.so -> 'cam -l' sẽ rỗng kèm lỗi 'Failed to load a suitable IPA library'."
  echo "  Sửa trên Ubuntu:  sudo apt install -y libcamera-ipa"
fi

echo
echo "== dmesg | grep -i ov928 (cần sudo để đọc đầy đủ) =="
sudo dmesg 2>/dev/null | grep -i ov928 || echo "(không có dòng log liên quan, hoặc dmesg đã bị xoay vòng)"

echo
echo "== v4l2-ctl --list-devices =="
if command -v v4l2-ctl >/dev/null; then
  v4l2-ctl --list-devices
else
  echo "(v4l-utils chưa cài)"
fi

echo
echo "Nếu 'rpicam-hello --list-cameras' (hoặc 'cam -l' trên Ubuntu) không thấy ov9281:"
echo "kiểm tra dtoverlay=ov9281 trong /boot/firmware/config.txt rồi reboot. Chi tiết: docs/CAMERA.md mục 9."
