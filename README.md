# Drone ROS 2 Workspace

Workspace ROS 2 Jazzy cho drone hạ cánh chính xác bằng AprilTag, chạy trên
Raspberry Pi 4 (Ubuntu 24.04) với camera OV9281 và flight controller Pixhawk.

Gộp từ hai nơi cũ: workspace ROS (`ros2_ws`) và repo bring-up camera (`PiDrone`).

## Cấu trúc

```
├── src/            9 package ROS 2 — mã nguồn chính
├── scripts/        chạy TRÊN drone, NGOÀI môi trường ROS
├── tools/          chạy tay lúc phát triển (hiệu chỉnh, benchmark, in tag)
├── systemd/        unit file cho boot
├── assets/         tag in sẵn, bàn cờ hiệu chỉnh, ảnh mẫu
├── docs/           tài liệu thiết kế + vận hành
└── build/ install/ log/     do colcon sinh ra, KHÔNG commit
```

`scripts/` nằm ở gốc workspace chứ không nằm trong package, vì
`camera_v4l2_setup.sh` phải chạy **trước** khi source môi trường ROS — không thể
định vị nó bằng `ros2 pkg prefix` được.

## Bắt đầu

```bash
colcon build
source install/setup.bash
ros2 launch drone_bringup full_system.launch.py
```

Xem hệ thống trực tiếp: mở Foxglove → `ws://<ip-drone>:8765`.

## Hai điều dễ vấp

**1. Camera phải cấu hình lại sau MỖI lần reboot.** Cài đặt V4L2 bị reset. Bỏ qua
thì topic vẫn ra đúng nhịp (30 Hz) nhưng **mọi khung hình toàn số 0** — `ros2 topic hz`
không phát hiện được, phải kiểm thống kê pixel.

```bash
scripts/camera_v4l2_setup.sh --width 640 --height 400 --vblank 3957 --exposure 300 --gain 32
```

`scripts/drone_startup.sh` đã làm sẵn việc này (MỤC 1) rồi mới source ROS (MỤC 2).

**2. `ros2 launch` đọc từ `install/`, không đọc từ `src/`.** Sửa file trong `src/`
mà chưa `colcon build` thì chạy vẫn ra bản cũ.

## Tự chạy lúc boot

Cắm điện Pi là cả hệ thống tự lên: `drone-startup.service` gọi `scripts/drone_startup.sh`, script
cấu hình camera V4L2 (MỤC 1), nạp ROS (MỤC 2) rồi `exec ros2 launch drone_bringup full_system.launch.py`
(MỤC 3). Mất khoảng 1 phút sau khi Pi có mạng thì GCS thấy drone.

**Cài một lần** (và cài lại mỗi khi sửa `systemd/drone-startup.service`):

```bash
sudo cp systemd/drone-startup.service /etc/systemd/system/
sudo systemctl daemon-reload
sudo systemctl enable --now drone-startup.service
```

**Vận hành:**

| Việc | Lệnh |
|---|---|
| Xem trạng thái | `systemctl status drone-startup` |
| Xem log stack (thay `/tmp/full_system.log` cũ) | `journalctl -u drone-startup -f` |
| Tắt stack (SIGINT như Ctrl+C, bag đóng file) | `sudo systemctl stop drone-startup` |
| Khởi động lại stack (vd sau khi nạp bản đồ tag qua dây) | `sudo systemctl restart drone-startup` |
| Không tự chạy lúc boot nữa | `sudo systemctl disable drone-startup` |

- **Tắt service trước khi `colcon build` hoặc chạy `ros2 launch` bằng tay.** Hai stack chạy song song là
  hai `gcs_link_node` tranh cổng UDP, hai MAVROS tranh cổng serial (xem `docs/HUONG_DAN_BUILD_CHAY.md`
  mục 1b). Build xong thì `sudo systemctl start drone-startup`.
- Tự chạy stack **không** làm drone tự cất cánh: cất cánh cần GCS gửi kế hoạch + lệnh bắt đầu, người lái bật
  công tắc cho phép OFFBOARD (ch8) và FC báo sẵn sàng arm.
- Camera không lên (overlay chưa nạp, cáp lỏng) thì service thử lại 5 lần trong 2 phút rồi dừng —
  `journalctl -u drone-startup` có dòng `[!] Không thấy sensor ov9281`. Launch chết bất thường thì tự
  chạy lại sau 5 s.
- Mỗi lần chạy bag ghi vào thư mục riêng `~/drone_logs/bag_<ngày>_<giờ>` (không ghi ảnh thô, khoảng
  60 MB/giờ). Pi 4 không có đồng hồ thời gian thực nên giờ trong tên lấy theo lúc Pi đồng bộ được giờ.
- Muốn chỉ nạp môi trường để gõ lệnh tay: `source scripts/drone_startup.sh` — dừng sau MỤC 2, không launch.

## Tài liệu

| File | Nội dung |
|---|---|
| `docs/so_do_he_thong.drawio` | **Bắt đầu từ đây** — sơ đồ khởi động, cây launch, luồng dữ liệu giữa các node |
| `docs/RUNBOOK.md` | Quy trình vận hành |
| `docs/GIAO_UOC_FC_ROS2.md` | **Hợp đồng FC ↔ Pi** — mọi thứ đi qua dây MAVLink tới flight controller |
| `docs/GIAO_UOC_GCS_PI.md` | **Hợp đồng GCS ↔ Pi** — kênh nhiệm vụ/telemetry qua 4G. Bản thảo 0.1, chờ GCS |
| `docs/CAMERA.md` | Camera OV9281: cấu hình, sự cố đã gặp và cách xử lý |
| `docs/thiet_ke_kien_truc_node_ros2.md` | Kiến trúc node |
| `docs/ke_hoach_thiet_ke_node.md` | Quyết định thiết kế và lý do |
| `docs/huong_dan_chi_tiet_cac_node_ros2_pi4.md` | Chi tiết từng node |
| `docs/nhat_ky_lam_viec.md` | Nhật ký làm việc theo phiên |
| `docs/huong_dan_cai_dat_ros2_pi4.md`, `docs/cai_dat_ros2_framework_pi4.md` | Cài đặt ROS 2 trên Pi |

## Ghi chú

`src/camera_ros/` là clone từ [upstream](https://github.com/christianrauch/camera_ros)
(tag 0.5.2), đã gitignore. **Hiện không được dùng** — dự án chọn `v4l2_camera` đọc
thẳng `/dev/video0` thay vì libcamera (lý do ở `docs/CAMERA.md` mục 9).
