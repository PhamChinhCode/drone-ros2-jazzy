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

```bash
sudo cp systemd/drone-startup.service /etc/systemd/system/
sudo systemctl daemon-reload
sudo systemctl enable --now drone-startup.service
```

Mặc định unit chỉ cấu hình camera + nạp môi trường ROS rồi thoát (MỤC 3 của
`drone_startup.sh` để trống có chủ ý — không tự cất cánh khi cắm điện). Muốn tự
chạy cả stack thì thêm lệnh `ros2 launch` vào MỤC 3 **và** đổi unit sang
`Type=simple` — hướng dẫn ngay trong comment đầu file service.

## Tài liệu

| File | Nội dung |
|---|---|
| `docs/so_do_he_thong.drawio` | **Bắt đầu từ đây** — sơ đồ khởi động, cây launch, luồng dữ liệu giữa các node |
| `docs/RUNBOOK.md` | Quy trình vận hành |
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
