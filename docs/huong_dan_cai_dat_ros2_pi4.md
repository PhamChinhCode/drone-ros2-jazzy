# Danh sách gói & hướng dẫn cài ROS 2 trên Raspberry Pi 4 (Ubuntu Server 24.04)

Companion computer (Pi 4) đóng vai trò: nhận ảnh từ camera → phát hiện marker ArUco/AprilTag làm điểm neo → tính optical flow để đo vận tốc trôi ngang → hợp nhất (EKF) với IMU từ flight controller qua MAVLink → tính PID vị trí → gửi setpoint xuống STM32H743 qua MAVROS.

Ubuntu Server 24.04 LTS (Noble) là bản chính thức đi kèm **ROS 2 Jazzy Jalisco** (LTS, hỗ trợ đến 2029), nên toàn bộ hướng dẫn dưới đây dùng Jazzy.

## 1. Danh sách gói cần cài

### A. Nền tảng hệ thống

| Gói | Vai trò |
|---|---|
| `ros-jazzy-ros-base` | Lõi ROS 2 Jazzy, bản không GUI — đúng cho Server headless |
| `python3-colcon-common-extensions` | Công cụ build workspace ROS 2 |
| `python3-rosdep`, `python3-vcstool` | Quản lý dependency & mã nguồn ROS |
| `build-essential`, `cmake`, `git` | Toolchain build C++/CMake |

### B. Giao tiếp với Flight Controller (STM32H743, MAVLink)

| Gói | Vai trò |
|---|---|
| `ros-jazzy-mavros` | Cầu nối MAVLink ⇄ ROS 2 (topic, service, OFFBOARD) |
| `ros-jazzy-mavros-extras` | Thêm plugin MAVROS (distance sensor, gimbal...) |
| `install_geographiclib_datasets.sh` (đi kèm mavros) | Dữ liệu geoid, bắt buộc cho các plugin dùng vị trí toàn cầu |

### C. Camera & xử lý ảnh cơ bản

| Gói | Vai trò |
|---|---|
| `python3-opencv` | Thư viện OpenCV cho node optical flow tự viết |
| `v4l-utils` | Kiểm tra/chẩn đoán camera USB (V4L2) |
| `ros-jazzy-usb-cam` hoặc `ros-jazzy-v4l2-camera` | Driver ROS 2 cho webcam USB (khuyên dùng — đơn giản nhất trên Pi 4) |
| `camera_ros` (build từ source, xem mục 4) | Driver libcamera cho Pi Camera Module (CSI) — chỉ cần nếu dùng camera CSI thay vì USB |
| `ros-jazzy-image-transport`, `ros-jazzy-image-transport-plugins` | Nén/truyền ảnh giữa các node |
| `ros-jazzy-cv-bridge` | Chuyển đổi ảnh ROS ⇄ mảng OpenCV |
| `ros-jazzy-image-proc`, `ros-jazzy-camera-calibration`, `ros-jazzy-camera-calibration-parsers`, `ros-jazzy-camera-info-manager` | Hiệu chỉnh (calibrate) & rectify ảnh — bắt buộc để ArUco/AprilTag đo khoảng cách 3D chính xác |

### D. Định vị bằng Marker (điểm neo)

| Gói | Vai trò |
|---|---|
| `ros-jazzy-aruco-opencv` | Phát hiện ArUco marker, publish pose 3D |
| `ros-jazzy-apriltag`, `ros-jazzy-apriltag-ros` | Phát hiện AprilTag, publish pose 3D (độ chính xác góc thường tốt hơn ArUco) |
| `ros-jazzy-tf2-ros`, `ros-jazzy-tf2-geometry-msgs` | Quản lý hệ tọa độ camera → drone → marker |

### E. Optical flow & hợp nhất cảm biến (EKF)

| Gói | Vai trò |
|---|---|
| OpenCV (đã có ở mục C) | `cv2.calcOpticalFlowPyrLK` / `calcOpticalFlowFarneback` dùng trực tiếp trong node Python tự viết, không cần gói ROS riêng |
| `ros-jazzy-robot-localization` | Node EKF/UKF hợp nhất vận tốc optical flow + pose marker + IMU (từ MAVROS) thành Odometry mượt, chống trôi |

### F. Điều khiển & tiện ích debug (tuỳ chọn)

| Gói | Vai trò |
|---|---|
| Node PID tự viết bằng `rclpy` | Không cần gói thêm |
| `ros-jazzy-rqt-image-view`, `ros-jazzy-rviz2` | Xem ảnh/marker/TF trực quan — chỉ hữu ích nếu có X11 forwarding hoặc dùng từ máy trạm khác, vì Server không có GUI |
| `ros-jazzy-teleop-twist-keyboard` | Test gửi lệnh vận tốc thủ công qua bàn phím |

## 2. Cài đặt hệ điều hành

1. Dùng Raspberry Pi Imager, chọn **Ubuntu Server 24.04.x LTS (64-bit)** cho Raspberry Pi 4.
2. Trong phần cấu hình nâng cao (biểu tượng bánh răng): bật SSH, đặt username/password, cấu hình Wi-Fi nếu cần, chọn locale/timezone.
3. Ghi ra thẻ nhớ, cắm vào Pi 4, khởi động, SSH vào Pi.

## 3. Cập nhật hệ thống & chuẩn bị locale

```bash
sudo apt update && sudo apt upgrade -y
sudo apt install -y locales software-properties-common curl gnupg
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8
sudo add-apt-repository universe -y
```

## 4. Thêm kho ROS 2 apt (Jazzy)

Phương pháp hiện hành dùng gói `ros-apt-source` (thay cho cách cũ tự thêm key + sources.list):

```bash
export ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F\" '{print $4}')
curl -L -o /tmp/ros2-apt-source.deb \
  "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo $VERSION_CODENAME)_all.deb"
sudo apt install -y /tmp/ros2-apt-source.deb
sudo apt update && sudo apt upgrade -y
```

## 5. Cài ROS 2 Jazzy (bản base, không GUI)

```bash
sudo apt install -y ros-jazzy-ros-base
sudo apt install -y python3-colcon-common-extensions python3-rosdep python3-vcstool python3-argcomplete build-essential cmake git
sudo rosdep init
rosdep update

echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

## 6. Cài MAVROS (kết nối STM32H743 qua MAVLink)

```bash
sudo apt install -y ros-jazzy-mavros ros-jazzy-mavros-extras
sudo /opt/ros/jazzy/lib/mavros/install_geographiclib_datasets.sh

# cấp quyền đọc/ghi cổng serial cho user hiện tại
sudo usermod -aG dialout $USER
# đăng xuất rồi đăng nhập lại (hoặc reboot) để quyền có hiệu lực
```

Kiểm tra kết nối (sửa `/dev/ttyUSB0` và baudrate cho đúng cấu hình MAVLink bạn đã làm trên H743):

```bash
ros2 launch mavros px4.launch fcu_url:=/dev/ttyUSB0:921600
# ở terminal khác:
ros2 topic echo /mavros/state
```

## 7. Cài gói camera & xử lý ảnh

```bash
sudo apt install -y python3-opencv v4l-utils
sudo apt install -y ros-jazzy-usb-cam ros-jazzy-v4l2-camera
sudo apt install -y ros-jazzy-cv-bridge ros-jazzy-image-transport ros-jazzy-image-transport-plugins
sudo apt install -y ros-jazzy-image-proc ros-jazzy-camera-calibration ros-jazzy-camera-calibration-parsers ros-jazzy-camera-info-manager
```

Kiểm tra camera USB được nhận diện:

```bash
v4l2-ctl --list-devices
```

**Nếu dùng Pi Camera Module (cổng CSI) thay vì webcam USB**: gói `camera_ros` (driver libcamera) thường chưa có sẵn ổn định qua apt cho mọi bản dựng Ubuntu 24.04 trên Pi, nên phải build từ source (xem mục 9). Nếu không bắt buộc phải dùng camera CSI, dùng webcam USB sẽ đỡ mất công hơn nhiều.

## 8. Cài gói định vị marker (ArUco/AprilTag) & sensor fusion

```bash
sudo apt install -y ros-jazzy-aruco-opencv
sudo apt install -y ros-jazzy-apriltag ros-jazzy-apriltag-ros
sudo apt install -y ros-jazzy-tf2-ros ros-jazzy-tf2-geometry-msgs
sudo apt install -y ros-jazzy-robot-localization
```

## 9. Tạo workspace cho node tự viết (PID, optical flow) và build gói thiếu bản apt

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src

# ví dụ: chỉ cần nếu dùng Pi Camera CSI thay vì webcam USB
# git clone https://github.com/christianrauch/camera_ros.git

cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install --parallel-workers 2

echo "source ~/ros2_ws/install/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

Pi 4 có RAM hạn chế (2/4/8GB tuỳ bản) — build từ source (như `camera_ros`) dễ bị OOM. Nên giới hạn `--parallel-workers 2` như trên, và thêm swap trước khi build:

```bash
sudo fallocate -l 2G /swapfile
sudo chmod 600 /swapfile
sudo mkswap /swapfile
sudo swapon /swapfile
echo '/swapfile none swap sw 0 0' | sudo tee -a /etc/fstab
```

## 10. Kiểm tra tổng thể

```bash
ros2 --version
ros2 pkg list | grep -E "mavros|aruco|apriltag|robot_localization|image_transport|usb_cam"
```

Nếu tất cả lệnh trên chạy không lỗi và `ros2 topic echo /mavros/state` thấy `connected: true` khi FC đã cắm, coi như phần cài đặt trên Pi 4 đã xong — bước tiếp theo là viết node optical flow, node PID và file launch hợp nhất tất cả (camera → ArUco/AprilTag → robot_localization → PID → MAVROS).

## 11. (Tuỳ chọn) Tự khởi động khi Pi boot bằng systemd

Tạo file `/etc/systemd/system/drone-ros2.service`:

```ini
[Unit]
Description=Drone ROS2 stack
After=network.target

[Service]
Type=simple
User=<ten_user_cua_ban>
ExecStart=/bin/bash -lc 'source /opt/ros/jazzy/setup.bash && source ~/ros2_ws/install/setup.bash && ros2 launch <ten_package_cua_ban> drone.launch.py'
Restart=on-failure

[Install]
WantedBy=multi-user.target
```

Kích hoạt:

```bash
sudo systemctl daemon-reload
sudo systemctl enable --now drone-ros2.service
```

## Ghi chú hiệu năng trên Pi 4

Pi 4 (Cortex-A72, 4 nhân 1.5GHz) yếu hơn đáng kể so với Pi 5 — với optical flow + ArUco/AprilTag chạy đồng thời, nên thử ở độ phân giải thấp (640×480) và giới hạn khung hình (15–30 fps) trước khi tăng dần. Có thể đặt CPU governor về `performance` để tránh throttling do tiết kiệm điện:

```bash
echo performance | sudo tee /sys/devices/system/cpu/cpu*/cpufreq/scaling_governor
```

Đảm bảo tản nhiệt tốt (heatsink + quạt) vì chạy governor performance liên tục dễ làm Pi 4 nóng và tự giảm xung khi quá nhiệt.
