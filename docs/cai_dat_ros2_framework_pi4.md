# Hướng dẫn cài đặt framework ROS 2 trên Pi 4 `pi4ubuntu` (bản khớp máy thực tế)

> File này là bản **đã soi máy thật** (`pi4ubuntu`, `pc@192.168.10.127`) và điều chỉnh lại từ
> [`huong_dan_cai_dat_ros2_pi4.md`](huong_dan_cai_dat_ros2_pi4.md). File gốc là lý thuyết tổng quát;
> file này là quy trình copy‑paste theo đúng thứ tự cho **chính máy này**. Xem mục cuối để biết
> các điểm khác biệt so với file gốc.

Companion computer (Pi 4) làm: nhận ảnh camera **OV9281** (CSI) → phát hiện ArUco/AprilTag làm điểm neo
→ optical flow đo vận tốc trôi ngang → EKF hợp nhất với IMU (từ flight controller qua MAVLink) →
PID vị trí → gửi setpoint xuống STM32H743 qua MAVROS.

Bản ROS 2: **Jazzy Jalisco** (đi kèm Ubuntu 24.04, LTS tới 2029).

---

## 0. Trạng thái máy hiện tại (đã kiểm tra ngày 2026‑09‑06)

| Hạng mục | Thực tế | Việc cần làm |
|---|---|---|
| OS | Ubuntu 24.04.4 LTS `noble`, arm64 | — (đúng bản) |
| Phần cứng | Pi 4, Cortex‑A72 ×4, RAM 3.7 GB, thẻ còn ~216 GB | — |
| **Swap** | **0 B** | **Mục 2 — tạo 4 GB trước khi build** |
| ROS 2 | chưa cài, chưa có kho apt ROS | Mục 3–4 |
| Toolchain build | thiếu `gcc`, `cmake`, `colcon`, `rosdep`, `vcstool` | Mục 4 |
| `v4l-utils`, `i2c-tools` | thiếu | Mục 6 / `scripts/install.sh` |
| User `pc` | có `sudo`, **CHƯA thuộc `dialout`** | Mục 5 |
| Locale | `C.UTF-8` | Mục 1 |
| CPU governor | `ondemand` | Mục 9 (khi bay) |
| `universe` / `multiverse` | đã bật | — |
| `noble-updates` | thiếu trong `ubuntu.sources` | **Mục 1 — BẮT BUỘC** (thiếu → `ros-base` kẹt "held broken packages") |
| Camera OV9281 CSI | overlay chưa nạp, `camera_auto_detect=1` | Mục 6A |
| Flight controller | chưa cắm | Mục 5 (khi có FC) |
| Mạng / Internet | wlan0 OK, ra ngoài OK | — |

Checklist tổng: **1** locale → **2** gói nền + swap → **3** kho ROS → **4** ROS base + dev tools →
**5** MAVROS + quyền serial → **6** camera OV9281 + gói ảnh → **7** marker + sensor fusion →
**8** workspace + build → **9** hiệu năng → **10** kiểm tra.

Toàn bộ lệnh dưới đây chạy bằng user `pc`. Dòng nào cần quyền root đã có sẵn `sudo`.

> Cuối mỗi mục có khối **`✔️ Nhận biết đã xong`**: lệnh kiểm tra + **kết quả mong đợi**.
> Nếu kết quả khác mô tả → mục đó **chưa** xong, đừng sang mục sau. Bảng tổng ở [mục 10](#10-kiểm-tra-tổng-thể).

---

## 1. Locale + cập nhật hệ thống

Máy đang ở `C.UTF-8`; ROS 2 khuyến nghị UTF‑8 “thật”.

```bash
sudo apt update && sudo apt upgrade -y
sudo apt install -y locales software-properties-common curl gnupg lsb-release
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8
sudo add-apt-repository universe -y   # đã bật sẵn, chạy cho chắc — idempotent
```

Đăng xuất/đăng nhập lại (hoặc `source /etc/default/locale`) để `LANG` mới có hiệu lực toàn phiên.

**BẮT BUỘC — không phải tuỳ chọn.** `ubuntu.sources` trên máy này thiếu suite `noble-updates`.
File dùng định dạng deb822 với **2 khối riêng**: `Suites: noble` và `Suites: noble-security`
(KHÔNG phải một dòng `Suites: noble noble-security`). Bỏ qua bước này sẽ **kẹt ở [mục 4](#4-ros-2-jazzy-bản-base-không-gui--công-cụ-dev)**:
`noble-security` đã nâng `liblz4-1` / `libzstd1` lên `1.9.4-1build1.1` (và tương tự cho `libzstd`),
nhưng gói `-dev` khớp phiên bản đó **chỉ có trong `noble-updates`** → `sudo apt install ros-jazzy-ros-base`
báo *"held broken packages"*:

```
liblz4-dev  : Depends: liblz4-1  (= 1.9.4-1build1) but 1.9.4-1build1.1 is to be installed
libzstd-dev : Depends: libzstd1  (= 1.5.5+dfsg2-2build1) but 1.5.5+dfsg2-2build1.1 is to be installed
```

Cách sửa (đã kiểm chứng trên máy này):

```bash
sudo cp /etc/apt/sources.list.d/ubuntu.sources /etc/apt/sources.list.d/ubuntu.sources.bak
# thêm "noble-updates" vào ĐÚNG dòng chỉ có "noble" — mẫu $ ở cuối để KHÔNG đụng dòng noble-security:
sudo sed -i 's/^Suites: noble$/Suites: noble noble-updates/' /etc/apt/sources.list.d/ubuntu.sources
grep '^Suites' /etc/apt/sources.list.d/ubuntu.sources
#   mong đợi:
#     Suites: noble noble-updates
#     Suites: noble-security
#   nếu sed không đổi được (dòng gốc khác mẫu) → sudo nano ...ubuntu.sources, thêm tay " noble-updates" vào sau "noble"
sudo apt update
sudo apt full-upgrade -y          # đồng bộ liblz4/libzstd (+dev) về cùng bản 1.9.4-1build1.1
```

`full-upgrade` có thể in `needrestart` ("Service restarts being deferred", "User sessions running
outdated binaries") — **bình thường**, không cần làm gì; reboot sau cũng được.

**✔️ Nhận biết đã xong mục 1**

```bash
locale | grep -E 'LANG|LC_ALL'      # LANG=en_US.UTF-8  (KHÔNG còn C.UTF-8)
apt-cache policy | grep -c universe # số > 0  → universe đang bật
grep -c 'noble-updates' /etc/apt/sources.list.d/ubuntu.sources   # ≥ 1  → đã bật noble-updates
apt-cache policy liblz4-dev | grep Candidate                     # 1.9.4-1build1.1  (KHỚP liblz4-1 đã cài, không phải ...build1)
apt-get -s upgrade | tail -1        # "0 upgraded, 0 newly installed" hoặc danh sách hợp lệ, không lỗi
```

Kết quả sai điển hình: `locale` vẫn `C.UTF-8` → chưa đăng xuất/vào lại SSH.

---

## 2. Gói nền + swap (làm swap TRƯỚC khi build source)

Máy chưa có swap. RAM 4 GB + không swap → `colcon build` (nhất là `camera_ros`, `mavros`)
rất dễ bị OOM‑kill. Tạo swap **4 GB** ngay:

```bash
sudo fallocate -l 4G /swapfile
sudo chmod 600 /swapfile
sudo mkswap /swapfile
sudo swapon /swapfile
grep -q '^/swapfile ' /etc/fstab || echo '/swapfile none swap sw 0 0' | sudo tee -a /etc/fstab

# chỉ dùng swap khi RAM gần cạn (mặc định 60 → 10)
echo 'vm.swappiness=10' | sudo tee /etc/sysctl.d/99-swappiness.conf
sudo sysctl -w vm.swappiness=10
```

Gói build cơ bản (cài luôn, các mục sau cần):

```bash
sudo apt install -y build-essential cmake git python3-pip python3-venv
```

**✔️ Nhận biết đã xong mục 2**

```bash
swapon --show          # 1 dòng: /swapfile file 4G ... (SIZE = 4G)
free -h                # dòng Swap:  total = 4.0Gi
cat /proc/sys/vm/swappiness   # 10
grep swap /etc/fstab   # có dòng: /swapfile none swap sw 0 0   → swap sống lại sau reboot
gcc --version && cmake --version   # in ra phiên bản, không "command not found"
```

Kết quả mong đợi của `swapon --show`:

```
NAME      TYPE  SIZE USED PRIO
/swapfile file    4G   0B   -2
```

---

## 3. Thêm kho ROS 2 apt (Jazzy)

Dùng gói `ros-apt-source` chính thức (thay cho cách cũ tự thêm key + `sources.list`):

```bash
export ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F\" '{print $4}')
curl -L -o /tmp/ros2-apt-source.deb \
  "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo $VERSION_CODENAME)_all.deb"
sudo apt install -y /tmp/ros2-apt-source.deb
sudo apt update && sudo apt upgrade -y
```

**✔️ Nhận biết đã xong mục 3**

```bash
ls /etc/apt/sources.list.d/ | grep ros            # có file ros2*.list / ros2*.sources
apt-cache policy ros-jazzy-ros-base | head -3     # có dòng "Candidate: <version>" (KHÔNG phải (none))
apt-get update 2>&1 | grep -i ros                 # dòng "Get:/Hit: ... packages.ros.org ..." không có "Err"
```

Kết quả sai điển hình:
- `Candidate: (none)` → repo chưa nạp, chạy lại `sudo apt update`.
- `sudo apt upgrade` / `apt install ros-jazzy-ros-base` báo **"held broken packages"** ở
  `liblz4-dev` / `libzstd-dev` → **chưa bật `noble-updates`**, quay lại [mục 1](#1-locale--cập-nhật-hệ-thống)
  chạy khối `sed` + `sudo apt full-upgrade -y` rồi mới cài tiếp.

---

## 4. ROS 2 Jazzy (bản base, không GUI) + công cụ dev

```bash
sudo apt install -y ros-jazzy-ros-base
sudo apt install -y \
  python3-colcon-common-extensions python3-rosdep python3-vcstool python3-argcomplete

# rosdep: nếu đã init lần trước sẽ báo "default sources list file already exists" — BỎ QUA, vô hại
sudo rosdep init 2>/dev/null || true
rosdep update            # chạy KHÔNG sudo; cảnh báo "pkg_resources is deprecated" vô hại

# nạp môi trường ROS mỗi lần mở shell — guard cho idempotent, KHÔNG thêm trùng dòng
grep -qxF 'source /opt/ros/jazzy/setup.bash' ~/.bashrc \
  || echo 'source /opt/ros/jazzy/setup.bash' >> ~/.bashrc
source /opt/ros/jazzy/setup.bash    # KHÔNG dùng `source ~/.bashrc` ở đây (xem cảnh báo dưới)
printenv ROS_DISTRO      # -> jazzy
```

> **Bẫy đã gặp trên máy này:**
> - `ros2 --version` **không còn** trong Jazzy → báo `error: unrecognized arguments: --version`.
>   Xem phiên bản bằng `apt show ros-jazzy-ros-base | grep -E 'Package|Version'`.
> - Nếu lỡ chạy `source ~/.bashrc` lúc `/opt/ros/jazzy/setup.bash` **chưa tồn tại** (ROS chưa cài
>   xong), lệnh `source` nuốt lỗi âm thầm; chạy lại `echo >>` sẽ khiến `~/.bashrc` có **2 dòng
>   `source` trùng**. Kiểm tra `grep -n 'opt/ros/jazzy' ~/.bashrc` — nếu > 1 dòng thì xoá dòng
>   thừa: `sed -i '<số_dòng_thừa>d' ~/.bashrc`.
> - `ros-base` **không kèm** `demo_nodes_cpp`/`demo_nodes_py`. Muốn test talker/listener:
>   `sudo apt install -y ros-jazzy-demo-nodes-cpp`.

**✔️ Nhận biết đã xong mục 4**

```bash
ls /opt/ros/jazzy/setup.bash           # file tồn tại
printenv ROS_DISTRO                     # jazzy
apt show ros-jazzy-ros-base 2>/dev/null | grep Version   # in version gói (KHÔNG dùng `ros2 --version` — Jazzy đã bỏ)
ros2 pkg list | wc -l                   # ~190–200 với ros-base headless (máy này: 194)
ros2 topic list                         # tối thiểu: /parameter_events và /rosout
ros2 doctor --report >/dev/null && echo "ros2 doctor OK"
colcon --help >/dev/null && echo "colcon OK"
rosdep --version                        # in ra số phiên bản
```

Kết quả sai điển hình: mở terminal mới mà `ros2` báo *command not found* → thiếu dòng
`source /opt/ros/jazzy/setup.bash` trong `~/.bashrc`, hoặc đang dùng `sh` thay vì `bash`.

---

## 5. MAVROS — cầu nối STM32H743 (MAVLink ⇄ ROS 2)

```bash
sudo apt install -y ros-jazzy-mavros ros-jazzy-mavros-extras
sudo /opt/ros/jazzy/lib/mavros/install_geographiclib_datasets.sh   # tải dữ liệu geoid, khá lâu
```

User `pc` **chưa** thuộc nhóm `dialout` (đã kiểm tra) → chưa mở được cổng serial:

```bash
sudo usermod -aG dialout pc
# BẮT BUỘC đăng xuất SSH rồi vào lại (hoặc `sudo reboot`) để quyền có hiệu lực
groups | tr ' ' '\n' | grep dialout   # xác nhận sau khi vào lại
```

Khi đã cắm FC (kiểm tra `ls /dev/serial/by-id/` hoặc `dmesg | tail` để biết là `ttyUSB0` hay `ttyACM0`):

```bash
ros2 launch mavros px4.launch fcu_url:=/dev/ttyACM0:921600   # sửa cổng + baud theo cấu hình H743
# terminal khác:
ros2 topic echo /mavros/state          # mong đợi: connected: true
```

> Dùng `apm.launch` thay `px4.launch` nếu firmware H743 là ArduPilot.

**✔️ Nhận biết đã xong mục 5**

```bash
# --- không cần FC ---
ros2 pkg list | grep -E '^mavros'          # mavros, mavros_extras, mavros_msgs
ros2 pkg prefix mavros                     # in ra /opt/ros/jazzy
geographiclib-get-geoids -h >/dev/null 2>&1; ls /usr/share/GeographicLib/geoids/ 2>/dev/null   # có file egm96-5.pgm ... → dataset đã cài
groups | tr ' ' '\n' | grep dialout        # in "dialout"  (sau khi đã đăng nhập lại)

# --- khi đã cắm FC ---
ls -l /dev/serial/by-id/                   # thấy thiết bị FC (vd usb-...STM32... hoặc ...ArduPilot...)
ros2 topic list | grep mavros              # có /mavros/state, /mavros/imu/data ...
ros2 topic echo /mavros/state --once       # connected: true   (quan trọng nhất)
ros2 topic hz /mavros/imu/data             # ~ vài chục Hz → IMU đang chảy về
```

Kết quả sai điển hình: `connected: false` → sai cổng/baud, chưa vào nhóm `dialout`, hoặc
dây TX/RX ngược. `permission denied '/dev/ttyACM0'` → chưa `usermod -aG dialout` + đăng nhập lại.

---

## 6. Camera OV9281 (CSI) + gói xử lý ảnh ROS

Repo này là bring‑up camera **OV9281** qua CSI, nên ưu tiên đường CSI. (Nếu sau này dùng webcam USB
thì xem 6C‑phương án B.)

### 6A. Nạp overlay + driver libcamera cho OV9281

Repo đã có sẵn script làm việc này — chạy nó thay vì gõ tay:

```bash
cd ~/ros2_ws        # đường dẫn repo trên máy này: /home/pc/ros2_ws
./scripts/install.sh
sudo reboot
```

`scripts/install.sh` tự nhận diện bản phân phối (đọc `/etc/os-release`):

| Bản phân phối | Gói camera cài qua apt |
|---|---|
| **Raspberry Pi OS** | `rpicam-apps`, `python3-picamera2`, `python3-opencv`, `i2c-tools`, `v4l-utils` |
| **Ubuntu trên Pi** (Ubuntu **không có** `rpicam-*` / `python3-picamera2` qua apt) | `libcamera-tools`, `libcamera-v4l2`, **`libcamera-ipa`**, `python3-libcamera`, `python3-opencv`, `i2c-tools`, `v4l-utils` |

> **`libcamera-ipa` bắt buộc trên Ubuntu** — chứa `ipa_rpi_vc4.so` + tuning `ov9281_mono.json`.
> Thiếu gói này: kernel vẫn nhận sensor (`i2cdetect` thấy `UU` ở `0x60`, `media-ctl` thấy
> `ov9281 10-0060`) nhưng `cam -l` rỗng kèm lỗi `Failed to load a suitable IPA library`.
> Sửa: `sudo apt install -y libcamera-ipa` (không cần reboot). Chi tiết: `docs/CAMERA.md` §4.5.

Sau đó (cho **mọi** bản phân phối trên Pi) script sao lưu `/boot/firmware/config.txt`,
đặt `camera_auto_detect=0` và thêm `dtoverlay=ov9281`. Nếu bước apt lỗi, script vẫn
chạy tiếp phần overlay và in cảnh báo ở cuối (trước đây `set -e` làm cả script dừng
tại lỗi apt nên overlay không bao giờ được bật — đây là lỗi mục 6A cũ).

Sau reboot, kiểm tra:

```bash
./scripts/check_camera.sh
# mong đợi: rpicam-hello --list-cameras (RPi OS) hoặc `cam -l` (Ubuntu) thấy 'ov9281';
#           i2cdetect -y 10 thấy 'UU' tại 0x60; lsmod thấy 'ov9282' + 'unicam'
```

> **KHÔNG** `pip install opencv-python` trên Pi — bản pip kéo numpy 2.x làm hỏng `picamera2`
> (build sẵn cho numpy 1.x của Ubuntu). Chỉ cài qua apt. `requirements.txt` của repo chỉ dùng
> cho máy **không phải** Pi.
>
> **Ubuntu**: nếu cần Picamera2 thì `pip install picamera2` (dựa trên `python3-libcamera`
> vừa cài), hoặc đọc camera trực tiếp qua OpenCV / GStreamer `libcamerasrc`, hoặc dùng
> `camera_ros` build từ source (mục 6C phương án A).

### 6B. Gói xử lý / truyền ảnh trong ROS 2

```bash
sudo apt install -y \
  ros-jazzy-cv-bridge \
  ros-jazzy-image-transport ros-jazzy-image-transport-plugins \
  ros-jazzy-image-proc \
  ros-jazzy-camera-calibration ros-jazzy-camera-calibration-parsers ros-jazzy-camera-info-manager
```

`camera-calibration*` bắt buộc để ArUco/AprilTag đo pose 3D chính xác (cần `camera_info` đúng).

### 6C. Driver ROS 2 xuất ảnh camera thành topic

**Phương án A — camera CSI/OV9281 đọc thẳng V4L2 bằng `v4l2_camera` (ĐÃ KIỂM CHỨNG, khuyến nghị).**

OV9281 là sensor **mono global-shutter** — không có ma trận Bayer nên **không cần ISP, không cần
debayer, không cần IPA của libcamera**. Đọc thẳng `/dev/video0` do `bcm2835-unicam` tạo ra vừa đơn
giản vừa tránh sạch các bug IPA của libcamera (xem "Phương án A-cũ" bên dưới).

```bash
sudo apt install -y ros-jazzy-v4l2-camera ros-jazzy-image-transport-plugins

# Một lệnh làm cả 2 bước: cấu hình V4L2 -> khởi động node.
# Phải chạy lại sau mỗi lần reboot (cấu hình V4L2 không tự giữ).
cd ~/ros2_ws && ./scripts/run_camera_node.sh --vblank 110 --exposure 800 --gain 120

# terminal khác:
ros2 topic hz /image_raw        # ~95 Hz ở 1280x800
ros2 run image_view image_view --ros-args -r image:=/image_raw
```

`run_camera_node.sh` gọi `camera_v4l2_setup.sh` trước, chỉ khởi động node nếu cấu hình thành công,
tự `source` ROS nếu chưa, và `exec` sang node để Ctrl+C ăn thẳng. Tham số: `--width/--height`,
`--exposure/--gain/--vblank`, `--setup-only`, và `-- <ros args>` để đẩy thêm tham số vào node.

Kết quả đo thực tế trên máy này (`ros2 topic hz /image_raw`, mono8):

| Độ phân giải | `vertical_blanking` | FPS đo được |
|---|---|---|
| 1280×800 | 3085 (sót lại từ libcamera) | 22.8 |
| 1280×800 | **110** (tối thiểu) | **~95** |
| 640×400 | **110** | **~246** |

> **Ba cái bẫy bắt buộc phải biết** (đây là lý do `scripts/camera_v4l2_setup.sh` tồn tại):
>
> 1. **`bcm2835-unicam` KHÔNG đổi độ sâu bit.** Sensor mặc định ở `Y10_1X10` (10-bit). Nếu ứng
>    dụng xin `GREY` (8-bit) mà subdev vẫn là `Y10_1X10` thì stream chạy, `ros2 topic hz` báo
>    30 Hz **nhưng mọi khung hình TOÀN SỐ 0**. Phải ép subdev sang `Y8_1X8` cho khớp:
>    `media-ctl -d /dev/media0 --set-v4l2 '"ov9281 10-0060":0[fmt:Y8_1X8/1280x800]'`
> 2. **Không có IPA thì không có auto-exposure.** Sensor giữ nguyên `exposure`/`analogue_gain`
>    của lần chạy trước (thường rất thấp) → ảnh đen. Phải đặt tay trên **subdev**, không phải
>    trên `/dev/video0`: `v4l2-ctl -d /dev/v4l-subdev0 --set-ctrl exposure=800,analogue_gain=120`
> 3. **`vertical_blanking` bóp FPS.** libcamera hay để lại giá trị rất lớn (đo được 3085) → chỉ
>    còn ~23 FPS thay vì ~95. Hạ về tối thiểu bằng `--vblank 110`. Lưu ý **thứ tự**: driver chốt
>    trần `exposure` theo `height + vblank`, nên phải hạ vblank **trước** rồi mới đặt exposure,
>    không thì exposure bị cắt im lặng (script đã làm đúng thứ tự và có cảnh báo).
>
> Cả ba đều **reset sau mỗi lần reboot**.
>
> Cách phân biệt "ảnh đen do sai format" với "ảnh đen do thiếu sáng": chụp 1 khung rồi xem
> thống kê. `min=0 max=0` tuyệt đối → sai format. `mean≈16, std<2` → đúng format nhưng không có
> ánh sáng (16 là black level của sensor). Có nội dung thật thì `std` lên vài chục.

**Phương án A-cũ — `camera_ros` qua libcamera: KHÔNG chạy được trên Ubuntu 24.04 + OV9281.**
Giữ lại đây để khỏi dò lại. Đã thử cả hai nhánh, **cả hai đều crash trong IPA của Raspberry Pi**:

| `camera_ros` | libcamera | Kết quả |
|---|---|---|
| 0.7.0 (`main`) | `ros-jazzy-libcamera` 0.7.1 — IPA chạy **isolated** qua IPC | `FATAL Serializer control_serializer.cpp:626 A list of V4L2 controls requires a ControlInfoMap` → worker chết → `Failed to call start: -110` |
| 0.5.2 | `libcamera-dev` hệ thống 0.2.0 — IPA chạy **in-process** | `FATAL default ipa_base.cpp:396 assertion "it != buffers_.end()" failed in prepareIsp()` |

Ghi chú thêm nếu sau này muốn thử lại:
- `camera_ros` ≥ 0.6.0 cần libcamera ≥ 0.3 (dùng `ControlList::MergePolicy`), mà Ubuntu 24.04 chỉ
  có 0.2.0 → build lỗi `'libcamera::ControlList::MergePolicy' has not been declared`. Tag mới nhất
  build được với 0.2.0 là **0.5.2**.
- `ros-jazzy-libcamera` và `libcamera-dev` **xung đột nhau**: `/opt/ros/jazzy/lib/libcamera.so.0.7`
  luôn thắng vì `/opt/ros/jazzy/lib` nằm trong `LD_LIBRARY_PATH`. Muốn build trên bản hệ thống thì
  phải `sudo apt remove ros-jazzy-libcamera` (không gói nào phụ thuộc nó).

**Phương án B — webcam USB (nếu KHÔNG dùng CSI):**

```bash
sudo apt install -y ros-jazzy-usb-cam ros-jazzy-v4l2-camera
v4l2-ctl --list-devices        # xem camera USB ra /dev/videoN nào
```

> Lưu ý: các `/dev/video10..31` sẵn có trên máy là node codec/ISP của Pi, **không phải** camera.
> Chỉ khi cắm camera thật (USB) hoặc nạp overlay (CSI) mới xuất hiện node camera thật.

**✔️ Nhận biết đã xong mục 6**

Phần cứng / overlay (sau reboot ở 6A):

```bash
grep -E 'camera_auto_detect|ov9281' /boot/firmware/config.txt   # camera_auto_detect=0  +  dtoverlay=ov9281
lsmod | grep -E 'ov9282|unicam'                                 # cả 2 dòng cùng có (Pi 4)
cam -l                                                          # (Ubuntu) liệt kê "ov9281"; RPi OS dùng rpicam-hello --list-cameras
sudo i2cdetect -y 10                                            # ô địa chỉ 60 hiện "UU" (driver đã chiếm)
media-ctl -p -d /dev/media0                                     # entity "ov9281 10-0060" nối vào "unicam-image"
```

Chuỗi ROS 2 (sau 6C phương án A):

```bash
./scripts/run_camera_node.sh --vblank 110 --exposure 800 --gain 120   # cấu hình + chạy node
ros2 topic hz /image_raw                                              # ~95 Hz ở 1280x800
```

Gói ROS xử lý ảnh (6B):

```bash
ros2 pkg list | grep -E 'cv_bridge|image_transport|image_proc|camera_calibration|camera_info_manager'
python3 -c "from cv_bridge import CvBridge; print('cv_bridge OK')"
```

Driver camera → topic (6C, phương án A):

```bash
ros2 pkg list | grep camera_ros            # có "camera_ros" → colcon build đã thành công
# chạy node ở 1 terminal, kiểm tra ở terminal khác:
ros2 topic list | grep image              # /camera/image_raw, /camera/camera_info
ros2 topic hz /camera/image_raw           # tần số ổn định (vd ~30–120 Hz tùy width/height/exposure)
ros2 topic echo /camera/camera_info --once  # width/height khớp tham số đã đặt
```

Kết quả sai điển hình: `No cameras available!` → overlay chưa nạp (chưa reboot, hoặc sửa nhầm
file `config.txt`). `colcon build` bị *Killed* → thiếu swap (quay lại mục 2) hoặc bỏ bớt
`--parallel-workers` xuống 1.

---

## 7. Định vị marker + hợp nhất cảm biến (EKF)

```bash
sudo apt install -y ros-jazzy-aruco-opencv
sudo apt install -y ros-jazzy-apriltag ros-jazzy-apriltag-ros
sudo apt install -y ros-jazzy-tf2-ros ros-jazzy-tf2-geometry-msgs
sudo apt install -y ros-jazzy-robot-localization
```

- `aruco-opencv` / `apriltag-ros`: phát hiện marker, publish pose 3D (điểm neo).
- `robot-localization`: node EKF hợp nhất vận tốc optical flow + pose marker + IMU (từ MAVROS)
  thành `Odometry` mượt, chống trôi.
- Optical flow: dùng thẳng `cv2.calcOpticalFlowPyrLK` / `Farneback` trong node `rclpy` tự viết,
  không cần gói ROS riêng.

**✔️ Nhận biết đã xong mục 7**

```bash
ros2 pkg list | grep -E 'aruco|apriltag|robot_localization|tf2_ros|tf2_geometry_msgs'
ros2 pkg executables aruco_opencv          # liệt kê node aruco_tracker ...
ros2 pkg executables robot_localization    # có ekf_node, ukf_node
ros2 run tf2_ros tf2_echo map base_link    # báo "Waiting for transform" là bình thường khi chưa có node phát TF
python3 -c "import cv2; print('opencv', cv2.__version__)"   # in phiên bản (dùng cho optical flow)
```

Kết quả mong đợi: mọi dòng `grep` đều trả về tên gói, `ros2 pkg executables` in ra node.
Không có gì in ra → gói chưa cài (chạy lại `apt install` tương ứng).

---

## 8. Workspace `ros2_ws` + build node tự viết

Nếu chưa tạo ở mục 6C:

```bash
mkdir -p ~/ros2_ws/src
```

Đặt các package tự viết (node camera publish `sensor_msgs/Image`, node optical flow, node PID,
file launch tổng) vào `~/ros2_ws/src/`. `src/camera_view.py` trong repo này là điểm khởi đầu để
viết node camera (logic mở camera dùng lại gần như nguyên vẹn).

```bash
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install --parallel-workers 2

echo "source ~/ros2_ws/install/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

RAM 4 GB: luôn `--parallel-workers 2`. Đã có swap (mục 2) nên build source ít bị OOM hơn;
nếu vẫn treo, thử `--parallel-workers 1` hoặc `MAKEFLAGS="-j1"`.

**✔️ Nhận biết đã xong mục 8**

```bash
colcon build ... 2>&1 | tail -3        # dòng cuối: "Summary: N packages finished" — KHÔNG có "failed"
ls ~/ros2_ws/install/setup.bash        # file tồn tại
cat ~/ros2_ws/log/latest_build/*/stdout_stderr.log 2>/dev/null | grep -i error   # rỗng là tốt
# mở terminal MỚI (đã source lại .bashrc):
ros2 pkg list | grep <ten_package_cua_ban>     # thấy package tự viết
ros2 launch <ten_package_cua_ban> <file>.launch.py --show-args   # đọc được launch file, không lỗi import
```

Kết quả sai điển hình: `Summary: ... 1 package failed` → mở log ở `~/ros2_ws/log/latest_build/`.
Build xong nhưng `ros2 pkg list` không thấy → quên `source ~/ros2_ws/install/setup.bash`.

---

## 9. Hiệu năng khi bay (Pi 4)

Máy đang để governor `ondemand`. Khi chạy pipeline thật (optical flow + ArUco/AprilTag đồng thời),
đặt về `performance` để tránh throttling do tiết kiệm điện:

```bash
sudo apt install -y cpufrequtils
echo 'GOVERNOR="performance"' | sudo tee /etc/default/cpufrequtils
sudo systemctl restart cpufrequtils
cat /sys/devices/system/cpu/cpu0/cpufreq/scaling_governor   # -> performance
```

- Bắt đầu ở 640×480 (hoặc 640×400 cho OV9281), 15–30 fps, rồi tăng dần.
- Governor `performance` chạy liên tục làm Pi 4 nóng → **bắt buộc heatsink + quạt**, nếu không
  Pi tự giảm xung khi quá nhiệt (nhiệt hiện tại nhàn rỗi ~58 °C).

**✔️ Nhận biết đã xong mục 9**

```bash
cat /sys/devices/system/cpu/cpu*/cpufreq/scaling_governor   # cả 4 dòng = performance
grep -c 1500000 /sys/devices/system/cpu/cpu*/cpufreq/scaling_cur_freq  # xung giữ ở mức tối đa (1.5GHz)
vcgencmd get_throttled 2>/dev/null   # throttled=0x0  → không bị giảm xung do nhiệt/nguồn
cat /sys/class/thermal/thermal_zone0/temp   # < 80000 (tức < 80 °C) khi tải nặng
```

Kết quả sai điển hình: `throttled=0x50000` hoặc nhiệt > 80 °C → tản nhiệt kém, cần thêm quạt
trước khi bay.

---

## 10. Kiểm tra tổng thể

Bảng "đã xong hay chưa" — chạy lần lượt, cột **Mong đợi** phải đúng thì mới coi là xong:

| # | Lệnh | Mong đợi |
|---|---|---|
| 1 | `locale \| grep LANG` | `LANG=en_US.UTF-8` |
| 2 | `swapon --show` | 1 dòng `/swapfile ... 4G` |
| 2 | `gcc --version` | in phiên bản (không "not found") |
| 3 | `apt-cache policy ros-jazzy-ros-base \| grep Candidate` | `Candidate:` có số, không `(none)` |
| 4 | `printenv ROS_DISTRO` | `jazzy` |
| 4 | `ros2 pkg list \| wc -l` | ~190–200 (ros-base headless; máy này 194) |
| 4 | `colcon --help >/dev/null && echo ok` | `ok` |
| 5 | `ros2 pkg list \| grep '^mavros'` | `mavros`, `mavros_extras`, `mavros_msgs` |
| 5 | `ls /usr/share/GeographicLib/geoids/` | có file `egm96-5.pgm` |
| 5 | `groups \| grep dialout` | có `dialout` |
| 6 | `lsmod \| grep -E 'ov9282\|unicam'` | cả 2 module (sau reboot 6A) |
| 6 | `rpicam-hello --list-cameras` | liệt kê `ov9281` |
| 6 | `python3 -c "from cv_bridge import CvBridge"` | không lỗi |
| 7 | `ros2 pkg list \| grep -E 'aruco\|apriltag\|robot_localization'` | đủ 3+ gói |
| 8 | `ls ~/ros2_ws/install/setup.bash` | file tồn tại |
| 8 | build log cuối | `Summary: ... 0 packages failed` |
| 9 | `cat /sys/devices/system/cpu/cpu0/cpufreq/scaling_governor` | `performance` (khi bay) |
| — | `vcgencmd get_throttled` | `throttled=0x0` |

Kiểm tra "sống" khi đã cắm đủ phần cứng (FC + camera), chạy các node rồi:

```bash
ros2 node list                            # thấy /mavros, /camera_node, (node tự viết) ...
ros2 topic echo /mavros/state --once      # connected: true
ros2 topic hz /camera/image_raw           # tần số ổn định, không tụt về 0
ros2 topic hz /mavros/imu/data            # IMU chảy đều
ros2 run tf2_tools view_frames && ls frames.pdf   # cây TF camera → drone → marker sinh ra được
```

Coi như **cài đặt hoàn tất** khi: bảng trên đúng hết, `/mavros/state` `connected: true`,
`/camera/image_raw` có tần số ổn định. Bước tiếp theo là viết node optical flow / PID và file
launch hợp nhất: camera → ArUco/AprilTag → robot_localization → PID → MAVROS.

---

## 11. (Tuỳ chọn) Tự khởi động khi Pi boot bằng systemd

`/etc/systemd/system/drone-ros2.service` (user trên máy này là `pc`):

```ini
[Unit]
Description=Drone ROS2 stack
After=network-online.target
Wants=network-online.target

[Service]
Type=simple
User=pc
ExecStart=/bin/bash -lc 'source /opt/ros/jazzy/setup.bash && source /home/pc/ros2_ws/install/setup.bash && ros2 launch <ten_package> drone.launch.py'
Restart=on-failure
RestartSec=3

[Install]
WantedBy=multi-user.target
```

```bash
sudo systemctl daemon-reload
sudo systemctl enable --now drone-ros2.service
journalctl -u drone-ros2.service -f
```

**✔️ Nhận biết đã xong mục 11**

```bash
systemctl is-enabled drone-ros2.service    # enabled  → sẽ tự chạy khi boot
systemctl is-active  drone-ros2.service    # active   → đang chạy
journalctl -u drone-ros2.service -b --no-pager | tail -20   # không lặp "Restart" liên tục
# thử thật: sudo reboot, đăng nhập lại rồi chạy 2 lệnh is-enabled/is-active ở trên
```

Kết quả sai điển hình: `activating (auto-restart)` lặp lại → sai đường dẫn `source` hoặc tên
package trong `ExecStart`; xem log bằng `journalctl -u drone-ros2.service -e`.

---

## Khác biệt so với `huong_dan_cai_dat_ros2_pi4.md`

| Điểm | File gốc | File này (theo máy thật) |
|---|---|---|
| Swap | Nêu ở mục 9, 2 GB, sau khi tạo workspace | Đưa lên **mục 2**, **4 GB** + `swappiness=10`, làm trước mọi build vì máy đang **0 swap** |
| Nhận biết đã xong | không có | Mỗi mục có khối **`✔️ Nhận biết đã xong`** (lệnh + kết quả mong đợi) + bảng tổng ở mục 10 |
| `dialout` | `usermod -aG dialout $USER` chung chung | Ghi rõ user `pc` **chưa** thuộc nhóm, kèm bước xác nhận |
| Camera | USB webcam là mặc định, CSI là “nếu cần” | **CSI/OV9281 là chính** (đúng mục tiêu repo): chạy `scripts/install.sh`, rồi `camera_ros` từ source; USB thành phương án B |
| `noble-updates` | không đề cập | Máy thiếu suite này — **bước bắt buộc** ở mục 1: thiếu thì `apt install ros-jazzy-ros-base` kẹt *"held broken packages"* do lệch `liblz4`/`libzstd` giữa `noble-security` và `noble` |
| Kiểm tra ROS | `ros2 --version`, `demo_nodes_cpp talker` | Jazzy đã bỏ `ros2 --version`; `demo_nodes_*` không nằm trong `ros-base` — thay bằng `apt show`, `ros2 doctor`, `ros2 topic list` |
| Nạp `.bashrc` | `echo >> ~/.bashrc` rồi `source ~/.bashrc` | Dùng guard `grep -qxF ... || echo >>` (idempotent) và `source /opt/ros/jazzy/setup.bash` trực tiếp — `source ~/.bashrc` khi file setup chưa có sẽ nuốt lỗi + gây dòng trùng |
| Locale | giả định chưa cấu hình | Xác nhận đang `C.UTF-8`, cần đổi |
| CPU governor | mục ghi chú cuối | Giữ nguyên nội dung, ghi rõ đang là `ondemand` |
| Cổng FC ví dụ | `/dev/ttyUSB0` | Gợi ý `/dev/ttyACM0` + cách tự dò (`/dev/serial/by-id/`), vì H743 hay ra `ACM` |
| rqt/rviz2 | liệt kê trong danh sách gói | Bỏ khỏi luồng cài (máy Server headless); cài trên máy trạm khi cần |
