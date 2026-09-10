# Runbook — khởi động các node trước khi test hệ thống

Quy trình bật hệ thống trên Pi 4 `pi4ubuntu` (Ubuntu 24.04 + ROS 2 Jazzy) theo đúng thứ tự,
kèm cách xác nhận từng bước đã đúng trước khi sang bước sau.

- Cài đặt lần đầu → `cai_dat_ros2_framework_pi4.md`
- Chi tiết camera → [`CAMERA.md`](CAMERA.md)
- File này chỉ trả lời: **"bật hệ thống lên để test thì gõ gì, theo thứ tự nào."**

Cập nhật: 2026-09-07

---

## 0. Trạng thái kiểm chứng

Không phải bước nào cũng đã chạy thật. Cột cuối nói rõ mức độ tin cậy:

| Node | Gói | Đã kiểm chứng? |
|---|---|---|
| Camera `v4l2_camera_node` | `ros-jazzy-v4l2-camera` | ✅ **Chạy thật, đo được 95 Hz @1280×800 mono8, ảnh có nội dung** |
| ArUco `aruco_tracker` | `ros-jazzy-aruco-opencv` | ⚠️ Node khởi động được và subscribe đúng topic; **chưa test phát hiện marker thật** (chưa hiệu chỉnh camera, chưa có marker in) |
| MAVROS | `ros-jazzy-mavros` | ⚠️ Gói đã cài, dataset geoid đã có, user đã thuộc nhóm `dialout`; **chưa cắm FC nên chưa test** |
| EKF `ekf_node` | `ros-jazzy-robot-localization` | ❌ Gói đã cài nhưng **chưa có file cấu hình YAML** — chưa chạy được |
| Optical flow | node tự viết | ❌ Chưa viết |

Nói cách khác: **hiện chỉ có tầng camera là dùng được ngay.** Các tầng trên còn phải làm thêm việc
(hiệu chỉnh camera, cắm FC, viết config EKF) — xem mục 9.

---

## 1. Hai việc BẮT BUỘC làm lại sau mỗi lần reboot

### a) Nạp môi trường ROS

`~/.bashrc` đã có sẵn dòng source, nên mở terminal mới là xong. Xác nhận:

```bash
echo $ROS_DISTRO        # phải in: jazzy
```

Nếu rỗng: `source /opt/ros/jazzy/setup.bash`

### b) Cấu hình đường V4L2 cho camera

**Đây là bước dễ quên nhất và hậu quả rất khó nhận ra.** Cấu hình sensor không tự giữ qua reboot.
`scripts/run_camera_node.sh` đã tự làm bước này nên bình thường không phải gõ riêng.

> **Cảnh báo:** nếu chạy thẳng `v4l2_camera_node` mà bỏ qua bước cấu hình thì
> `ros2 topic hz` **vẫn báo nhịp bình thường nhưng mọi khung hình toàn số 0**.
> `topic hz` không phát hiện được lỗi này — phải kiểm tra thống kê pixel (mục 7).

---

## 2. Thứ tự khởi động

Mỗi node một terminal. **Thứ tự quan trọng**: node sau phụ thuộc topic của node trước.

```
T1  camera        ->  /image_raw, /camera_info
T2  aruco_tracker ->  /aruco_detections, /tf        (cần T1)
T3  mavros        ->  /mavros/*                     (độc lập, cần FC cắm sẵn)
T4  ekf_node      ->  /odometry/filtered            (cần T2 + T3)
```

Bật lần lượt, **xác nhận từng cái xong mới sang cái tiếp theo**.

---

## 3. T1 — Camera *(bắt buộc, đã kiểm chứng)*

```bash
cd ~/ros2_ws
./scripts/run_camera_node.sh --vblank 110 --exposure 800 --gain 120
```

Tham số theo tình huống:

| Tình huống | Lệnh |
|---|---|
| Mặc định, độ phân giải đầy đủ | `--vblank 110 --exposure 800 --gain 120` |
| Cần FPS rất cao (optical flow) | `--width 640 --height 400 --vblank 110 --exposure 700 --gain 120` → ~246 Hz |
| Thiếu sáng / trong nhà tối | tăng `--gain` (tối đa 255), rồi mới tăng `--exposure` |
| Cần phơi sáng lâu hơn trần | tăng `--vblank` (đổi lại giảm FPS) |
| Chỉ cấu hình, không chạy node | thêm `--setup-only` |

**Xác nhận T1 (terminal khác):**

```bash
ros2 topic hz /image_raw     # ~95 Hz @1280x800, ~246 Hz @640x400
ros2 topic list | grep -E 'image_raw|camera_info'
```

Bỏ qua an toàn: `Unable to open camera calibration file [...unicam.yaml]` — bình thường khi chưa
hiệu chỉnh, node vẫn publish ảnh.

---

## 4. T2 — ArUco *(cần T1 + cần hiệu chỉnh camera để ra pose đúng)*

```bash
ros2 run aruco_opencv aruco_tracker_autostart --ros-args \
  -p cam_base_topic:=image_raw \
  -p marker_size:=0.10 \
  -p marker_dict:=4X4_50
```

- `cam_base_topic:=image_raw` **bắt buộc** — mặc định của node là `camera/image_raw`, không khớp
  với topic `/image_raw` mà `v4l2_camera` phát. Đặt như trên thì node subscribe đúng
  `/image_raw` + `/camera_info` (đã kiểm chứng).
- `marker_size` là **cạnh marker thật tính bằng mét** — đo bằng thước, sai số ở đây vào thẳng
  sai số khoảng cách.
- `marker_dict` phải khớp bộ từ điển lúc in marker. Tạo marker bằng:
  `ros2 run aruco_opencv create_marker`

**Xác nhận T2:**

```bash
ros2 topic echo /aruco_detections --once    # đưa marker vào khung hình rồi chạy
ros2 run rqt_image_view rqt_image_view /aruco_tracker/debug   # xem marker được khoanh
```

> ⚠️ **Chưa hiệu chỉnh camera thì pose 3D sẽ sai.** Node vẫn phát hiện được marker và đọc ID,
> nhưng `pose` (x/y/z tính bằng mét) không đáng tin vì thiếu ma trận nội tại `K`.
> Ống kính của module này là **fisheye góc siêu rộng** nên méo ở rìa ảnh rất nặng.
> Xem mục 9a.

---

## 5. T3 — MAVROS *(chưa test — chưa cắm FC)*

Cắm FC rồi tìm cổng:

```bash
ls -l /dev/serial/by-id/     # phải thấy thiết bị STM32/ArduPilot
dmesg | tail -5              # xem nó ra ttyACM0 hay ttyUSB0
```

```bash
ros2 launch mavros px4.launch fcu_url:=/dev/ttyACM0:921600
# firmware ArduPilot thì dùng apm.launch thay px4.launch
```

**Xác nhận T3:**

```bash
ros2 topic echo /mavros/state --once    # connected: true  <- quan trọng nhất
ros2 topic hz /mavros/imu/data          # vài chục Hz
```

| Lỗi | Nguyên nhân |
|---|---|
| `connected: false` | Sai cổng hoặc sai baud, hoặc dây TX/RX ngược |
| `permission denied '/dev/ttyACM0'` | Chưa vào nhóm `dialout` — user `pc` **đã** thuộc nhóm này, nếu vẫn lỗi thì đăng xuất/đăng nhập lại |

---

## 6. T4 — EKF *(chưa chạy được — thiếu file cấu hình)*

`ros-jazzy-robot-localization` đã cài nhưng `ekf_node` **bắt buộc phải có file YAML cấu hình**
khai báo nguồn dữ liệu (`odom0`, `imu0`, `pose0`…) và ma trận `*_config` chọn trường nào được dùng.
File này **chưa được viết** trong repo.

Khi đã có, chạy:

```bash
ros2 run robot_localization ekf_node --ros-args --params-file ~/ros2_ws/config/ekf.yaml
# xác nhận:
ros2 topic hz /odometry/filtered
```

---

## 7. Kiểm tra toàn hệ thống

**a) Sức khoẻ chung**

```bash
ros2 node list                # liệt kê node đang chạy
ros2 topic list               # liệt kê topic
ros2 run tf2_tools view_frames    # xuất cây TF ra PDF
```

**b) Kiểm tra ảnh có nội dung thật — `topic hz` KHÔNG phát hiện được lỗi ảnh rỗng**

```bash
python3 - <<'EOF'
import rclpy, numpy as np
from rclpy.node import Node
from sensor_msgs.msg import Image
class S(Node):
    def __init__(self):
        super().__init__('chk'); self.n=0
        self.create_subscription(Image,'/image_raw',self.cb,10)
    def cb(self,m):
        a=np.frombuffer(m.data,dtype=np.uint8)
        verdict = "OK - anh that" if a.std()>8 else ("LOI - TOAN SO 0" if a.max()==0 else "PHANG/TOI")
        print(f'{m.width}x{m.height} {m.encoding} | min={a.min()} max={a.max()} '
              f'mean={a.mean():.1f} std={a.std():.1f} -> {verdict}')
        self.n+=1
        if self.n>=3: raise SystemExit
rclpy.init()
try: rclpy.spin(S())
except SystemExit: pass
EOF
```

Cách đọc kết quả:

| Kết quả | Nghĩa |
|---|---|
| `std` vài chục, `max` gần 255 | ✅ Ảnh thật |
| `min=0 max=0` tuyệt đối | ❌ Sai format — chạy `./scripts/camera_v4l2_setup.sh` |
| `mean≈16`, `std<2` | ⚠️ Format đúng nhưng **không có ánh sáng** (16 là black level) — tháo nắp ống kính, bật đèn |

**c) Chẩn đoán tầng dưới khi có nghi ngờ**

```bash
./scripts/check_camera.sh     # overlay, I2C, module kernel, IPA, /dev/videoN
```

---

## 8. Dừng hệ thống

`Ctrl+C` ở từng terminal theo **thứ tự ngược** (T4 → T3 → T2 → T1).

Nếu node còn sót lại:

```bash
pgrep -af 'v4l2_camera_no[d]e'
kill $(pgrep -f 'v4l2_camera_no[d]e')
```

> **Đừng dùng `pkill -f v4l2_camera_node`.** Cờ `-f` khớp cả command line của chính shell đang
> chạy lệnh đó, nên nó tự giết mình (thoát với mã 144). Luôn dùng dạng ngoặc `no[d]e` như trên.

---

## 9. Việc còn phải làm trước khi test được toàn hệ thống

### a) Hiệu chỉnh camera *(chặn ArUco/AprilTag đo pose 3D)*

Không có ma trận nội tại `K` thì không tính được khoảng cách: `Z = fx × S / w`, thiếu `fx` là bế tắc.

Máy này **không có GUI** nên `ros2 run camera_calibration cameracalibrator` (bắt buộc mở cửa sổ đồ
hoạ) không chạy được. Dùng script không GUI của repo:

```bash
# 0) Sinh bàn cờ để in (A4 ngang, 10x7 ô, ô 25 mm)
./tools/calibrate_camera.py pattern --size 9x6 --square 0.025 --out chessboard.svg
#    In ở tỉ lệ 100% (KHÔNG "fit to page"), đo lại thước 100 mm in kèm,
#    rồi dán PHẲNG lên bìa cứng, giữ nguyên vành trắng quanh bàn cờ.

# DỪNG T1 trước — /dev/video0 chỉ cho một tiến trình mở
./scripts/camera_v4l2_setup.sh --vblank 110 --exposure 800 --gain 120

./tools/calibrate_camera.py capture --size 9x6 --square 0.025   # chụp bộ ảnh bàn cờ
./tools/calibrate_camera.py solve  --size 9x6 --square 0.025 --write-ros
```

`--size` đếm **góc bên trong**, không đếm ô (bàn cờ 10×7 ô → `9x6`). `--square` là cạnh ô **đo
thật sau khi in**, tính bằng mét — sai ở đây đi thẳng vào sai số khoảng cách.

Đưa bàn cờ khắp khung hình, **đặc biệt là 4 góc** (fisheye méo mạnh ở rìa) và nhớ **nghiêng**
30–45° — ảnh chụp bàn cờ song song với cảm biến gần như không cho thêm thông tin. Script in lưới
phủ 6×6 để bạn biết còn thiếu vùng nào. Chi tiết cách chuẩn bị bàn cờ: [`CAMERA.md`](CAMERA.md) §4.7‑A0.

`solve` in sai số tái chiếu (RMS): **< 0.5 px là tốt**, > 1.0 nên chụp lại. `--write-ros` ghi thẳng
ra `~/.ros/camera_info/unicam.yaml`. Khởi động lại T1 — dòng `Unable to open camera calibration
file` sẽ biến mất.

> Hiệu chỉnh **gắn với độ phân giải**. Calibrate ở 1280×800 rồi chạy 640×400 là sai —
> `fx`, `cx` co theo tỉ lệ. Calibrate ở đúng độ phân giải sẽ dùng khi bay.

Muốn dùng công cụ chuẩn của ROS thì phải `ssh -X` (phía Pi đã bật sẵn X11Forwarding và có `xauth`;
máy khách cần X server: XQuartz trên macOS, VcXsrv/MobaXterm trên Windows). Chi tiết cả 3 cách:
[`CAMERA.md`](CAMERA.md) §4.7.

### b) Cắm và kiểm tra FC (mục 5)

### c) Viết `config/ekf.yaml` cho `robot_localization` (mục 6)

### d) Viết node optical flow

Dùng `cv2.calcOpticalFlowPyrLK` / `Farneback` trong node `rclpy` tự viết.
`tools/camera_view.py` là điểm khởi đầu — logic mở camera dùng lại được gần như nguyên vẹn.

---

## 10. Bảng tra sự cố nhanh

| Hiện tượng | Xử lý |
|---|---|
| `ros2 topic hz` bình thường nhưng ảnh toàn số 0 | Chưa chạy `camera_v4l2_setup.sh` — subdev còn ở `Y10_1X10`. `CAMERA.md` §4.6 |
| Ảnh `mean≈16`, `std<2` | Đúng format nhưng thiếu sáng (16 = black level) |
| Chỉ đạt ~23 FPS thay vì ~95 | `vertical_blanking` sót lại từ libcamera — thêm `--vblank 110` |
| `--exposure` đặt cao nhưng bị cắt | Trần exposure = `height + vblank`. Tăng `--vblank` |
| `cam -l` rỗng, `Failed to load a suitable IPA library` | Thiếu gói `libcamera-ipa` |
| `aruco_tracker` không nhận ảnh | Quên `-p cam_base_topic:=image_raw` |
| ArUco phát hiện được marker nhưng pose vô lý | Chưa hiệu chỉnh camera (mục 9a) |
| `/mavros/state` báo `connected: false` | Sai cổng/baud, hoặc TX/RX ngược |
| `camera_ros` crash trong IPA | Không dùng `camera_ros` — repo này đi đường V4L2 trực tiếp. `CAMERA.md` §9 |
