# Hướng dẫn chi tiết cách xây dựng từng node ROS 2 trên Raspberry Pi 4

*Tài liệu triển khai — bổ sung cho `thiet_ke_kien_truc_node_ros2.md` (kiến trúc tổng thể, "node nào làm việc gì và tại sao"), `huong_dan_cai_dat_ros2_pi4.md` (cài hệ điều hành + gói apt) và `thiet_ke_cau_truc_du_lieu_he_thong_drone.md` (struct dùng chung ba tầng FC/Pi4/GCS). Tài liệu này trả lời câu hỏi tiếp theo: **với từng node, cụ thể cần cấu hình gì, viết gì, và kiểm thử thế nào** — cập nhật 08/09/2026, dùng ROS 2 Jazzy Jalisco trên Ubuntu Server 24.04.*

Mỗi mục node dưới đây theo cùng một khuôn: mục đích, dùng gói có sẵn hay tự viết, lệnh cài đặt, bảng input/output kèm loại message và QoS khuyến nghị, tham số cấu hình quan trọng, khung code tối thiểu (với node tự viết), cách kiểm thử độc lập trước khi ráp vào toàn hệ thống, và lưu ý riêng khi chạy trên CPU 4 nhân Cortex-A72 của Pi 4.

## 0. Quy ước dùng chung trước khi bắt đầu

Tất cả node tự viết đặt trong workspace `~/ros2_ws/src/`, chia theo 6 package đã chốt trong tài liệu kiến trúc (`drone_perception`, `drone_estimation`, `drone_control`, `drone_mission`, `drone_comms`, `drone_safety`) cộng `drone_interfaces` (message/service tuỳ biến) và `drone_bringup` (launch + tham số). Message tuỳ biến nên ánh xạ đúng tên trường với struct C đã thiết kế phía FC để log/so sánh giữa ba tầng không bị lệch tên — ví dụ `MissionState.msg` dùng đúng các hằng số `IDLE, TAKEOFF, ENROUTE, MARKER_SEARCH, PRECISION_LAND, ACTUATE_GRIPPER, RETRY_LOITER, RTH, EMERGENCY_LAND, MISSION_COMPLETE, FAILSAFE` như `mission_state_e` trong tài liệu cấu trúc dữ liệu.

QoS: các topic tần số cao và "chỉ cần giá trị mới nhất" (ảnh, IMU, odometry, setpoint) nên dùng `Best Effort` + `depth=1` để không tích luỹ độ trễ khi Pi 4 quá tải một nhịp; các topic sự kiện rời rạc và quan trọng (lệnh, failsafe, ACK, trạng thái nhiệm vụ) dùng `Reliable` + `depth ≥ 10` để không mất gói. Toàn bộ node tự viết nên nhận tham số qua file YAML nạp trong launch (không hard-code), vì nhiều giá trị (kích thước marker, ngưỡng failsafe, hệ số PID) cần chỉnh tại hiện trường mà không build lại.

Kiểm thử node tự viết theo ba bước cố định trước khi bay thật: (1) chạy độc lập node đó với dữ liệu giả lập/ghi sẵn (`ros2 bag play`) và `ros2 topic echo` để xem output đúng định dạng; (2) đo tần số thực tế bằng `ros2 topic hz <topic>` và độ trễ bằng `ros2 topic delay <topic>` — so với tần số tham khảo trong bảng ở mục 5 tài liệu kiến trúc; (3) chạy `colcon test` hoặc ít nhất một script `pytest` cho phần logic thuần (không phụ thuộc ROS) như tính PID, tính optical flow, máy trạng thái — tách phần logic ra hàm thuần để test được mà không cần khởi động cả node.

## 1. Lớp Cảm nhận (`drone_perception`)

### 1.1 `camera_node`

**Mục đích**: nguồn ảnh duy nhất cho toàn hệ thống — mọi lỗi hiệu chỉnh ở đây sẽ lan ra sai số khoảng cách ở `marker_detector_node` và sai vận tốc ở `optical_flow_node`.

**Dùng gói có sẵn**: `ros-jazzy-usb-cam` (webcam USB, khuyến nghị vì đơn giản nhất trên Pi 4) hoặc `ros-jazzy-v4l2-camera`; nếu bắt buộc dùng Pi Camera Module (CSI) thì dùng `camera_ros` (build từ source, xem mục 9 của `huong_dan_cai_dat_ros2_pi4.md`).

**Cài đặt**:
```bash
sudo apt install -y ros-jazzy-usb-cam v4l-utils
v4l2-ctl --list-devices          # xác định /dev/videoX đúng camera
v4l2-ctl -d /dev/video0 --list-formats-ext   # xem độ phân giải/fps camera hỗ trợ
```

**Output**:

| Topic | Kiểu | QoS | Ghi chú |
|---|---|---|---|
| `/camera/image_raw` | `sensor_msgs/Image` | Best Effort, depth 1 | ảnh thô, chưa rectify |
| `/camera/camera_info` | `sensor_msgs/CameraInfo` | Best Effort, depth 1 | **phải cùng `header.stamp`** với image_raw tương ứng |

**Tham số quan trọng** (file `usb_cam_params.yaml`): `video_device` (`/dev/video0`), `image_width`/`image_height` (bắt đầu **640×480**, đúng khuyến nghị hiệu năng Pi 4), `framerate` (bắt đầu **15 fps**, không đặt 30 fps ngay), `pixel_format` (`yuyv` hoặc `mjpeg2rgb` tuỳ camera — `mjpeg` giảm tải USB bus nhưng tốn CPU giải nén), `camera_info_url` (đường dẫn file `.yaml` xuất ra từ bước hiệu chỉnh bên dưới).

**Bước bắt buộc — hiệu chỉnh camera trước khi dùng cho marker**:
```bash
ros2 run camera_calibration cameracalibrator \
  --size 8x6 --square 0.024 \
  --ros-args -r image:=/camera/image_raw -p camera:=/camera
```
In bàn cờ checkerboard 8×6 ô vuông 24 mm (hoặc kích thước tự chọn, chỉnh lại tham số), di chuyển bàn cờ trước camera ở nhiều góc/khoảng cách/vùng khung hình đến khi nút "CALIBRATE" sáng, bấm "SAVE" — file `camera_info.yaml` sinh ra nạp vào `camera_info_url` của `camera_node`. **Bỏ qua bước này khiến mọi phép đo khoảng cách marker sai lệch hệ thống, không phải nhiễu ngẫu nhiên** — vì `marker_detector_node` dùng thẳng ma trận nội tại này để giải PnP.

**Kiểm thử độc lập**: `ros2 run rqt_image_view rqt_image_view` (qua SSH X11 forwarding hoặc từ máy trạm khác trên cùng mạng ROS_DOMAIN_ID) để xem ảnh có rõ/đúng hướng không; `ros2 topic hz /camera/image_raw` phải ra đúng fps đã đặt — nếu thấp hơn nhiều, hạ độ phân giải hoặc đổi `pixel_format`.

### 1.2 `image_rectify`

**Mục đích**: khử méo ống kính (distortion) trước khi đưa vào phát hiện marker — marker ở rìa khung hình bị méo nhiều nhất nên nếu bỏ qua bước này, marker ở rìa sẽ cho pose sai nhiều hơn ở giữa.

**Dùng gói có sẵn**: `ros-jazzy-image-proc`, node `rectify` (hoặc component trong container `image_proc`).

**Cài đặt**: đã có trong danh sách gói xử lý ảnh của `huong_dan_cai_dat_ros2_pi4.md` (`ros-jazzy-image-proc`).

**Input/Output**:

| Topic | Kiểu | Hướng |
|---|---|---|
| `/camera/image_raw`, `/camera/camera_info` | `Image`, `CameraInfo` | vào |
| `/camera/image_rect` | `Image` | ra — dùng thay `image_raw` cho mọi node phía sau |

**Cấu hình**: không cần code, chỉ khai báo trong launch file:
```python
Node(package='image_proc', executable='rectify_node', name='image_rectify',
     remappings=[('image', '/camera/image_raw'),
                 ('camera_info', '/camera/camera_info'),
                 ('image_rect', '/camera/image_rect')])
```

**Lưu ý hiệu năng Pi 4**: rectify chạy trên CPU tốn thêm một khung ảnh full-size mỗi frame — nếu CPU đã căng ở bước sau (ArUco + optical flow), cân nhắc chạy rectify ở `mono8` thay vì `color` nếu marker detector không cần màu, giảm băng thông xử lý đáng kể.

### 1.3 `marker_detector_node`

**Mục đích**: nguồn định vị tuyệt đối tại bãi đáp — thay thế hoàn toàn việc tự viết PnP cho QR code.

**Dùng gói có sẵn**: `ros-jazzy-aruco-opencv` (dễ in, dễ cấu hình, đủ tốt cho phần lớn trường hợp) hoặc `ros-jazzy-apriltag-ros` (chính xác góc tốt hơn, bù được xoay/nghiêng lớn hơn — ưu tiên nếu drone hạ cánh ở góc nghiêng lớn hoặc cần độ chính xác cao).

**Cài đặt**:
```bash
sudo apt install -y ros-jazzy-aruco-opencv ros-jazzy-tf2-ros ros-jazzy-tf2-geometry-msgs
# hoặc: sudo apt install -y ros-jazzy-apriltag ros-jazzy-apriltag-ros
```

**Input/Output**:

| Topic | Kiểu | Hướng |
|---|---|---|
| `/camera/image_rect`, `/camera/camera_info` | `Image`, `CameraInfo` | vào |
| `/aruco/poses` (hoặc `/apriltag/detections`) | `PoseArray` / `aruco_opencv_msgs/ArucoDetection` / `apriltag_msgs/AprilTagDetectionArray` | ra — pose 3D từng marker + ID |
| `tf2`: `camera_frame → marker_<id>` | transform | ra |
| `/marker/tracking_quality` (**tự thêm**, không có sẵn trong gói) | `drone_interfaces/MarkerQuality` | ra — ID đang bám, khoảng cách, sai số reprojection |

**Tham số quan trọng**: `marker_size` (mét, **phải đúng kích thước in thực tế** — sai kích thước làm sai tỉ lệ khoảng cách theo đúng tỉ lệ sai đó), `dictionary`/`family` (ví dụ `DICT_4X4_50` cho ArUco hoặc `tag36h11` cho AprilTag — cố định một loại cho toàn bộ bãi đáp), `image_transport` (`raw` hoặc `compressed` tuỳ băng thông), mỗi bãi đáp gán một ID marker riêng để `mission_manager_node` xác thực đúng điểm (tương đương vai trò `expected_qr` trong `mission_waypoint_t`).

**Node cần tự thêm**: gói `aruco_opencv`/`apriltag_ros` không tự publish "chất lượng bám" gộp sẵn — cần một node nhỏ (`marker_quality_publisher.py`, có thể gộp luôn vào launch cùng gói) subscribe kết quả detection thô và publish tóm tắt cho `failsafe_monitor_node`, tránh để node an toàn phải tự parse `ArucoDetection` chi tiết:

```python
class MarkerQualityPublisher(Node):
    def __init__(self):
        super().__init__('marker_quality_publisher')
        self.declare_parameter('expected_marker_id', -1)  # -1 = chấp nhận mọi ID
        self.sub = self.create_subscription(ArucoDetection, '/aruco/detections', self.cb, 10)
        self.pub = self.create_publisher(MarkerQuality, '/marker/tracking_quality', 10)
        self.last_seen = self.get_clock().now()

    def cb(self, msg):
        expected = self.get_parameter('expected_marker_id').value
        for m in msg.markers:
            if expected == -1 or m.marker_id == expected:
                q = MarkerQuality()
                q.marker_id = m.marker_id
                q.distance_m = m.pose.position.z
                q.reprojection_error = m.reprojection_error
                q.stamp = self.get_clock().now().to_msg()
                self.pub.publish(q)
                self.last_seen = self.get_clock().now()
```

**Kiểm thử độc lập**: in marker đúng dictionary/kích thước đã cấu hình, đặt trước camera, chạy `ros2 topic echo /aruco/poses`, kiểm tra `pose.position.z` (khoảng cách) đúng với khoảng cách đo tay bằng thước — sai lệch quá 5–10% thường là do sai `marker_size` hoặc thiếu hiệu chỉnh camera ở bước 1.1. Xem `tf2` bằng `ros2 run tf2_tools view_frames` để xác nhận cây transform đúng hướng.

### 1.4 `optical_flow_node` — tự viết

**Mục đích**: đo vận tốc trôi ngang giữa hai lần thấy marker (hoặc khi hoàn toàn không thấy marker) — bù cho khoảng trống của `marker_detector_node`. Đây là node có khả năng cao nhất gây lỗi position-hold nếu làm ẩu, đúng như nghi vấn trong `poshold_drift_analysis*.png`.

**Package**: `drone_perception`, ngôn ngữ `rclpy` + `opencv-python` (đã có `python3-opencv` từ bước cài đặt).

**Input/Output**:

| Topic | Kiểu | Hướng |
|---|---|---|
| `/camera/image_raw` (ảnh xám, không cần rectify — sai số méo ảnh nhỏ so với sai số optical flow) | `Image` | vào |
| `/mavros/distance_sensor/...` hoặc `/mavros/global_position/rel_alt` | độ cao hiện tại | vào — cần để quy đổi pixel → mét |
| `/optical_flow/velocity` | `geometry_msgs/TwistWithCovarianceStamped` | ra |
| `/optical_flow/quality` | `std_msgs/UInt8` hoặc gộp vào covariance (xem dưới) | ra |

**Thuật toán khung** (Lucas-Kanade thưa — nhẹ CPU hơn Farneback dày, nên bắt đầu ở đây):
```python
import cv2, numpy as np

class OpticalFlowEstimator:
    def __init__(self, focal_px):
        self.focal_px = focal_px
        self.prev_gray = None
        self.prev_pts = None

    def process(self, gray, dt, altitude_m):
        if self.prev_gray is None or self.prev_pts is None or len(self.prev_pts) < 15:
            self.prev_pts = cv2.goodFeaturesToTrack(gray, maxCorners=100, qualityLevel=0.3, minDistance=7)
            self.prev_gray = gray
            return None, 0  # chưa đủ đặc trưng, chưa có kết quả

        next_pts, status, err = cv2.calcOpticalFlowPyrLK(self.prev_gray, gray, self.prev_pts, None)
        good_new = next_pts[status == 1]
        good_old = self.prev_pts[status == 1]
        n_tracked = len(good_new)

        if n_tracked < 8:                      # quá ít đặc trưng theo dõi được -> không tin kết quả
            self.prev_gray, self.prev_pts = gray, None
            return None, n_tracked

        flow_px = np.median(good_new - good_old, axis=0)     # median chống nhiễu tốt hơn mean
        vel_mps = (flow_px * altitude_m / self.focal_px) / dt  # quy đổi pixel/s -> m/s theo độ cao

        self.prev_gray, self.prev_pts = gray, good_new.reshape(-1, 1, 2)
        return vel_mps, n_tracked
```

Node bọc quanh lớp này phải: (1) chuyển ảnh sang xám bằng `cv_bridge` + `cv2.cvtColor`; (2) gọi lại `goodFeaturesToTrack` định kỳ (ví dụ mỗi 1–2 giây hoặc khi `n_tracked` tụt dưới ngưỡng) để không theo dõi mãi các điểm cũ đã trôi ra khỏi khung hình; (3) đặt **hiệp phương sai (covariance) của `TwistWithCovarianceStamped` tỉ lệ nghịch với `n_tracked`** — đây là cách "báo chất lượng thấp" tự nhiên nhất trong ROS 2 vì `state_estimator_node` (robot_localization) đọc thẳng trường này để tự động tin ít hơn vào phép đo yếu, không cần thêm topic cờ riêng; (4) nếu `n_tracked` dưới một ngưỡng cứng (ví dụ 8), **không publish gì cả** thay vì publish giá trị nhiễu — `robot_localization` xử lý việc thiếu phép đo tốt hơn nhiều so với phép đo sai mà covariance khai thấp.

**Tham số quan trọng**: `max_corners`, `quality_level`, `min_distance` (của `goodFeaturesToTrack`), `min_tracked_features` (ngưỡng dừng publish), `focal_length_px` (lấy từ `camera_info.K[0]` đã hiệu chỉnh ở bước 1.1, **không** đo tay), `process_every_n_frames` (bỏ bớt khung hình đầu vào nếu CPU không kịp — ví dụ tính flow trên 1/2 số khung ảnh camera gửi ra).

**Kiểm thử độc lập**: giữ camera cố định — vận tốc ra phải ~0 (kiểm tra nhiễu nền); di chuyển camera một khoảng đã đo bằng thước ở độ cao cố định — tích phân vận tốc theo thời gian phải khớp gần đúng quãng đường đã đo; kiểm tra `ros2 topic hz /optical_flow/velocity` và CPU (`htop`) khi chạy đồng thời với `marker_detector_node` — đây là tổ hợp tải nặng nhất trên Pi 4, nên đo trước khi cho bay.

**Lưu ý hiệu năng Pi 4**: Lucas-Kanade thưa nhẹ hơn Farneback dày đáng kể — chỉ chuyển sang Farneback nếu độ chính xác Lucas-Kanade không đủ và đã hết cách tối ưu khác (giảm fps, giảm độ phân giải, giảm `max_corners`).

## 2. Lớp Ước lượng (`drone_estimation`)

### 2.1 `state_estimator_node`

**Mục đích**: hợp nhất ba nguồn cảm biến lệch nhau về đặc tính (tuyệt đối/tương đối, tần số, độ trễ) thành một `Odometry` mượt, chống trôi.

**Dùng gói có sẵn**: `ros-jazzy-robot-localization`, node `ekf_node` — không tự viết Kalman filter.

**Input**: `/mavros/imu/data` (`sensor_msgs/Imu`, tần số cao), `/optical_flow/velocity` (`TwistWithCovarianceStamped`), `/aruco/poses` chuyển đổi qua `tf2` thành `PoseWithCovarianceStamped` trong khung `odom` (cần một node nhỏ hoặc `robot_localization` tự lấy qua `tf2` nếu cấu hình đúng frame).

**Output**: `/odometry/filtered` (`nav_msgs/Odometry`), `tf2`: `odom → base_link`.

**File cấu hình mẫu** (`ekf.yaml`) — phần quan trọng nhất là khai đúng "nguồn nào đóng góp trường nào":
```yaml
ekf_filter_node:
  ros__parameters:
    frequency: 30.0
    two_d_mode: false
    odom_frame: odom
    base_link_frame: base_link
    world_frame: odom

    imu0: /mavros/imu/data
    imu0_config: [false, false, false,   # x, y, z position
                  true,  true,  true,    # roll, pitch, yaw
                  false, false, false,   # vx, vy, vz
                  true,  true,  true,    # vroll, vpitch, vyaw
                  true,  true,  true]    # ax, ay, az
    imu0_differential: false

    twist0: /optical_flow/velocity
    twist0_config: [false, false, false,
                    false, false, false,
                    true,  true,  false,  # chỉ tin vx, vy ngang - KHÔNG dùng vz từ optical flow
                    false, false, false,
                    false, false, false]

    pose0: /marker/pose_odom_frame
    pose0_config: [true, true, true,     # marker cho vị trí tuyệt đối x,y,z khi có
                   false, false, false,
                   false, false, false,
                   false, false, false,
                   false, false, false]
    pose0_differential: false
    pose0_rejection_threshold: 2.0        # loại bỏ pose marker nếu nhảy vọt bất thường (Mahalanobis distance)
```

**Nguyên tắc chỉnh hiệp phương sai**: marker cho vị trí tuyệt đối nhưng chỉ có khi nhìn thấy — hiệp phương sai thấp (tin nhiều) nhưng thưa; optical flow cho vận tốc liên tục nhưng trôi tích luỹ theo thời gian — hiệp phương sai nên tăng dần nếu node 1.4 không tự cập nhật động (tốt nhất là để `optical_flow_node` tự động tăng covariance khi `n_tracked` giảm, như đã thiết kế ở mục 1.4, thay vì đặt cố định ở đây).

**Kiểm thử độc lập**: bắt đầu chỉ với IMU (không marker, không optical flow) — kiểm tra `/odometry/filtered` không phát nan/inf; thêm dần từng nguồn một, mỗi lần thêm kiểm tra `ros2 topic echo /odometry/filtered` không có bước nhảy đột ngột khi nguồn mới bắt đầu/ngừng publish; theo dõi trường tự thêm tương ứng `ekf_state_t.healthy` (dựa vào `P_diag` — nếu phương sai vị trí tăng liên tục không hội tụ, đặt cờ `healthy=false`) để `failsafe_monitor_node` dùng — `robot_localization` không tự có khái niệm "healthy" nhị phân này, cần một node nhỏ đọc `diagnostics` của EKF hoặc tự tính từ topic `/diagnostics` mà `robot_localization` xuất ra.

## 3. Lớp Điều khiển (`drone_control`)

### 3.1 `landing_target_bridge_node` — tự viết

**Mục đích**: chỉ node này được phép nói "đang bám marker đúng ID mong đợi" — tách trách nhiệm xác thực ID khỏi `marker_detector_node` (vốn chỉ báo cáo mọi ID thấy được) và khỏi `mission_manager_node` (vốn không nên tự parse pose thô).

**Package**: `drone_control`, `rclpy`.

**Input/Output**:

| Topic | Kiểu | Hướng |
|---|---|---|
| `/aruco/poses` (hoặc `/marker/tracking_quality`) | — | vào |
| `/mission/expected_marker_id` | `std_msgs/Int32` (từ `mission_manager_node`) | vào |
| `landing_target` (plugin `mavros-extras`) hoặc `/mavros/setpoint_raw/local` | `mavros_msgs/LandingTarget` / `PositionTarget` | ra |
| `/landing_target/lost` | `std_msgs/Bool` | ra — báo `failsafe_monitor_node` khi mất bám quá ngưỡng |

**Khung logic**:
```python
def on_marker(self, msg):
    for m in msg.markers:
        if m.marker_id == self.expected_id:
            self.last_seen_time = self.get_clock().now()
            lt = LandingTarget()
            lt.header.stamp = msg.header.stamp
            lt.target_num = m.marker_id
            # pose tương đối camera->marker; frame_id cần khớp cấu hình khung của FC
            lt.pose = transform_to_body_frame(m.pose, self.camera_to_body_tf)
            self.pub_landing_target.publish(lt)
            return
    # không thấy marker mong đợi trong lần cập nhật này -> không publish, không nội suy

def timer_check_timeout(self):
    if (self.get_clock().now() - self.last_seen_time) > self.timeout_duration:
        self.pub_lost.publish(Bool(data=True))
```

**Tham số quan trọng**: `timeout_s` (ví dụ 0.5–1.0s tuỳ tốc độ hạ cánh — quá dài làm FC bám vào vị trí cũ không còn đúng, quá ngắn gây báo mất bám giả khi camera rung một khung hình), `publish_rate_hz` (20–30 Hz trong pha bám, khớp bảng tần suất ở tài liệu kiến trúc), transform tĩnh `camera_to_body_tf` (đo/cấu hình một lần theo vị trí lắp camera thực tế trên khung drone — sai transform này làm marker "thấy đúng nhưng bay lệch tâm").

**Kiểm thử độc lập**: giả lập bằng cách di chuyển marker bằng tay trước camera đứng yên (không cần bay), publish `landing_target` và `ros2 topic echo` — kiểm tra pose đổi dấu/hướng đúng khi di chuyển marker sang trái/phải/gần/xa; cố tình che marker đúng `timeout_s` để xác nhận `/landing_target/lost` bật đúng thời điểm, không sớm/muộn.

### 3.2 `position_controller_node` — tự viết

**Mục đích**: vòng PID vị trí/vận tốc cấp companion computer — **khác tầng** với cascade PID góc/tốc độ góc đã chạy sẵn trên STM32H743 (tài liệu `phan_tich_thiet_ke_he_thong_van_chuyen.md`/firmware FC); node này chỉ ra lệnh cấp "muốn ở đâu/muốn bay nhanh cỡ nào", FC vẫn tự lo giữ góc nghiêng ổn định.

**Package**: `drone_control`, `rclpy`.

**Input/Output**:

| Topic | Kiểu | Hướng |
|---|---|---|
| `/odometry/filtered` | `nav_msgs/Odometry` | vào |
| `/mission/setpoint` (từ `mission_manager_node`) hoặc `landing_target` (khi đang hạ cánh) | — | vào |
| `/mavros/setpoint_raw/local` | `mavros_msgs/PositionTarget` | ra |

**Khung PID** (dùng chung quy ước tên trường với `pid_gains_t` phía FC để dễ đối chiếu khi tune):
```python
class PID:
    def __init__(self, kp, ki, kd, i_limit, out_limit):
        self.kp, self.ki, self.kd = kp, ki, kd
        self.i_limit, self.out_limit = i_limit, out_limit
        self.integral = 0.0
        self.prev_error = 0.0

    def update(self, error, dt):
        self.integral = clamp(self.integral + error * dt, -self.i_limit, self.i_limit)  # anti-windup
        derivative = (error - self.prev_error) / dt if dt > 0 else 0.0
        self.prev_error = error
        output = self.kp * error + self.ki * self.integral + self.kd * derivative
        return clamp(output, -self.out_limit, self.out_limit)
```

**Yêu cầu bắt buộc — chuyển setpoint mượt**: khi `mission_manager_node` đổi nguồn setpoint (waypoint hành trình ↔ landing target), **không được reset tích phân PID về 0 đột ngột và không được nhảy setpoint tức thời** — dùng một bước nội suy ngắn (ví dụ ramp trong 0.5–1s) hoặc giữ nguyên tích phân nếu sai số trước/sau chuyển đổi không lớn, để tránh giật máy bay đúng như yêu cầu đã ghi trong tài liệu kiến trúc.

**Tham số quan trọng**: `kp/ki/kd` riêng cho từng trục (x, y, z) và có thể riêng cho từng pha (hành trình vs. hạ cánh chính xác — pha hạ cánh thường cần đáp ứng nhanh hơn, sai số nhỏ hơn được chấp nhận dao động nhẹ), `i_limit`, `out_limit` (giới hạn vận tốc/góc nghiêng đầu ra để không vượt an toàn cơ khí).

**Kiểm thử độc lập**: **không bao giờ tune PID lần đầu trên drone thật** — dùng mô phỏng Gazebo đã thiết kế trong `thiet_ke_mo_phong_gazebo_ros2.md` để chỉnh gain trước, chỉ tinh chỉnh nhỏ trên phần cứng thật sau khi mô phỏng ổn định; khi bay thật, bắt đầu buộc dây/hoặc bay thấp trong lồng an toàn, tăng dần `kp` từ 0 đến khi có dao động nhẹ rồi lùi lại ~30-50%.

## 4. Lớp Hành vi Nhiệm vụ (`drone_mission`)

### 4.1 `mission_manager_node` — tự viết

**Mục đích**: máy trạng thái trung tâm, hiện thực hoá đúng `mission_state_e` (IDLE, TAKEOFF, ENROUTE, QR_SEARCH/MARKER_SEARCH, PRECISION_LAND, ACTUATE_GRIPPER, RETRY_LOITER, RTH, EMERGENCY_LAND, MISSION_COMPLETE, FAILSAFE).

**Package**: `drone_mission`, `rclpy` + `smach` (đơn giản, đủ dùng cho FSM tuyến tính) hoặc `py_trees`/`BehaviorTree.CPP` (nếu cần hành vi phức tạp hơn dạng cây, dễ mở rộng sau này).

**Input/Output chính**:

| Topic/Service | Kiểu | Hướng |
|---|---|---|
| `/mission/plan` (từ `gcs_link_node`) | `drone_interfaces/MissionPlan` | vào |
| `/mavros/state`, `/mavros/battery` | chuẩn MAVROS | vào |
| `/landing_target/lost`, `/marker/tracking_quality` | — | vào |
| `/gripper/status` | `drone_interfaces/GripperStatus` | vào |
| `/failsafe_event` | `drone_interfaces/FailsafeEvent` | vào |
| service `fc_command_bridge/*` (arm, takeoff, goto, hold, precision_land, land_now, rth) | tuỳ định nghĩa `.srv` | ra |
| `/gripper/command` | `drone_interfaces/GripperCommand` | ra |
| `/mission/state` | `drone_interfaces/MissionState` | ra — cho `telemetry_aggregator_node` và `mission_logger_node` |

**Khung FSM tối giản (smach)**:
```python
class GotoWaypoint(smach.State):
    def __init__(self, node):
        smach.State.__init__(self, outcomes=['arrived', 'failed'])
        self.node = node

    def execute(self, userdata):
        wp = self.node.current_waypoint
        self.node.fc_bridge_client.call_async(GotoWaypointRequest(target=wp.pos_ned, max_vel=wp.max_vel))
        while not self.node.arrived_at(wp, self.node.latest_odom):
            if self.node.failsafe_triggered:
                return 'failed'
            rclpy.spin_once(self.node, timeout_sec=0.1)
        return 'arrived'

class MarkerSearch(smach.State):
    def __init__(self, node):
        smach.State.__init__(self, outcomes=['found', 'timeout'])
        self.node = node

    def execute(self, userdata):
        self.node.publish_expected_marker_id(self.node.current_waypoint.expected_marker_id)
        deadline = self.node.get_clock().now() + Duration(seconds=self.node.search_timeout_s)
        while self.node.get_clock().now() < deadline:
            if self.node.marker_confirmed:
                return 'found'
            rclpy.spin_once(self.node, timeout_sec=0.1)
        return 'timeout'
```

**Nguyên tắc bắt buộc**: không bao giờ chuyển sang hạ cánh chỉ dựa vào toạ độ GPS/EKF một mình — luôn chờ `landing_target_bridge_node` xác nhận đúng ID marker mong đợi trước ("hai lớp định vị bổ trợ"); có `retry_count`/`max_retries` ở trạng thái `MARKER_SEARCH` — hết số lần thử chuyển `RETRY_LOITER` rồi báo `failsafe_monitor_node` thay vì lặp vô hạn; là nơi **duy nhất** gọi `ACTION_PICKUP`/`ACTION_DROPOFF` và **phải** chờ `gripper_status.sensor_confirmed = true` mới cho phép rời điểm — tuyệt đối không dùng timeout cố định thay cho xác nhận cảm biến thật (hàng có thể chưa gắp chắc dù đã đủ thời gian).

**Tham số quan trọng**: `search_timeout_s`, `max_retries`, `acceptance_radius` (khớp `fc_command_t.acceptance_radius`), `pre_dropoff_settle_s` (thời gian giữ ổn định trước khi mở gripper để tránh thả hàng khi còn đang dao động).

**Kiểm thử độc lập**: viết `pytest` cho riêng lớp FSM (không cần ROS chạy thật) bằng cách giả lập input qua mock object — kiểm tra mọi nhánh chuyển trạng thái kể cả các nhánh lỗi (mất marker, mất gripper confirm, hết pin giữa chừng) trước khi test tích hợp; test tích hợp chạy toàn bộ chuỗi trong Gazebo trước khi thử trên phần cứng.

### 4.2 `gripper_controller_node` — tự viết

**Mục đích**: điều khiển cơ cấu gắp/thả và **là nguồn xác nhận cứng duy nhất** rằng hàng đã gắp/thả — không được để `mission_manager_node` tự suy đoán qua timeout.

**Package**: `drone_mission`, `rclpy` + `pigpio`/`gpiozero`.

**Cài đặt** (nếu dùng `pigpio` — khuyến nghị vì cho PWM chính xác hơn `RPi.GPIO` phần mềm thuần trên Pi 4):
```bash
sudo apt install -y pigpio python3-pigpio
sudo systemctl enable --now pigpiod
```

**Input/Output**:

| Topic | Kiểu | Hướng |
|---|---|---|
| `/gripper/command` | `drone_interfaces/GripperCommand` (`GRIPPER_OPEN`/`GRIPPER_CLOSE`) | vào |
| `/gripper/status` | `drone_interfaces/GripperStatus` (`state`, `sensor_confirmed`, `force_reading_n`) | ra |

**Khung code** (servo/nam châm qua PWM, đọc cảm biến lực qua ADC hoặc công tắc hành trình qua GPIO ngắt):
```python
class GripperController(Node):
    def __init__(self):
        super().__init__('gripper_controller_node')
        self.pi = pigpio.pi()
        self.servo_pin = self.declare_parameter('servo_gpio', 18).value
        self.confirm_pin = self.declare_parameter('confirm_switch_gpio', 23).value
        self.pi.set_mode(self.confirm_pin, pigpio.INPUT)
        self.pi.set_pull_up_down(self.confirm_pin, pigpio.PUD_UP)
        self.sub = self.create_subscription(GripperCommand, '/gripper/command', self.on_command, 10)
        self.pub = self.create_publisher(GripperStatus, '/gripper/status', 10)
        self.timer = self.create_timer(0.1, self.publish_status)  # 10 Hz theo dõi liên tục

    def on_command(self, msg):
        pulse = self.close_pulse_us if msg.command == GripperCommand.GRIPPER_CLOSE else self.open_pulse_us
        self.pi.set_servo_pulsewidth(self.servo_pin, pulse)
        self.target_state = GripperStatus.GRIP_STATE_MOVING

    def publish_status(self):
        status = GripperStatus()
        status.sensor_confirmed = (self.pi.read(self.confirm_pin) == 0)  # active-low switch
        status.force_reading_n = self.read_force_sensor()  # nếu có cảm biến lực; else 0.0 và bỏ qua trong logic
        status.state = self.compute_state(status.sensor_confirmed)
        self.pub.publish(status)
```

**Tham số quan trọng**: `open_pulse_us`/`close_pulse_us` (hiệu chỉnh riêng theo servo/cơ cấu cơ khí thực tế — đo bằng tay trước khi hard-code), `confirm_debounce_ms` (chống nhiễu công tắc hành trình), `force_threshold_n` (nếu dùng cảm biến lực để phát hiện "gắp hụt" dù công tắc báo đóng — hàng quá nhẹ/lệch có thể không kích hoạt công tắc đúng cách).

**Kiểm thử độc lập**: test cơ khí tay không trước (không gắn cánh quạt) — gửi lệnh qua `ros2 topic pub`, xác nhận servo di chuyển đúng hướng/đủ hành trình; test với vật mẫu đúng trọng lượng thiết kế để xác nhận `sensor_confirmed` lên đúng khi gắp thật, không báo dương tính giả khi gắp hụt.

### 4.3 `fc_command_bridge_node` — tự viết

**Mục đích**: điểm tập trung duy nhất gọi API MAVROS — tránh các node khác gọi rải rác `/mavros/cmd/arming`, `/mavros/set_mode` khiến khó log và khó validate.

**Package**: `drone_mission`, `rclpy` + `mavros_msgs`.

**Service cung cấp cho `mission_manager_node`**: `arm`, `disarm`, `takeoff`, `goto_waypoint`, `hold`, `precision_land`, `land_now`, `rth` — mỗi service map tới đúng `fc_command_type_e` (`FC_CMD_ARM`, `FC_CMD_TAKEOFF`, `FC_CMD_GOTO_WAYPOINT`, `FC_CMD_HOLD`, `FC_CMD_PRECISION_LAND`, `FC_CMD_LAND_NOW`, `FC_CMD_RTH`).

**Khung code — gắn `seq` và chờ ACK** (khớp `fc_status_t.last_command_seq_acked`):
```python
class FcCommandBridge(Node):
    def __init__(self):
        super().__init__('fc_command_bridge_node')
        self.seq_counter = 0
        self.arming_client = self.create_client(CommandBool, '/mavros/cmd/arming')
        self.setpoint_pub = self.create_publisher(PositionTarget, '/mavros/setpoint_raw/local', 10)
        self.status_sub = self.create_subscription(FcStatus, '/mavros/... hoặc /fc_status_bridge', self.on_status, 10)

    def send_goto(self, target_ned, max_vel, acceptance_radius, timeout_s=5.0):
        self.seq_counter += 1
        my_seq = self.seq_counter
        setpoint = PositionTarget()
        setpoint.position.x, setpoint.position.y, setpoint.position.z = target_ned
        setpoint.coordinate_frame = PositionTarget.FRAME_LOCAL_NED
        self.setpoint_pub.publish(setpoint)

        deadline = self.get_clock().now() + Duration(seconds=timeout_s)
        while self.get_clock().now() < deadline:
            if self.last_acked_seq >= my_seq:
                return True
            rclpy.spin_once(self, timeout_sec=0.1)
        self.get_logger().error(f'FC không ACK lệnh seq={my_seq} trong {timeout_s}s')
        return False
```

**Lưu ý quan trọng — firmware tuỳ biến**: vì firmware FC là tuỳ biến (không phải PX4/ArduPilot nguyên bản), cần xác minh sớm rằng firmware đã hiện thực đủ `COMMAND_LONG` với `MAV_CMD_COMPONENT_ARM_DISARM`, `SET_POSITION_TARGET_LOCAL_NED`, và `HEARTBEAT` có `base_mode`/`custom_mode` hợp lệ trước khi tin tưởng hoàn toàn plugin MAVROS mặc định — nếu một số lệnh chưa khớp hành vi PX4/ArduPilot mà plugin giả định, node này có thể cần gọi trực tiếp bằng `pymavlink` thô cho riêng lệnh đó thay vì qua plugin.

**Kiểm thử độc lập**: dùng `ros2 service call` gọi từng service một trước khi ráp vào FSM đầy đủ; test ACK bằng cách rút dây kết nối FC giữa chừng — service phải trả lỗi rõ ràng trong `timeout_s`, không treo vô hạn.

## 5. Lớp Giao tiếp (`drone_comms`)

### 5.1 `mavros`

**Cấu hình** (`mavros_params.yaml`), không tự viết node:
```yaml
mavros:
  ros__parameters:
    fcu_url: "/dev/ttyUSB0:921600"     # khớp baudrate MAVLink đã cấu hình trên H743
    plugin_allowlist:
      - 'sys_status'
      - 'setpoint_position'
      - 'setpoint_raw'
      - 'command'
      - 'imu'
      - 'battery'
      - 'landing_target'
      - 'local_position'
```
Dùng `plugin_allowlist` thay vì để mặc định bật toàn bộ — vừa giảm tải CPU Pi 4, vừa tránh plugin ngầm gửi lệnh mà firmware tuỳ biến không hiểu (gây log lỗi liên tục hoặc trong trường hợp xấu là hành vi không mong muốn).

**Kiểm thử**: `ros2 launch mavros px4.launch fcu_url:=/dev/ttyUSB0:921600` rồi `ros2 topic echo /mavros/state` — trường `connected: true` là điều kiện cần trước khi làm bất kỳ node nào ở lớp trên.

### 5.2 `gcs_link_node` — tự viết

**Mục đích**: đường liên lạc GCS riêng của Pi 4 (4G/LTE hoặc radio riêng), độc lập với USART3/radio telemetry của FC — theo phương án (b) đã chốt trong tài liệu kiến trúc.

**Package**: `drone_comms`, `rclpy` + `pymavlink` (dùng dialect MAVLink tuỳ biến mang `mission_plan_t`/`telemetry_packet_t`/`failsafe_event_t`).

**Input/Output**:

| | Kiểu | Hướng |
|---|---|---|
| socket 4G/LTE (UDP/TCP tới GCS) | MAVLink tuỳ biến | vào/ra |
| `/mission/plan` | `drone_interfaces/MissionPlan` | ra (nội bộ, cho mission_manager) |
| `/telemetry/outgoing` | `drone_interfaces/TelemetryPacket` | vào (nội bộ, từ telemetry_aggregator) |
| `/gcs_link/connected` | `std_msgs/Bool` | ra — cho failsafe_monitor |

**Nguyên tắc bắt buộc**: lệnh khẩn cấp (RTH, hạ cánh khẩn cấp, huỷ nhiệm vụ) đi qua hàng đợi ưu tiên riêng, **không** xếp sau các gói telemetry thường trong cùng một hàng đợi gửi — nên dùng hai socket/hai kênh logic riêng nếu băng thông cho phép, hoặc ít nhất hai hàng đợi ưu tiên trong cùng một kết nối; tự phát hiện mất kết nối bằng watchdog thời gian (không đợi GCS chủ động báo — lúc mất kết nối GCS không thể báo gì), publish `/gcs_link/connected = false` ngay khi vượt ngưỡng timeout để `failsafe_monitor_node` phản ứng.

**Tham số quan trọng**: `link_timeout_s`, `heartbeat_interval_s`, ưu tiên hàng đợi (`emergency` > `mission_ack` > `telemetry`).

**Kiểm thử độc lập**: giả lập GCS bằng một script `pymavlink` chạy trên máy tính khác cùng mạng 4G/hotspot; test rút mất kết nối 4G giữa chừng, đo đúng thời điểm `/gcs_link/connected` chuyển `false` so với `link_timeout_s` đã đặt.

### 5.3 `telemetry_aggregator_node` — tự viết

**Mục đích**: gộp trạng thái nhiều nguồn thành một gói `telemetry_packet_t` duy nhất, tách khỏi logic mã hoá/giải mã của `gcs_link_node`.

**Package**: `drone_comms`, `rclpy`.

**Input**: `/mission/state`, `/mavros/state`, `/mavros/battery`, `/gripper/status`, `/marker/tracking_quality`, RSSI (từ modem 4G qua `AT command`/`ModemManager` hoặc từ driver radio).

**Output**: `/telemetry/outgoing` (`drone_interfaces/TelemetryPacket`), tần số **1–5 Hz cố định bằng timer**, không publish theo sự kiện của từng nguồn (tránh làm ngập kênh 4G/radio băng thông hẹp).

**Lưu ý**: trường nào không đổi thường xuyên (ví dụ `mission_id`) không cần gửi lại mỗi gói nếu giao thức phía GCS hỗ trợ giữ trạng thái, nhưng nên bắt đầu đơn giản (gửi đủ trường mỗi lần) và chỉ tối ưu băng thông sau khi đã đo được là cần thiết.

## 6. Lớp An toàn & Ghi log (`drone_safety`)

### 6.1 `failsafe_monitor_node` — tự viết

**Mục đích**: giám sát tập trung duy nhất được phép **chủ động** yêu cầu đổi trạng thái nhiệm vụ khi có sự cố — hiện thực hoá `failsafe_type_e` (`FS_QR_TIMEOUT`/`FS_MARKER_TIMEOUT`, `FS_GRIP_CONFIRM_FAIL`, `FS_LINK_LOST`, `FS_LOW_BATTERY`, `FS_EKF_UNHEALTHY`, `FS_OBSTACLE`, `FS_FC_COMM_LOST`).

**Package**: `drone_safety`, `rclpy`.

**Input**: `/mavros/battery`, `/gcs_link/connected`, `/landing_target/lost`, EKF `healthy` (từ `state_estimator_node`, mục 2.1), `/gripper/status`, `/mavros/state` (mất heartbeat FC).

**Output**: `/failsafe_event` (`drone_interfaces/FailsafeEvent`, Reliable QoS, không được rớt gói này), gọi trực tiếp service tương ứng của `fc_command_bridge_node`/`mission_manager_node` để leo thang **loiter → RTH → hạ cánh khẩn cấp** theo đúng thứ tự đã mô tả trong tài liệu phân tích tổng thể — không chỉ publish cảnh báo suông rồi chờ node khác tự quyết.

**Khung logic — kiểm tra định kỳ bằng timer, không phải theo callback lẻ tẻ** (để đảm bảo không sự cố nào "lọt" vì không có sự kiện mới kích hoạt kiểm tra):
```python
def periodic_check(self):
    if self.battery_pct < self.params.low_battery_pct:
        self.raise_failsafe(FS_LOW_BATTERY, escalate_to='RTH')
    if not self.ekf_healthy:
        self.raise_failsafe(FS_EKF_UNHEALTHY, escalate_to='LOITER_THEN_LAND')
    if self.time_since_fc_heartbeat() > self.params.fc_comm_timeout_s:
        self.raise_failsafe(FS_FC_COMM_LOST, escalate_to='EMERGENCY_LAND')
    if self.state == MSTATE_MARKER_SEARCH and self.time_in_state() > self.params.marker_search_timeout_s:
        self.raise_failsafe(FS_QR_TIMEOUT, escalate_to='RETRY_LOITER')
```

**Tham số quan trọng — bắt buộc khớp với GCS**: `low_battery_pct`, `link_lost_timeout_s`, `marker_search_timeout_s`, `max_retries`, `fc_comm_timeout_s` — toàn bộ phải nạp từ cùng một nguồn cấu hình (hoặc đồng bộ thủ công cẩn thận) với bảng `system_config` đã thiết kế phía GCS, để hai bên không lệch ngưỡng khi vận hành thực địa (ví dụ GCS nghĩ pin còn an toàn trong khi Pi 4 đã kích RTH, gây hoang mang người vận hành).

**Kiểm thử độc lập**: test riêng từng nhánh failsafe bằng cách giả lập input (ngắt dây FC, che marker, rút anten 4G, publish pin giả thấp qua `ros2 topic pub` đè lên topic thật trong môi trường test) — xác nhận đúng chuỗi leo thang, không nhảy thẳng lên bước nặng nhất khi chưa cần, và không "quên" một nhánh nào khi test tích hợp toàn bộ cùng lúc nhiều lỗi.

### 6.2 `diagnostics_node`

**Dùng gói có sẵn**: `ros-jazzy-diagnostic-updater` (thư viện dùng trong từng node) + `ros-jazzy-diagnostic-aggregator` (gộp tập trung) — không tự viết.

**Cấu hình**: mỗi node quan trọng (`camera_node`, `marker_detector_node`, `optical_flow_node`, `state_estimator_node`, `mavros`) tự thêm vài dòng dùng `diagnostic_updater.Updater` để publish `DiagnosticStatus` (OK/WARN/ERROR kèm lý do) của chính mình lên `/diagnostics`, thay vì để `failsafe_monitor_node` tự đoán qua dữ liệu gián tiếp. Ví dụ tối thiểu thêm vào `optical_flow_node`:
```python
self.diag_updater = diagnostic_updater.Updater(self)
self.diag_updater.setHardwareID('optical_flow')
self.diag_updater.add('flow_quality', self.diagnostic_callback)

def diagnostic_callback(self, stat):
    if self.n_tracked < self.min_tracked_features:
        stat.summary(DiagnosticStatus.WARN, f'Chỉ theo dõi được {self.n_tracked} đặc trưng')
    else:
        stat.summary(DiagnosticStatus.OK, f'{self.n_tracked} đặc trưng đang theo dõi')
    return stat
```

`aggregator.yaml` gộp các nguồn này thành cây trạng thái tổng — dùng trực tiếp làm nguồn dữ liệu cho màn hình "Quản trị hệ thống" phía GCS mà không cần thêm tầng chuyển đổi.

### 6.3 `mission_logger_node` — tự viết + `rosbag2`

**Mục đích**: hai loại log khác mục đích, không gộp làm một — `rosbag2` cho debug kỹ thuật thô, node tự viết cho log nghiệp vụ có cấu trúc đối chiếu được với CSDL GCS.

**Phần dùng sẵn** (`rosbag2`, không tự viết):
```bash
ros2 bag record -o mission_$(date +%Y%m%d_%H%M%S) \
  /camera/image_raw /aruco/poses /optical_flow/velocity /odometry/filtered \
  /mavros/state /mavros/battery /mission/state /failsafe_event /gripper/status
```
Nên gọi trong launch file bằng `ExecuteProcess` để tự khởi động cùng toàn bộ stack, đặt tên thư mục theo `mission_id` ngay khi có thể để dễ đối chiếu sau này.

**Phần tự viết**: node subscribe `/mission/state`, `/failsafe_event`, `/marker/tracking_quality`, `/gripper/status` và ghi thành `log_event_t` dạng có cấu trúc (`event_type`: `"STATE_CHANGE"`, `"FAILSAFE"`, `"MARKER_MATCH"`, `"GRIP_OK"`...) ra file JSON-lines theo thư mục `mission_id`, cộng lưu ảnh xác nhận tại thời điểm gắp/thả (tương ứng `mission_photo` phía GCS) — vì dữ liệu này cần đối chiếu nghiệp vụ (bảng `mission`, `mission_waypoint`, `mission_photo`), không chỉ để debug kỹ thuật như `rosbag2`.

**Lưu ý hiệu năng Pi 4**: ghi `rosbag2` với `/camera/image_raw` full-size liên tục có thể tốn I/O thẻ SD đáng kể trên chuyến bay dài — cân nhắc dùng thẻ SD tốc độ cao (class U3/A2) hoặc SSD qua USB 3.0, và/hoặc chỉ ghi ảnh nén (`image_transport compressed`) thay vì ảnh thô nếu dung lượng/tốc độ ghi là vấn đề.

## 7. Bảng tổng hợp gói cần cài theo từng node (tra nhanh)

| Node | Loại | Gói apt (nếu có) |
|---|---|---|
| camera_node | có sẵn | `ros-jazzy-usb-cam` / `ros-jazzy-v4l2-camera` / `camera_ros` (build source) |
| image_rectify | có sẵn | `ros-jazzy-image-proc` |
| marker_detector_node | có sẵn | `ros-jazzy-aruco-opencv` / `ros-jazzy-apriltag-ros` |
| optical_flow_node | tự viết | `python3-opencv` |
| state_estimator_node | có sẵn | `ros-jazzy-robot-localization` |
| landing_target_bridge_node | tự viết | `ros-jazzy-mavros-extras` (message `LandingTarget`) |
| position_controller_node | tự viết | — |
| mission_manager_node | tự viết | `python3-smach` (hoặc `py_trees`) |
| gripper_controller_node | tự viết | `pigpio`/`python3-pigpio` |
| fc_command_bridge_node | tự viết | `ros-jazzy-mavros` |
| mavros | có sẵn | `ros-jazzy-mavros`, `ros-jazzy-mavros-extras` |
| gcs_link_node | tự viết | `pymavlink` (`pip install pymavlink`) |
| telemetry_aggregator_node | tự viết | — |
| failsafe_monitor_node | tự viết | — |
| diagnostics_node | có sẵn (thư viện dùng trong node) | `ros-jazzy-diagnostic-updater`, `ros-jazzy-diagnostic-aggregator` |
| mission_logger_node | tự viết + có sẵn | `ros-jazzy-rosbag2` |

## 8. Thứ tự triển khai và kiểm thử khuyến nghị

Bám sát lộ trình 5 giai đoạn đã chốt trong `thiet_ke_kien_truc_node_ros2.md`, thứ tự dựng node cụ thể nên là: (1) `camera_node` → hiệu chỉnh camera → `image_rectify` → `marker_detector_node`, xác nhận pose marker đúng bằng thước đo tay; (2) `optical_flow_node`, xác nhận vận tốc đúng bằng cách di chuyển camera một quãng đã biết; (3) `state_estimator_node`, thêm dần từng nguồn một (IMU trước, rồi optical flow, rồi marker); (4) `landing_target_bridge_node` + `position_controller_node`, tune PID trong Gazebo trước khi lên phần cứng; (5) `fc_command_bridge_node` + `mission_manager_node` (điều khiển thủ công tại chỗ); (6) `gripper_controller_node`, test cơ khí độc lập trước khi ráp vào FSM; (7) `gcs_link_node` + `telemetry_aggregator_node`, test bằng GCS giả lập trước khi dùng GCS thật; (8) `failsafe_monitor_node` + `diagnostics_node` + `mission_logger_node`, test từng nhánh lỗi một cách chủ động (rút dây, che marker, rút anten) trước khi coi hệ thống sẵn sàng bay nhiệm vụ đầy đủ nhiều chặng liên tục.
