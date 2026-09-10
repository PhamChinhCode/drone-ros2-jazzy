# Kế hoạch thiết kế chi tiết từng node ROS 2 và trình tự triển khai

*Bản kế hoạch thi công, dẫn xuất từ `thiet_ke_kien_truc_node_ros2.md` (kiến trúc) và
`huong_dan_chi_tiet_cac_node_ros2_pi4.md` (hướng dẫn từng node). Tài liệu này không đặt lại kiến trúc —
nó chốt các chi tiết còn để ngỏ (tên topic, tên trường message, QoS cụ thể, tham số mặc định) tới mức
có thể gõ code thẳng ra được, và nêu rõ tiêu chí "xong" của từng node.
Cập nhật 08/09/2026 — ROS 2 Jazzy, Ubuntu 24.04, Raspberry Pi 4.*

## 1. Các quyết định chốt trước khi thi công

| # | Điểm để ngỏ trong tài liệu gốc | Quyết định | Lý do |
|---|---|---|---|
| 1 | `usb_cam` / `v4l2_camera` / `camera_ros` | ~~`camera_ros`~~ → **`v4l2_camera`** đọc thẳng `/dev/video0` (*sửa 08/09, xem mục 1b*) | `camera_ros`/libcamera có hai lỗi không sửa được từ ứng dụng trên máy này |
| 2 | ArUco hay AprilTag | **AprilTag**, family `tag36h11`, cạnh **0.13 m** | chính xác góc tốt hơn khi nghiêng lớn; 0.13 m khớp bản in sẵn trong `/home/pc/ros2_ws/assets/tags/` |
| 3 | Kênh GCS: FC forward (a) hay đường riêng (b) | **(b)** — 4G/LTE gắn thẳng Pi 4 | không phải sửa firmware FC; đúng lựa chọn mục 1 tài liệu kiến trúc |
| 4 | `smach` / `py_trees` / lifecycle tự quản | **FSM tự viết dạng lớp Python thuần** | `python3-smach` chưa cài, và FSM ở đây tuyến tính; lớp thuần dễ `pytest` không cần ROS |
| 5 | `LandingTarget` plugin hay `setpoint_raw` | **`mavros_msgs/LandingTarget`** qua plugin `landing_target` | đúng thiết kế `landing_target_t`; có đường lùi sang `setpoint_raw` nếu firmware không nhận |
| 6 | Ngôn ngữ node tự viết | **`rclpy` toàn bộ** | khớp mọi khung code trong tài liệu hướng dẫn; đủ nhanh ở tần số các node này cần |

### 1b. Sửa quyết định số 1 (08/09/2026)

`/home/pc/ros2_ws/docs/CAMERA.md` — tài liệu đã kiểm chứng trên đúng máy này — ghi nhận
`camera_ros`/libcamera **không dùng được** trên Ubuntu 24.04 + libcamera 0.2.0 (hai lỗi IPA không
sửa được từ phía ứng dụng). Đường đã kiểm chứng chạy được là **đọc thẳng V4L2** qua `v4l2_camera`.

Camera thực tế là **OV9281 mono global shutter 1280×800**, không phải camera màu như giả định ban
đầu — kéo theo `pixel_format=GREY`, `output_encoding=mono8`. Sensor mono có lợi: không Bayer nên
không cần ISP/debayer, AprilTag không cần màu, và rectify chạy mono8 nhẹ CPU đúng như tài liệu gốc
khuyến nghị.

**Quy ước khung toạ độ** (chốt một lần, mọi node theo): `map` → `odom` → `base_link` → `camera_link` →
`camera_optical_frame`. EKF publish `odom → base_link`. Transform tĩnh `base_link → camera_link`
khai trong `drone_bringup` (đo theo vị trí lắp camera thực tế). Marker nằm trong `map`.

**Quy ước QoS** (theo mục 0 tài liệu hướng dẫn):
- `SENSOR_QOS` = Best Effort, depth 1 — ảnh, IMU, odometry, optical flow, setpoint, landing_target.
- `EVENT_QOS` = Reliable, depth 10 — lệnh, failsafe, ACK, mission state, gripper, mission plan.

**Quy ước tham số**: mọi node tự viết chỉ đọc tham số qua `declare_parameter` và được nạp từ YAML trong
`drone_bringup/config/`. Không hard-code ngưỡng nào trong code.

## 2. Cấu trúc workspace mục tiêu

```
ros2_ws/src/
├── camera_ros/              # đã có sẵn (driver CSI), không sửa
├── drone_interfaces/        # ament_cmake — .msg/.srv (định nghĩa ĐẦY ĐỦ)
├── drone_perception/        # optical_flow_node, marker_quality_node
├── drone_estimation/        # marker_pose_republisher_node, ekf_health_node (+ ekf.yaml)
├── drone_control/           # landing_target_bridge_node, position_controller_node
├── drone_mission/           # mission_manager_node, gripper_controller_node, fc_command_bridge_node
├── drone_comms/             # gcs_link_node, telemetry_aggregator_node (+ mavros.yaml)
├── drone_safety/            # failsafe_monitor_node, mission_logger_node (+ aggregator.yaml)
└── drone_bringup/           # launch theo từng giai đoạn + toàn bộ file YAML tham số
```

`drone_interfaces` dùng `ament_cmake` (bắt buộc với `rosidl`); tám package còn lại dùng `ament_python`.

Ba node **không có trong danh sách tài liệu gốc** nhưng cần thêm, vì tài liệu có nêu nhu cầu mà không
đặt tên node riêng:

- `marker_quality_node` — mục 1.3 tài liệu hướng dẫn gọi là `marker_quality_publisher.py`.
- `marker_pose_republisher_node` — mục 2.1 nói cần "một node nhỏ" đổi detection sang
  `PoseWithCovarianceStamped` trong khung `odom` cho `robot_localization`.
- `ekf_health_node` — mục 2.1 nói `robot_localization` không có khái niệm `healthy` nhị phân,
  cần node nhỏ suy ra từ `/diagnostics`.

## 3. Package `drone_interfaces` — định nghĩa message/service

Tên trường bám theo struct C đã nêu trong tài liệu (`mission_state_e`, `fc_command_type_e`,
`failsafe_type_e`, `gripper_status_t`). **Chưa đối chiếu được với `thiet_ke_cau_truc_du_lieu_he_thong_drone.md`
vì file đó không có trong `docs/` — cần rà lại tên trường khi có file.**

### 3.1 Message

| File | Trường chính |
|---|---|
| `MissionWaypoint.msg` | hằng `ACTION_NONE/ACTION_PICKUP/ACTION_DROPOFF`; `seq`, `lat`, `lon`, `alt_m`, `pos_ned[3]`, `expected_marker_id` (−1 = không xác thực), `action`, `acceptance_radius_m`, `max_vel_mps`, `loiter_s` |
| `MissionPlan.msg` | `mission_id`, `plan_name`, `MissionWaypoint[] waypoints`, `max_retries`, `search_timeout_s`, `issued_stamp` |
| `MissionState.msg` | hằng `IDLE, TAKEOFF, ENROUTE, MARKER_SEARCH, PRECISION_LAND, ACTUATE_GRIPPER, RETRY_LOITER, RTH, EMERGENCY_LAND, MISSION_COMPLETE, FAILSAFE`; `state`, `mission_id`, `current_wp_index`, `expected_marker_id`, `retry_count`, `state_entered_stamp`, `detail` |
| `FcCommand.msg` | hằng `FC_CMD_ARM/DISARM/TAKEOFF/GOTO_WAYPOINT/HOLD/PRECISION_LAND/LAND_NOW/RTH`; `seq`, `command`, `target_ned[3]`, `yaw_deg`, `max_vel_mps`, `acceptance_radius_m` |
| `GripperCommand.msg` | hằng `GRIPPER_OPEN/GRIPPER_CLOSE`; `seq`, `command` |
| `GripperStatus.msg` | hằng `GRIP_STATE_OPEN/CLOSED/MOVING/ERROR`; `state`, `sensor_confirmed`, `force_reading_n`, `stamp` |
| `MarkerQuality.msg` | `marker_id`, `visible`, `distance_m`, `lateral_offset_m`, `reprojection_error`, `stamp` |
| `EkfHealth.msg` | `healthy`, `pos_variance[3]`, `reason`, `stamp` |
| `FailsafeEvent.msg` | hằng `FS_MARKER_TIMEOUT/FS_GRIP_CONFIRM_FAIL/FS_LINK_LOST/FS_LOW_BATTERY/FS_EKF_UNHEALTHY/FS_FC_COMM_LOST`; hằng leo thang `ESCALATE_NONE/LOITER/RETRY_LOITER/RTH/EMERGENCY_LAND`; `type`, `escalate_to`, `active`, `detail`, `stamp` |
| `TelemetryPacket.msg` | `mission_id`, `mission_state`, `current_wp_index`, `lat`, `lon`, `alt_m`, `vel_ned[3]`, `battery_pct`, `battery_v`, `fc_connected`, `gcs_rssi_dbm`, `marker_id_tracking`, `gripper_state`, `failsafe_type`, `stamp` |

### 3.2 Service (cho `fc_command_bridge_node`)

Tài liệu liệt kê 8 service. Gộp 5 lệnh không tham số vào một service enum để bớt trùng lặp:

| File | Request | Response |
|---|---|---|
| `Arm.srv` | `bool arm` | `bool success`, `string message`, `uint32 seq` |
| `Takeoff.srv` | `float32 altitude_m` | như trên |
| `GotoWaypoint.srv` | `float32[3] target_ned`, `float32 yaw_deg`, `float32 max_vel_mps`, `float32 acceptance_radius_m` | như trên |
| `FcSimpleCommand.srv` | `uint8 command` (`HOLD/PRECISION_LAND/LAND_NOW/RTH/DISARM`) | như trên |

**Tiêu chí xong**: `colcon build --packages-select drone_interfaces` sạch và
`ros2 interface show drone_interfaces/msg/<X>` in đúng mọi message.

## 4. Thiết kế chi tiết từng node

Mỗi node ghi theo cùng khuôn: trách nhiệm → I/O (topic, kiểu, QoS) → tham số + giá trị mặc định →
trạng thái nội bộ → điểm dễ sai → tiêu chí xong. Cột "Phiên này" cho biết mức hoàn thiện đợt hiện tại
(**Stub** = đã nối dây I/O + tham số, phần thuật toán để `TODO`).

### 4.1 `drone_perception`

#### `camera_node` — dùng `v4l2_camera` (không viết code)

- **Ra**: `/camera/image_raw` (`sensor_msgs/Image`, `mono8`, SENSOR_QOS), `/camera/camera_info`.
- **Tham số**: `video_device=/dev/video0`, `pixel_format=GREY`, `output_encoding=mono8`,
  `image_size=[640,400]` (chế độ binned; 1280×800 nếu cần tầm phát hiện xa hơn),
  `camera_info_url` **phải trỏ file khớp đúng độ phân giải** (`unicam_640x400.yaml`) — để rỗng
  thì node rơi về `unicam.yaml` (1280×800), lệch độ phân giải nên `camera_info_manager` bỏ im
  lặng và publish intrinsics rỗng (`k` toàn 0), node **chỉ log INFO**.
- **Bước bắt buộc trước mỗi lần chạy, và lặp lại sau MỖI LẦN REBOOT**:
  `/home/pc/ros2_ws/scripts/camera_v4l2_setup.sh --width 640 --height 400 --vblank 1779 --exposure 800 --gain 120`
  — ép subdev về `Y8_1X8` cho khớp `GREY`, và đặt exposure/gain bằng tay (không có IPA thì
  không có auto-exposure). Bỏ qua bước này thì topic vẫn ra đúng nhịp nhưng **mọi khung hình
  toàn số 0**, và `ros2 topic hz` **không phát hiện được** — phải kiểm thống kê pixel.
- **FPS đặt qua `--vblank`, không qua YAML**: `vblank=1779` → 60 FPS. **Không dùng 246 FPS**
  (`--vblank 110`) dù sensor chạy được: đo thực trên máy này cho thấy 246 FPS làm Pi 4 bão hoà
  (load 9.4, idle 1%) và phá vỡ đồng bộ ảnh↔`camera_info` của `apriltag_ros` (chỉ 15 cặp/10 s,
  WARN liên tục). Ở 60 FPS: 373/375 khung khớp stamp, log sạch, apriltag xử lý đủ 60 Hz.
  Công thức: `FPS = pixel_rate / ((400+vblank)×(640+890))`, `pixel_rate=200e6`.
- **Điểm dễ sai**: bỏ qua hiệu chỉnh camera → sai lệch **hệ thống** ở mọi phép đo khoảng cách
  marker. Máy không có GUI nên dùng `/home/pc/ros2_ws/tools/calibrate_camera.py`.
- **Phiên này**: chỉ launch + YAML.

#### `image_rectify` — dùng `image_proc/rectify_node` (không viết code)

- **Vào**: `/camera/image_raw`, `/camera/camera_info` → **Ra**: `/camera/image_rect`.
- **Lưu ý Pi 4**: nếu CPU căng, chạy rectify ở `mono8` (AprilTag không cần màu) thay vì color.
- **Phiên này**: chỉ khai trong launch với remapping.

#### `marker_detector_node` — dùng `apriltag_ros` (không viết code)

- **Vào**: `/camera/image_rect`, `/camera/camera_info`.
- **Ra**: `/apriltag/detections` (`apriltag_msgs/AprilTagDetectionArray`), tf2 `camera_optical_frame → tag_<id>`.
- **Tham số**: `family=tag36h11`, `size=0.15` (mét — **phải đúng kích thước in thật**, sai bao nhiêu %
  thì khoảng cách sai bấy nhiêu %), khai danh sách tag của từng bãi đáp trong YAML.
- **Phiên này**: chỉ launch + YAML.

#### `marker_quality_node` — **tự viết**

- **Trách nhiệm**: dịch detection thô thành một bản tóm tắt "đang bám ID nào, xa bao nhiêu, sai số bao nhiêu"
  để `failsafe_monitor_node` không phải tự parse cấu trúc detection.
- **Vào**: `/apriltag/detections` (SENSOR_QOS), `/mission/expected_marker_id` (`std_msgs/Int32`, EVENT_QOS).
- **Ra**: `/marker/tracking_quality` (`drone_interfaces/MarkerQuality`, EVENT_QOS) — publish đều bằng timer
  10 Hz kể cả khi không thấy marker (khi đó `visible=false`), để bên nhận phân biệt được
  "không thấy marker" với "node chết".
- **Tham số**: `publish_rate_hz=10.0`, `visible_timeout_s=0.5`.
- **Tiêu chí xong**: che/mở marker → `visible` lật đúng trong khoảng `visible_timeout_s`.
- **Phiên này**: **Stub**.

#### `optical_flow_node` — **tự viết** (node rủi ro cao nhất)

- **Trách nhiệm**: ước lượng vận tốc trôi ngang (vx, vy trong khung thân) khi không thấy marker.
- **Vào**: `/camera/image_raw` (ảnh xám, SENSOR_QOS), `/mavros/global_position/rel_alt`
  (`std_msgs/Float64`) hoặc `/mavros/distance_sensor/rangefinder` — cần độ cao để quy đổi pixel → mét;
  `/camera/camera_info` để lấy `focal_length_px = K[0]` (**không đo tay**).
- **Ra**: `/optical_flow/velocity` (`TwistWithCovarianceStamped`, SENSOR_QOS).
- **Thuật toán**: Lucas–Kanade thưa (`goodFeaturesToTrack` + `calcOpticalFlowPyrLK`), lấy **median**
  của vector dịch chuyển (chống nhiễu tốt hơn mean), quy đổi `v = flow_px * alt / focal_px / dt`.
- **Tham số**: `max_corners=100`, `quality_level=0.3`, `min_distance=7`, `min_tracked_features=8`,
  `refresh_interval_s=1.5`, `process_every_n_frames=2`.
- **Ba nguyên tắc bắt buộc** (theo mục 1.4 tài liệu hướng dẫn):
  1. covariance **tỉ lệ nghịch** với số đặc trưng bám được — đây là cách báo chất lượng thấp cho EKF,
     không cần topic cờ riêng;
  2. `n_tracked < min_tracked_features` → **không publish gì cả**, tuyệt đối không publish giá trị nhiễu;
  3. gọi lại `goodFeaturesToTrack` định kỳ để không bám mãi các điểm đã trôi khỏi khung hình.
- **Phần logic thuần**: lớp `OpticalFlowEstimator` tách rời khỏi node để `pytest` được không cần ROS.
- **Tiêu chí xong**: camera đứng yên → vận tốc ≈ 0; dịch camera một quãng đo bằng thước ở độ cao cố định →
  tích phân vận tốc khớp gần đúng quãng đó.
- **Phiên này**: **Stub** (khung lớp + node đã nối dây, phần tính flow để `TODO`).

### 4.2 `drone_estimation`

#### `state_estimator_node` — dùng `robot_localization/ekf_node` (không viết code)

- **Vào**: `/mavros/imu/data` (imu0), `/optical_flow/velocity` (twist0 — **chỉ tin vx, vy; KHÔNG lấy vz**),
  `/marker/pose_odom` (pose0 — chỉ vị trí x, y, z).
- **Ra**: `/odometry/filtered` (`nav_msgs/Odometry`, ≥ 20–30 Hz), tf2 `odom → base_link`.
- **Cấu hình**: `ekf.yaml` với `frequency=30.0`, `two_d_mode=false`, `pose0_rejection_threshold=2.0`
  (loại pose marker nhảy vọt theo Mahalanobis).
- **Nguyên tắc chỉnh hiệp phương sai**: marker = vị trí tuyệt đối, thưa, tin nhiều (covariance thấp);
  optical flow = vận tốc tương đối, liên tục, trôi tích luỹ → để chính `optical_flow_node` tự tăng
  covariance động theo `n_tracked` thay vì đặt cứng ở đây.
- **Phiên này**: chỉ `ekf.yaml` + launch.

#### `marker_pose_republisher_node` — **tự viết**

- **Trách nhiệm**: đổi detection AprilTag (pose tương đối trong khung quang học camera) thành
  `PoseWithCovarianceStamped` **trong khung `odom`** để EKF nhận được — EKF không tự làm bước này.
- **Vào**: `/apriltag/detections`, tf2. **Ra**: `/marker/pose_odom` (`PoseWithCovarianceStamped`, SENSOR_QOS).
- **Tham số**: `pos_covariance=0.01` (m², thấp vì marker là vị trí tuyệt đối), `known_tags` (map ID → toạ độ bãi đáp trong `map`).
- **Điểm dễ sai**: nhầm chiều transform camera↔body → "thấy đúng marker nhưng bay lệch tâm".
- **Phiên này**: **Stub**.

#### `ekf_health_node` — **tự viết**

- **Trách nhiệm**: sinh cờ `healthy` nhị phân mà `robot_localization` không có sẵn.
- **Vào**: `/diagnostics`, `/odometry/filtered` (đọc `pose.covariance` đường chéo).
- **Ra**: `/ekf/health` (`drone_interfaces/EkfHealth`, EVENT_QOS).
- **Quy tắc**: phương sai vị trí vượt `max_pos_variance` (mặc định `2.0`) liên tục quá `unhealthy_after_s`
  (mặc định `2.0`) → `healthy=false`; hoặc `/odometry/filtered` im quá `odom_timeout_s=1.0`; hoặc gặp nan/inf.
- **Phiên này**: **Stub**.

### 4.3 `drone_control`

#### `landing_target_bridge_node` — **tự viết**

- **Trách nhiệm**: **node duy nhất** được phép khẳng định "đang bám đúng marker mong đợi" — tách việc
  xác thực ID khỏi `marker_detector_node` (chỉ báo mọi ID thấy được) và khỏi `mission_manager_node`
  (không nên tự parse pose thô).
- **Vào**: `/apriltag/detections` (SENSOR_QOS), `/mission/expected_marker_id` (`std_msgs/Int32`, EVENT_QOS), tf2 tĩnh `camera → base_link`.
- **Ra**: `/mavros/landing_target/raw` (`mavros_msgs/LandingTarget`, SENSOR_QOS, 20–30 Hz trong pha bám);
  `/landing_target/lost` (`std_msgs/Bool`, EVENT_QOS).
- **Tham số**: `timeout_s=0.7` (quá dài → FC bám vị trí cũ đã sai; quá ngắn → báo mất bám giả khi rung
  một khung hình), `publish_rate_hz=25.0`, `target_frame=base_link`.
- **Nguyên tắc bắt buộc**: mất marker → **ngừng publish ngay**, không nội suy, không giữ giá trị cũ.
- **Tiêu chí xong**: di chuyển marker bằng tay trước camera đứng yên → pose đổi dấu đúng chiều
  trái/phải/gần/xa; che marker đúng `timeout_s` → `/landing_target/lost` bật đúng lúc.
- **Phiên này**: **Stub**.

#### `position_controller_node` — **tự viết**

- **Trách nhiệm**: vòng PID vị trí/vận tốc cấp companion computer. **Khác tầng** với cascade PID
  góc/tốc-độ-góc đã chạy trên STM32H743 — node này chỉ nói "muốn ở đâu / nhanh cỡ nào", FC vẫn tự giữ góc.
- **Vào**: `/odometry/filtered` (SENSOR_QOS), `/mission/setpoint` (`geometry_msgs/PoseStamped`, EVENT_QOS),
  `/mavros/landing_target/raw` (khi đang hạ cánh chính xác).
- **Ra**: `/mavros/setpoint_raw/local` (`mavros_msgs/PositionTarget`, SENSOR_QOS, 20 Hz).
- **Tham số**: `kp/ki/kd` riêng từng trục x, y, z và riêng từng pha (hành trình ↔ hạ cánh),
  `i_limit`, `out_limit` (giới hạn vận tốc ra để không vượt an toàn cơ khí). Dùng đúng quy ước tên
  trường của `pid_gains_t` phía FC để dễ đối chiếu khi tune.
- **Yêu cầu bắt buộc — chuyển setpoint mượt**: khi đổi nguồn setpoint (waypoint ↔ landing target)
  **không reset tích phân về 0 đột ngột** và **không nhảy setpoint tức thời** — ramp 0.5–1 s.
- **Phần logic thuần**: lớp `PID` (có anti-windup, kẹp ngõ ra) tách rời để `pytest`.
- **Cảnh báo an toàn**: **không bao giờ tune PID lần đầu trên drone thật** — tune trong Gazebo trước,
  bay thật thì buộc dây/lồng an toàn, tăng `kp` tới khi dao động nhẹ rồi lùi 30–50%.
- **Phiên này**: **Stub**.

### 4.4 `drone_mission`

#### `mission_manager_node` — **tự viết** (trái tim logic)

- **Trách nhiệm**: máy trạng thái hiện thực đúng `mission_state_e`.
- **Vào**: `/mission/plan` (`MissionPlan`), `/mavros/state`, `/mavros/battery`, `/odometry/filtered`,
  `/landing_target/lost`, `/marker/tracking_quality`, `/gripper/status`, `/failsafe_event`.
- **Ra**: `/mission/state` (`MissionState`, EVENT_QOS, 5 Hz đều bằng timer),
  `/mission/expected_marker_id` (`Int32`, latched-style EVENT_QOS), `/mission/setpoint`,
  `/gripper/command`; gọi service của `fc_command_bridge_node`.
- **Tham số**: `search_timeout_s=20.0`, `max_retries=3`, `acceptance_radius_m=1.5`,
  `pre_dropoff_settle_s=2.0` (giữ ổn định trước khi mở gripper, tránh thả khi còn dao động),
  `takeoff_alt_m=5.0`.
- **Ba nguyên tắc bắt buộc**:
  1. **Không bao giờ** chuyển sang hạ cánh chỉ dựa vào GPS/EKF — luôn chờ `landing_target_bridge_node`
     xác thực đúng ID marker trước ("hai lớp định vị bổ trợ");
  2. `MARKER_SEARCH` có `retry_count`/`max_retries` — hết lượt thì `RETRY_LOITER` rồi báo failsafe,
     không lặp vô hạn;
  3. là nơi **duy nhất** ra lệnh `ACTION_PICKUP`/`ACTION_DROPOFF`, và **phải** chờ
     `gripper_status.sensor_confirmed == true` mới cho rời điểm — **tuyệt đối không dùng timeout thay
     cho xác nhận cảm biến thật**.
- **Phần logic thuần**: lớp `MissionFsm` không phụ thuộc `rclpy` (nhận snapshot input, trả state +
  action), để `pytest` được mọi nhánh lỗi trước khi bay.
- **Phiên này**: **Stub** (bảng chuyển trạng thái khai sẵn, thân mỗi trạng thái để `TODO`).

#### `gripper_controller_node` — **tự viết**

- **Trách nhiệm**: điều khiển cơ cấu gắp/thả và là **nguồn xác nhận cứng duy nhất** rằng hàng đã gắp/thả.
- **Vào**: `/gripper/command` (`GripperCommand`, EVENT_QOS).
- **Ra**: `/gripper/status` (`GripperStatus`, EVENT_QOS, 10 Hz đều bằng timer).
- **Phần cứng**: servo PWM + công tắc hành trình qua `pigpio` (chính xác hơn `RPi.GPIO` phần mềm thuần).
- **Tham số**: `servo_gpio=18`, `confirm_switch_gpio=23`, `open_pulse_us=1000`, `close_pulse_us=2000`
  (**đo tay theo cơ cấu thật trước khi chốt**), `confirm_debounce_ms=50`, `force_threshold_n=1.0`.
- **Điểm dễ sai**: hàng nhẹ/lệch có thể không kích hoạt công tắc dù đã gắp → cần cảm biến lực bổ sung,
  không chỉ cờ boolean.
- **Phiên này**: **Stub**, có chế độ `simulate=true` để chạy được khi không có `pigpio`/phần cứng.

#### `fc_command_bridge_node` — **tự viết**

- **Trách nhiệm**: điểm **duy nhất** gọi API MAVROS — tập trung một chỗ để dễ log và validate, thay vì
  các node rải rác tự gọi `/mavros/cmd/arming`, `/mavros/set_mode`.
- **Cung cấp**: service `~/arm`, `~/takeoff`, `~/goto_waypoint`, `~/simple_command`.
- **Gọi ra**: `/mavros/cmd/arming`, `/mavros/cmd/takeoff`, `/mavros/set_mode`, `/mavros/setpoint_raw/local`.
- **Tham số**: `ack_timeout_s=5.0`, `setpoint_rate_hz=20.0`.
- **Bắt buộc**: gắn `seq` tăng dần cho mỗi lệnh, chờ ACK trong `ack_timeout_s`, **báo lỗi rõ ràng thay
  vì treo vô hạn** khi FC không trả lời.
- **Rủi ro lớn nhất của cả hệ thống**: firmware FC là **tuỳ biến**, không phải PX4/ArduPilot nguyên bản.
  Phải xác minh sớm firmware đã hiện thực đủ `COMMAND_LONG` + `MAV_CMD_COMPONENT_ARM_DISARM`,
  `SET_POSITION_TARGET_LOCAL_NED`, và `HEARTBEAT` có `base_mode`/`custom_mode` hợp lệ. Nếu lệch,
  node này bọc `pymavlink` thô cho riêng lệnh đó thay vì tin plugin MAVROS.
- **Tiêu chí xong**: `ros2 service call` từng service chạy đúng; **rút dây FC giữa chừng → service trả
  lỗi trong `ack_timeout_s`, không treo**.
- **Phiên này**: **Stub**.

### 4.5 `drone_comms`

#### `mavros` — dùng gói có sẵn (không viết code)

- **Cấu hình** `mavros.yaml`: `fcu_url="/dev/ttyUSB0:921600"` (khớp baudrate MAVLink phía H743),
  và **`plugin_allowlist`** chỉ bật: `sys_status`, `setpoint_position`, `setpoint_raw`, `command`,
  `imu`, `battery`, `landing_target`, `local_position`, `global_position`.
- **Lý do dùng allowlist**: giảm tải CPU Pi 4, và tránh plugin ngầm gửi lệnh mà firmware tuỳ biến không
  hiểu (log lỗi liên tục, xấu nhất là hành vi ngoài ý muốn).
- **Điều kiện cần trước mọi việc khác**: `ros2 topic echo /mavros/state` thấy `connected: true`.

#### `gcs_link_node` — **tự viết**

- **Trách nhiệm**: kênh liên lạc riêng Pi 4 ↔ GCS qua 4G/LTE (UDP), độc lập USART3 của FC.
- **Vào/Ra**: socket UDP mang MAVLink dialect tuỳ biến ↔ `/mission/plan` (ra, EVENT_QOS),
  `/telemetry/outgoing` (vào), `/gcs_link/connected` (`std_msgs/Bool`, ra, EVENT_QOS).
- **Tham số**: `gcs_host`, `gcs_port=14550`, `local_port=14551`, `link_timeout_s=5.0`,
  `heartbeat_interval_s=1.0`.
- **Hai nguyên tắc bắt buộc**:
  1. lệnh khẩn cấp (RTH, hạ cánh khẩn, huỷ nhiệm vụ) đi **hàng đợi ưu tiên riêng**
     (`emergency` > `mission_ack` > `telemetry`), không xếp sau telemetry thường;
  2. **tự** phát hiện mất kết nối bằng watchdog — không đợi GCS báo, vì lúc mất kết nối GCS không báo
     được gì cả.
- **Phiên này**: **Stub** (khung hàng đợi ưu tiên + watchdog; phần mã hoá MAVLink tuỳ biến để `TODO`
  vì `pymavlink` chưa cài và dialect chưa được định nghĩa ở tài liệu nào hiện có).

#### `telemetry_aggregator_node` — **tự viết**

- **Trách nhiệm**: gộp nhiều nguồn thành đúng một gói `TelemetryPacket`, tách khỏi logic mã hoá của `gcs_link_node`.
- **Vào**: `/mission/state`, `/mavros/state`, `/mavros/battery`, `/mavros/global_position/global`,
  `/gripper/status`, `/marker/tracking_quality`, `/failsafe_event`.
- **Ra**: `/telemetry/outgoing` (EVENT_QOS), **tần số cố định bằng timer 2 Hz** — không publish theo sự
  kiện của từng nguồn, tránh làm ngập kênh 4G băng hẹp.
- **Phiên này**: **Stub**.

### 4.6 `drone_safety`

#### `failsafe_monitor_node` — **tự viết**

- **Trách nhiệm**: giám sát tập trung, là nơi **duy nhất** được **chủ động** yêu cầu đổi trạng thái
  nhiệm vụ khi có sự cố — không chỉ cảnh báo suông rồi chờ node khác tự quyết.
- **Vào**: `/mavros/battery`, `/mavros/state`, `/gcs_link/connected`, `/landing_target/lost`,
  `/ekf/health`, `/gripper/status`, `/mission/state`.
- **Ra**: `/failsafe_event` (`FailsafeEvent`, **EVENT_QOS — không được rớt gói**); gọi service của
  `fc_command_bridge_node` để leo thang **loiter → RTH → hạ cánh khẩn cấp**.
- **Cấu trúc bắt buộc**: kiểm tra **định kỳ bằng timer** (2 Hz), **không** kiểm theo callback lẻ tẻ —
  để không sự cố nào lọt chỉ vì không có sự kiện mới kích hoạt kiểm tra.
- **Tham số (bắt buộc khớp bảng `system_config` phía GCS)**: `low_battery_pct=25.0`,
  `critical_battery_pct=15.0`, `link_lost_timeout_s=10.0`, `marker_search_timeout_s=20.0`,
  `fc_comm_timeout_s=3.0`, `max_retries=3`. Lệch ngưỡng hai bên gây hoang mang người vận hành
  (GCS nghĩ pin còn an toàn trong khi Pi 4 đã kích RTH).
- **Tiêu chí xong**: test **từng nhánh một** bằng cách giả lập (rút dây FC, che marker, rút anten 4G,
  `ros2 topic pub` pin giả thấp) — đúng thứ tự leo thang, không nhảy thẳng bước nặng nhất, không sót nhánh.
- **Phiên này**: **Stub** (khung timer + bảng ngưỡng đủ; thân `raise_failsafe` để `TODO`).

#### `diagnostics_node` — dùng `diagnostic_updater` + `diagnostic_aggregator` (không viết node riêng)

- **Cách làm**: mỗi node quan trọng (`camera_node`, `marker_quality_node`, `optical_flow_node`,
  `ekf_health_node`, `mavros`) **tự** publish `DiagnosticStatus` của mình lên `/diagnostics`, thay vì để
  `failsafe_monitor_node` đoán qua dữ liệu gián tiếp. `aggregator.yaml` gộp thành cây trạng thái tổng,
  dùng thẳng cho màn hình "Quản trị hệ thống" phía GCS.
- **Cần cài thêm**: `ros-jazzy-diagnostic-aggregator` (hiện **chưa có** trên máy).
- **Phiên này**: `aggregator.yaml` + móc `diagnostic_updater` trong các node stub.

#### `mission_logger_node` — **tự viết** + `rosbag2`

- **Hai loại log, không gộp**: `rosbag2` ghi thô để debug kỹ thuật; node tự viết ghi log **nghiệp vụ**
  có cấu trúc để đối chiếu với CSDL GCS (bảng `mission`, `mission_waypoint`, `mission_photo`).
- **Vào**: `/mission/state`, `/failsafe_event`, `/marker/tracking_quality`, `/gripper/status`,
  `/camera/image_raw` (chỉ chụp ảnh xác nhận tại thời điểm gắp/thả).
- **Ra**: file JSON-lines `~/drone_logs/<mission_id>/events.jsonl` + ảnh `photos/<seq>_<action>.jpg`.
- **Lưu ý Pi 4**: ghi `rosbag2` kèm ảnh thô full-size tốn I/O thẻ SD đáng kể trên chuyến bay dài —
  dùng thẻ U3/A2 hoặc SSD qua USB 3.0, và/hoặc chỉ ghi ảnh nén.
- **Phiên này**: **Stub**.

### 4.7 `drone_bringup`

Không có node, chỉ launch + toàn bộ YAML tham số. Bốn launch theo lộ trình 5 giai đoạn:

| Launch | Nội dung | Ứng với giai đoạn |
|---|---|---|
| `perception.launch.py` | camera_ros + rectify + apriltag + marker_quality + optical_flow | GĐ 1 |
| `estimation.launch.py` | perception + mavros + ekf + marker_pose_republisher + ekf_health | GĐ 1 |
| `control.launch.py` | estimation + landing_target_bridge + position_controller | GĐ 2 |
| `full_system.launch.py` | tất cả + mission + comms + safety | GĐ 3–5 |

File YAML tập trung tại `drone_bringup/config/`: `camera.yaml`, `apriltag.yaml`, `ekf.yaml`,
`mavros.yaml`, `control.yaml`, `mission.yaml`, `comms.yaml`, `safety.yaml`, `aggregator.yaml`.

## 5. Trình tự thi công phiên này

1. `drone_interfaces` (đầy đủ) → build → `ros2 interface show` kiểm chứng.
2. Sáu package node stub, mỗi package build ngay sau khi tạo.
3. `drone_bringup` (launch + YAML) → build cả workspace.
4. Kiểm chứng cuối: `colcon build` sạch toàn workspace; `ros2 pkg list | grep drone`;
   khởi động thử vài node không cần phần cứng.

## 6. Việc còn nợ sau phiên này

- **Cài gói thiếu**: `ros-jazzy-diagnostic-aggregator`, `python3-pigpio` + `pigpiod`, `pymavlink`.
- **Điền thuật toán** cho 11 node stub, theo đúng thứ tự 8 bước ở mục 8 tài liệu hướng dẫn.
- **Hiệu chỉnh camera** (`camera_calibration`) — chặn toàn bộ nhánh marker nếu chưa làm.
- **Đối chiếu tên trường** message với `thiet_ke_cau_truc_du_lieu_he_thong_drone.md` khi có file đó.
- **Xác minh firmware FC** với từng plugin MAVROS trước khi tin lớp trên — rủi ro tương thích lớn nhất.
- **Định nghĩa MAVLink dialect tuỳ biến** cho `gcs_link_node` (chưa tài liệu nào mô tả khung gói này).
