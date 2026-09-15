# drone_sim — tune bay waypoint trong Gazebo

Chạy trên **PC** (Ubuntu 24.04 + ROS 2 Jazzy + Gazebo Harmonic). **Không chạy trên Pi 4**: không GPU,
thiếu RAM. Mô phỏng này giúp chỉnh gain `cruise.*` của `position_controller_node`, là vòng vị trí Pi đặt
lên trên vòng vận tốc của FC.

## 0. Mô phỏng thay cái gì, giữ cái gì

| Thật | Trong mô phỏng |
|---|---|
| FC STM32 (vòng góc + vòng vận tốc OFFBOARD) | Plugin `MulticopterVelocityControl` của Gazebo (drone X3) |
| MAVROS + hợp đồng FC (ARM/DISARM, `OB_*`, hết hạn 500 ms, laser) | `sim_fc_bridge_node` + `sim_fc.py` |
| EKF (`robot_localization`) | Vị trí thật từ Gazebo, thêm trễ/nhiễu tuỳ chọn |
| Camera + `image_proc` + `apriltag_ros` | `sim_tag_node` tính pose tag từ ground truth |
| Servo gripper + công tắc hành trình | `gripper_controller_node` với `simulate:=true` |
| `position_controller_node`, `mission_manager_node`, `fc_command_bridge_node`, `failsafe_monitor_node` | **Chạy nguyên code thật** |

**Giới hạn cần nhớ:** gain chỉ dùng được cho drone thật khi vòng vận tốc của X3 phản ứng giống vòng
vận tốc của FC. Hằng số thời gian τ của FC chưa đo (cần có điện động cơ, 12.B). Coi gain tune ở đây
là **điểm xuất phát an toàn**, không phải con số cuối.

Khác FC thật (có chủ đích): quyền điều khiển luôn có sẵn (không có ch8). DISARM trả lại quyền sau
`KHOA`.

## 1. Cài đặt (một lần)

```bash
# ROS 2 Jazzy đã cài (ros-jazzy-desktop). Thêm Gazebo Harmonic + cầu và các gói message:
sudo apt install ros-jazzy-ros-gz ros-jazzy-mavros-msgs ros-jazzy-apriltag-msgs \
    ros-jazzy-tf2-geometry-msgs ros-jazzy-cv-bridge python3-opencv python3-yaml \
    python3-colcon-common-extensions
git clone https://github.com/PhamChinhCode/drone-ros2-jazzy.git ~/drone_ws && cd ~/drone_ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-up-to drone_sim      # chỉ build các gói mô phỏng cần, bỏ qua camera/MAVROS
source install/setup.bash
```

Không cần cài MAVROS, apriltag_ros, robot_localization hay v4l2_camera trên PC.

Lần chạy Gazebo đầu tiên cần Internet để tải model X3 từ Gazebo Fuel.

## 1b. BẮT BUỘC: tách miền ROS khỏi drone thật

Mô phỏng dùng **đúng tên topic/service của MAVROS** (`/mavros/setpoint_raw/local`,
`/mavros/state`, `/odometry/filtered`, service `/mavros/cmd/command`). ROS 2 tự phát hiện node
qua multicast trên LAN, nên nếu PC và Pi cùng `ROS_DOMAIN_ID` (mặc định 0) thì **hai hệ trộn vào
nhau**: FC giả đè lên dữ liệu FC thật, `position_controller_node` trên Pi nhận vị trí drone ảo,
và setpoint của mô phỏng đi thẳng xuống FC thật qua MAVROS.

Đặt **cả hai** biến trước khi chạy bất cứ lệnh nào của `drone_sim`:

```bash
export ROS_DOMAIN_ID=42                      # khac mien voi drone that
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST   # chan tim node ra ngoai may nay
```

Kiểm tra trước khi tune — `ros2 node list` **chỉ được** thấy `/gz_bridge`,
`/sim_fc_bridge_node`, `/position_controller_node`. Thấy `/mavros/*`, `/ekf_filter_node` hay
`/marker_detector_node` là đang dính vào drone thật, **dừng ngay**.

Dấu hiệu đã bị trộn: `ros2 topic echo /mavros/debug_value/named_value_int` cho **hai dãy
`OB_RX_OK` xen kẽ** với giá trị cách xa nhau.


## 2. Chạy mô phỏng tune

```bash
ros2 launch drone_sim sim_tune.launch.py            # thêm gui:=false nếu PC yếu
```

Mặc định dùng renderer Ogre1 (`render_engine:=ogre`): Ogre2 làm màn hình 3D nhấp nháy trên máy ảo.
PC thật có GPU tốt muốn dùng Ogre2 thì thêm `render_engine:=ogre2`.

Kiểm tra trước khi tune (terminal khác, nhớ `source install/setup.bash`):

```bash
ros2 topic hz /odometry/filtered                    # ~50 Hz
ros2 topic hz /mavros/setpoint_raw/local            # 20 Hz
gz topic -l | grep X3                               # /X3/gazebo/command/twist, /X3/enable, /model/X3/odometry
```

Không có odometry thì model X3 chưa tải được hoặc tên topic đã khác. Đối chiếu với `gz topic -l`.

**Kiểm dấu (bắt buộc, làm một lần):**

```bash
ros2 param set /position_controller_node cruise.z.kp 0.5
ros2 run drone_sim step_test --axis z --step 1.5    # phải BAY LÊN, sai số cuối nhỏ dần
ros2 param set /position_controller_node cruise.x.kp 0.5
ros2 run drone_sim step_test --axis x --step 1.0    # phải bay TỚI TRƯỚC (x tăng)
```

Bay ngược chiều thì dừng lại, đừng tune tiếp: quy ước khung thân của plugin khác giả định.

## 3. Quy trình tune `cruise.*`

Gain đổi ngay khi đang chạy (`ros2 param set`), tích phân được giữ nguyên. Giá trị âm hoặc nan bị từ chối.

`step_test` in ra: thời gian lên (10→90 %), % vượt đích, thời điểm ổn định (vào dải ±`--band`
lần cuối), sai số cuối, % thời gian lệnh ngang bị kẹp ở 1,9 m/s.

**Thứ tự: trục z trước, rồi x, rồi y (dùng lại gain của x).**

1. `ki = kd = 0`. Cho `kp` = 0,3 → 0,5 → 0,7 → … Mỗi mức chạy:
   `ros2 run drone_sim step_test --axis z --step 2.0 --back`
2. Dừng tăng khi **vượt đích > 20 %** hoặc bắt đầu dao động. Lấy `kp` đó **lùi lại 30–50 %**.
   - Đầu ra là vận tốc, vòng gần như bậc một: thường **chỉ cần `kp`**. Thời gian đáp ứng ≈ `1/kp` giây.
   - Bước lớn sẽ bị kẹp tốc độ (1,9 m/s ngang). Đánh giá `kp` bằng bước nhỏ, khoảng 1 m, không bão hoà.
3. Còn vượt đích thì thêm `kd` nhỏ (0,05–0,2). Không nên lớn: D khuếch đại nhiễu EKF.
4. Chỉ thêm `ki` khi còn sai số tĩnh. Giữ `i_limit` 0,3–0,5 để bước dài không dồn tích phân.
5. **Tiêu chí đạt** (bước 1 m và 2 m, cả hai chiều): vượt đích < 10 %, ổn định trong ±0,1 m,
   dưới ~3 s sau khi hết bão hoà, không dao động.
6. **Kiểm độ bền** với EKF thật hơn: chạy lại bước 5 bằng
   `ros2 launch drone_sim sim_tune.launch.py odom_delay_s:=0.1 odom_noise_m:=0.03`.
   Dao động thì giảm `kp` và `kd`.

Ghi lại để vẽ (PlotJuggler/Foxglove):
`ros2 bag record /odometry/filtered /mission/setpoint /mavros/setpoint_raw/local`.

Chép gain đạt vào `src/drone_bringup/config/control.yaml`.

## 3b. Giả lập tag — `sim_tag_node`

Thay cả chuỗi camera → `image_proc` → `apriltag_ros` mà không cần camera: lấy vị trí thật của drone
trong Gazebo, tính pose tag trong khung ảnh, rồi phát đúng hai thứ `landing_target_bridge_node` cần —
`/apriltag/detections` và TF `camera_optical_frame → <tên khung tag>`. Dùng **đúng chuỗi TF của drone
thật** (`base_link → camera_link → camera_optical_frame`, số đo lấy từ `estimation.launch.py`), nên
phép `lookupTransform` của bridge đi qua y hệt đường thật, kể cả sai số do lắp camera nghiêng 20°.

Bãi đáp trong `worlds/drone_tune.sdf` (`pad_home` ở 0,0 và `pad_a` ở 10,0) đã trùng khớp `tags.yaml`.

```bash
ros2 launch drone_sim sim_mission.launch.py tag_noise_m:=0.01 tag_dropout:=0.1
```

Tầm nhìn mô phỏng bằng hình chóp: nửa FOV 44,7° ngang và 32,0° dọc (suy từ hiệu chỉnh 640×400,
fx 323,62 / fy 320,33), tầm 0,15–8 m. Camera nghiêng 20° ra trước nên **drone mất tag ở độ cao thấp
nếu bay quá tag một chút** — đây là ràng buộc thật, đo được trong mô phỏng: ở z = 0,27 m và lệch 3 cm
về phía sau tag, góc tới tag là 38,4°, vượt nửa FOV dọc 32°. FSM xử lý đúng: từ chối hạ mù, leo lên
bắt lại rồi hạ tiếp.

**KHÔNG mô phỏng:** mờ ảnh khi bay nhanh, thiếu sáng, tag bị loá, sai số PnP theo góc nghiêng. Tag ở
đây **dễ thấy hơn ngoài đời**, nên gain `landing.*` tune được vẫn chỉ là điểm xuất phát.

## 3c. Tune `landing.*`

Cùng cách với `cruise.*` nhưng chỉ tiêu là **sai số chạm đất trên tag** và **số lần mất tag**:

```bash
ros2 param set /position_controller_node landing.x.kp 0.5
ros2 param set /position_controller_node landing.y.kp 0.5
ros2 run drone_mission send_mission_plan <ke_hoach co acceptance_radius_m lon>
```

Đo được (nhiễu pose tag 1 cm, mất 10 % khung, EKF trễ 0,1 s + nhiễu 3 cm):

| `landing.kp` | Sai số chạm đất | Số lần thử lại | Thời gian |
|---|---|---|---|
| 0,5 | 3 cm | 0 | 25 s |
| 1,5 | 4 cm | 2 | 30 s |
| 2,5 | 9 cm | 4 | 38 s |

Gain cao **phản tác dụng**: khuếch đại nhiễu pose tag thành chuyển động ngang, đẩy tag ra khỏi khung
hình. Khác hẳn `cruise.*` nơi gain cao chỉ gây vọt lố.

`landing.z.*` **không được đọc ở đâu**: `SOURCE_LANDING` chỉ chi phối `vx, vy`.

## 4. Thử bay waypoint qua nhiệm vụ thật

```bash
ros2 launch drone_sim sim_mission.launch.py
ros2 run drone_mission send_mission_plan $(ros2 pkg prefix drone_sim)/share/drone_sim/config/missions/sim_tag1.yaml
ros2 service call /mission_manager_node/start std_srvs/srv/Trigger
ros2 topic echo /mission/state --field detail
```

Chuỗi mong đợi: ARM → TAKEOFF (2 m) → ENROUTE tới tag 1 (x = 10 m) → MARKER_SEARCH → hết giờ
5 s → RETRY_LOITER → MARKER_SEARCH → hết lượt → EMERGENCY_LAND tại chỗ → DISARM → IDLE.

Chưa giả lập camera nên chưa hạ xuống tag được. Gain `landing.*` tune ở giai đoạn sau.

Lưu ý: `control.yaml` phải đã có gain `cruise.*` khác 0 thì ENROUTE mới bay.

## 4b. Nhiệm vụ nhiều chặng có gắp/thả

`gripper_controller_node` chạy ở `simulate:=true` giả lập hành trình servo (`sim_travel_s`, mặc
định 0,8 s) và công tắc xác nhận, đủ để chạy hết chuỗi ACTUATE_GRIPPER mà không cần phần cứng.

```yaml
# hai chang: gap tai tag 1 roi bay ve tha tai tag 0
waypoints:
  - {marker_id: 1, action: pickup,  alt_m: 2.0, acceptance_radius_m: 0.5, max_vel_mps: 1.9, loiter_s: 1.0}
  - {marker_id: 0, action: dropoff, alt_m: 2.0, acceptance_radius_m: 0.5, max_vel_mps: 1.9, loiter_s: 1.0}
```

`action` nhận **tên** (`none` / `pickup` / `dropoff`), không nhận số.

Chuỗi đầy đủ đã chạy được: ARM → TAKEOFF → ENROUTE → MARKER_SEARCH → PRECISION_LAND → **ACTUATE_GRIPPER
(giữ ổn định → gắp → chờ xác nhận cảm biến)** → TAKEOFF lại → ENROUTE tới tag 0 → … → **thả** →
hạ cánh và DISARM.

**Thử đường thất bại:** không chạy `gripper_controller_node` thì `/gripper/status` không bao giờ
tới, failsafe `FS_GRIP_CONFIRM_FAIL` bật sau 5 s và FSM thử lại cả chặng, hết lượt thì hạ tại chỗ.

## 5. Đưa gain sang drone thật — chưa làm bây giờ

Chỉ khi đã có điện động cơ (giai đoạn C):

- Dùng khoảng **50 %** gain mô phỏng.
- Bay thấp, buộc dây hoặc lồng an toàn.
- Tăng dần tới khi bắt đầu dao động nhẹ rồi lùi 30–50 %.
- Đo τ vòng vận tốc của FC (bước `vx` 0 → 0,5 m/s, xem `ODOMETRY`), chỉnh `velocityGain` của X3
  trong `worlds/drone_tune.sdf` cho khớp rồi tune lại.

## Thành phần

| File | Vai trò |
|---|---|
| `drone_sim/sim_fc.py` | FC giả theo giao ước: setpoint `frame 8 / 0x07C7`, hết hạn 500 ms → `KHOA`, ARM/DISARM (cổng 20 cm, 21196), `OB_*`, laser |
| `drone_sim/sim_fc_bridge_node.py` | Nối FC giả với Gazebo (`ros_gz_bridge`) và các topic/service MAVROS |
| `drone_sim/step_response.py`, `step_test.py` | Thử đáp ứng bước và tính chỉ số |
| `worlds/drone_tune.sdf` | X3 + `MulticopterVelocityControl` + `OdometryPublisher`, hai bãi đáp trùng `tags.yaml` |
| `launch/sim_tune.launch.py` | Gazebo + cầu + FC giả tự arm + `position_controller_node` |
| `drone_sim/sim_tag_node.py` | Giả lập phát hiện tag từ ground truth: `/apriltag/detections` + TF, thay cả camera và `apriltag_ros` |
| `launch/sim_mission.launch.py` | Thêm `fc_command_bridge_node`, `mission_manager_node`, `ekf_health_node`, `failsafe_monitor_node`, `sim_tag_node`, `landing_target_bridge_node`, `gripper_controller_node` |
