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
| `position_controller_node`, `mission_manager_node`, `fc_command_bridge_node`, `failsafe_monitor_node` | **Chạy nguyên code thật** |

**Giới hạn cần nhớ:** gain chỉ dùng được cho drone thật khi vòng vận tốc của X3 phản ứng giống vòng
vận tốc của FC. Hằng số thời gian τ của FC chưa đo (cần có điện động cơ, 12.B). Coi gain tune ở đây
là **điểm xuất phát an toàn**, không phải con số cuối.

Khác FC thật (có chủ đích): quyền điều khiển luôn có sẵn (không có ch8). DISARM trả lại quyền sau
`KHOA`. Chưa giả lập camera/tag.

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
| `launch/sim_mission.launch.py` | Thêm `fc_command_bridge_node`, `mission_manager_node`, `ekf_health_node`, `failsafe_monitor_node` |
