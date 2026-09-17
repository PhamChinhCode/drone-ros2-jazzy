# Hướng dẫn build và chạy — Pi (drone-ros2-jazzy)

Tham khảo nhanh, gộp từ [`README.md`](../README.md) và những gì đã xác nhận chạy thật (kể cả trên
máy phát triển Ubuntu 24.04 không phải Pi 4 — mô phỏng Gazebo chạy được trên bất kỳ máy x86_64 nào).
Song sinh với [`../../GrounControlStation/docs/HUONG_DAN_BUILD_CHAY.md`](../../GrounControlStation/docs/HUONG_DAN_BUILD_CHAY.md)
phía GCS — hai tài liệu tách riêng vì build/chạy là việc nội bộ mỗi bên (giao ước
[`GIAO_UOC_GCS_PI.md`](GIAO_UOC_GCS_PI.md) mục 0.4 chỉ nói byte trên dây, không nói cách build).

## 0. Yêu cầu

- Ubuntu 24.04 + ROS 2 Jazzy. Cài đặt lần đầu: [`docs/huong_dan_cai_dat_ros2_pi4.md`](huong_dan_cai_dat_ros2_pi4.md),
  [`docs/cai_dat_ros2_framework_pi4.md`](cai_dat_ros2_framework_pi4.md).
- `pymavlink` **ghim đúng 2.4.49**, khớp phía GCS — bộ sinh mã dialect là một phần của hợp đồng
  (giao ước 7.1). Cài: `sudo pip3 install --break-system-packages pymavlink==2.4.49`.
- Chạy mô phỏng (Gazebo) cần thêm `ros-jazzy-ros-gz` — không cần phần cứng drone/camera thật.

## 1. Build

**Tắt stack đang chạy trước** (mục 1b). Rồi:

```bash
cd ~/drone-ros2-jazzy
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
```

**Lỗi `failed to create symbolic link ... existing path cannot be removed: Is a directory`** nghĩa là
`build/` được tạo bởi một lần build **không** có `--symlink-install` (hai chế độ không trộn được). Xoá
thư mục sinh ra rồi build lại từ đầu — cả ba đều gitignore, không mất gì (gặp thật 2026-09-17):

```bash
rm -rf build install log
colcon build --symlink-install
```

Kiểm đã đúng chế độ symlink: `ls -la install/drone_bringup/share/drone_bringup/config/tags.yaml` phải
là liên kết (`->`), không phải file thường.

`--symlink-install` để sửa file Python trong `src/` có hiệu lực ngay, không phải build lại mỗi lần —
**trừ** file mới tạo hoặc thay đổi trong `package.xml`/`setup.py`, vẫn cần build lại. `ros2 launch`
đọc từ `install/`, **không** đọc từ `src/` — quên build là nguyên nhân "sửa rồi mà không thấy đổi"
phổ biến nhất.

Build riêng một gói khi chỉ sửa gói đó (nhanh hơn build toàn bộ):

```bash
colcon build --packages-select drone_comms drone_mission --symlink-install
```

## 1b. Tắt stack cũ trước khi build và chạy lại

**Không chạy bộ mới khi bộ cũ còn sống.** ROS 2 không chặn hai node trùng tên, chỉ cảnh báo — nên bộ
cũ vẫn chạy code cũ song song với bộ mới: `gcs_link_node` mới không mở được UDP `14551` (*Address already
in use*) nên **GCS vẫn nói chuyện với node cũ**; hai `position_controller_node` cùng phát setpoint; hai
nguồn `/odometry/filtered` làm vị trí nhảy; MAVROS mới tranh cổng serial với MAVROS cũ. Log chỉ báo lỗi
ở vài node, phần còn lại trông như chạy bình thường.

```bash
# Ctrl+C trong terminal đang chạy ros2 launch, hoặc:
pkill -INT -f "ros2 launch"
# mô phỏng: Gazebo hay sót lại
pkill -9 -f "gz sim"
# kiểm đã sạch - danh sách phải TRỐNG:
ros2 daemon stop && ros2 daemon start && ros2 node list
```

Trên drone thật dùng `-INT` (như Ctrl+C) trước, `-9` sau cùng: tắt cứng thì `ros2 bag record` không kịp
đóng file. FC còn armed thì disarm trước khi tắt.

## 2. Sinh mã dialect MAVLink (chỉ khi `docs/mavlink/drone_gcs.xml` đổi)

`src/drone_comms/drone_comms/dialect_gcs.py` là mã **sinh ra** (~24 nghìn dòng), gitignore, không
sửa tay. Nguồn duy nhất là XML (giao ước 7.1):

```bash
python3 tools/sinh_dialect.py
colcon build --packages-select drone_comms --symlink-install   # de goi cai lai ban moi vao install/
```

Sửa XML thì sửa **ở đây trước** (`drone-ros2-jazzy` là bản gốc duy nhất — giao ước mục 0.2), rồi
chép `docs/mavlink/drone_gcs.xml` sang `GrounControlStation/backend/gcs_backend/link_mav/drone_gcs.xml`
và sinh mã lại **cả hai bên**.

## 3. Chạy — mô phỏng (Gazebo, không cần phần cứng)

```bash
# Bay tay chỉnh gain PID, chưa có nhiệm vụ:
ros2 launch drone_sim sim_tune.launch.py
ros2 launch drone_sim sim_tune.launch.py gui:=false              # chỉ server, máy yếu
ros2 launch drone_sim sim_tune.launch.py render_engine:=ogre2    # máy ảo/GPU yếu thì đổi engine

# Nhiệm vụ đầy đủ qua kênh GCS (mission_manager_node, gcs_link_node, EKF, failsafe...):
ros2 launch drone_sim sim_mission.launch.py
ros2 launch drone_sim sim_mission.launch.py tag_noise_m:=0.02 tag_dropout:=0.1   # nhiễu/mất khung
```

**Kiểm `sim_mission` đã lên đủ** (terminal khác, đã `source install/setup.bash`):

```bash
ros2 node list                                  # 13 node, xem danh sách dưới
ros2 topic hz /odometry/filtered                # ~40 Hz
ros2 topic echo --once /telemetry/outgoing      # tagmap_crc khác 0, contract_ver = 600
```

13 node: `ekf_health_node`, `failsafe_monitor_node`, `fc_command_bridge_node`, `gcs_link_node`,
`gripper_controller_node`, `gz_bridge`, `landing_target_bridge_node`, `marker_pose_republisher_node`,
`mission_manager_node`, `position_controller_node`, `sim_fc_bridge_node`, `sim_tag_node`,
`telemetry_aggregator_node`. **Không có MAVROS và `ekf_filter_node` là đúng** — trong mô phỏng
`sim_fc_bridge_node` thay cả hai.

Cảnh báo **bình thường** khi chưa có GCS nối vào và chưa nạp kế hoạch — không phải lỗi:
`FAILSAFE 3 -> leo thang 3: mat GCS 10 s`, `CHAY KHONG CHU KY (signing_required = false)`,
`/mission/state im qua 1.0 s - NGUNG phat setpoint`, và `EKF KHONG healthy: chua nhan /odometry/filtered`
ở giây đầu tiên. Cái cần tìm là `ERROR`, `Traceback`, `process has died`.

Gửi thử kế hoạch nội bộ ROS (không cần GCS thật):

```bash
ros2 run drone_mission send_mission_plan $(ros2 pkg prefix drone_sim)/share/drone_sim/config/missions/sim_tag1.yaml
ros2 service call /mission_manager_node/start std_srvs/srv/Trigger
```

Hoặc thử **qua đúng kênh MAVLink** như GCS thật sẽ làm, không cần chạy GCS thật:

```bash
python3 tools/gcs_sim.py                                    # chỉ nghe, in telemetry
python3 tools/gcs_sim.py --plan config/missions/ban_home.yaml --cmd start
python3 tools/gcs_sim.py --cmd rtl                           # rtl / land / start / disarm / abort / pause
```

## 4. Chạy — phần cứng thật

```bash
scripts/camera_v4l2_setup.sh --width 640 --height 400 --vblank 3957 --exposure 300 --gain 32
source install/setup.bash
ros2 launch drone_bringup full_system.launch.py
```

**Camera phải cấu hình lại V4L2 sau MỖI lần reboot** — bỏ qua thì topic vẫn ra đúng nhịp nhưng mọi
khung hình toàn số 0, `ros2 topic hz` không phát hiện được. `scripts/drone_startup.sh` làm việc này
tự động trước khi source ROS. Xem trực tiếp: Foxglove → `ws://<ip-drone>:8765`.

Tự chạy lúc boot: xem [`README.md` mục "Tự chạy lúc boot"](../README.md#tự-chạy-lúc-boot).

## 5. Kết nối tới GCS ở máy khác (không cùng LAN)

`src/drone_bringup/config/comms.yaml` → `gcs_host` phải là địa chỉ **GCS gọi được từ máy Pi này**.
Thiết lập hiện tại (Pi mô phỏng trên Ubuntu, GCS trên máy Windows khác) nối qua **Tailscale**:
`gcs_host` phải là IP Tailscale của máy GCS (`tailscale ip -4` chạy trên máy đó), không phải IP LAN.

`signing_required` trong `comms.yaml` phải **khớp `GCS_SIGNING` phía GCS** — lệch nhau thì liên kết
**im lặng không lên**, không lỗi hiển thị (giao ước 7.6). Sinh khoá khi cần bật chữ ký:

```bash
python3 tools/tao_khoa_gcs.py            # mặc định ghi ra ~/.drone_gcs_key, quyền 600
# chép ĐÚNG khoá này sang phía GCS bằng đường an toàn (scp/USB) - không qua chat/email/git
```

## 6. Kiểm thử

Test thuần Python (không cần ROS chạy, nhưng cần môi trường đã source để import được các package):

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
python3 -m pytest src/drone_comms/test/ src/drone_mission/test/ src/drone_control/test/ \
                  src/drone_safety/test/ src/drone_estimation/test/ src/drone_sim/test/ \
                  src/drone_perception/test/ -q
# 2026-09-17: 184 passed
```

Hoặc theo cách chuẩn ROS 2 (chạy cả linter):

```bash
colcon test --packages-select drone_comms drone_mission
colcon test-result --verbose
```

## 7. Nạp bản đồ tag qua dây (giao ước 8.7)

Từ bản 0.6, GCS nạp toạ độ tag qua UDP thay vì chép tay `tags.yaml`. Phía Pi: `gcs_link_node` ghi
bản đã nhận ra `~/.config/drone_ros2_jazzy/tags_override.yaml` (**ngoài** cây `install/`, không bị
`colcon build` ghi đè) — bốn launch file (`control`, `estimation`, `full_system`, `sim_launch`, nên
cả `sim_mission.launch.py`) tự ưu tiên đọc file đó nếu tồn tại, thay cho `config/tags.yaml` đóng gói
sẵn.

**Sau khi GCS báo đã nạp (`ACCEPTED`), phải khởi động lại stack** (`ros2 launch` lại) — bốn node đọc
bản đồ (`marker_pose_republisher_node`, `landing_target_bridge_node`, `mission_manager_node`,
`telemetry_aggregator_node`) chỉ đọc **đúng một lần lúc khởi động**, chưa hỗ trợ nạp nóng.

Muốn quay lại dùng bản đồ đóng gói trong repo: xoá file override rồi khởi động lại —
```bash
rm ~/.config/drone_ros2_jazzy/tags_override.yaml
```

Nạp qua dây **chỉ đổi được toạ độ tag đã khai** trong `src/drone_bringup/config/apriltag.yaml`
(`tag.ids`/`tag.frames`/`tag.sizes`). Thêm tag mới vẫn phải sửa file đó bằng tay + `colcon build` +
khởi động lại — `apriltag_ros` bỏ qua hoàn toàn tag không nằm trong `tag.frames`. Chi tiết đầy đủ:
[`GIAO_UOC_GCS_PI.md` mục 8.7](GIAO_UOC_GCS_PI.md#87-nạp-bản-đồ-tag-qua-dây--thoả-thuận-hiện-thực-xong-cả-hai-bên-đã-chạy-thật-trên-dây-87).

## 8. Gỡ lỗi trên dây

```bash
sudo tcpdump -i any -n udp port 14550 or udp port 14551 -w /tmp/pi.pcap
```

## 9. Việc hay vấp

| Triệu chứng | Nguyên nhân thường gặp |
|---|---|
| Sửa code mà chạy không thấy đổi | Quên `colcon build` — `ros2 launch` đọc `install/`, không đọc `src/`; **hoặc bộ cũ vẫn chạy song song** (mục 1b) |
| `colcon build` báo `failed to create symbolic link ... Is a directory` | Build cũ không có `--symlink-install` — `rm -rf build install log` rồi build lại (mục 1) |
| GCS vẫn nhận `tagmap_crc`/hành vi cũ sau khi chạy lại; log `Address already in use` | `gcs_link_node` cũ còn sống giữ cổng `14551` — tắt hết theo mục 1b |
| Camera ra đúng nhịp nhưng ảnh toàn đen/0 | Quên chạy lại `camera_v4l2_setup.sh` sau reboot |
| Liên kết GCS↔Pi không bao giờ lên, không lỗi | `gcs_host` sai IP, hoặc `signing_required` lệch nhau hai bên |
| `ImportError: No module named 'drone_comms'` khi chạy pytest trực tiếp | Chưa `source install/setup.bash` (hoặc `/opt/ros/jazzy/setup.bash`) trước |
| Thiếu module `dialect_gcs`/`drone_gcs` | Chưa chạy `tools/sinh_dialect.py` (Pi) / `gen_dialect.py` (GCS) sau khi tải mã nguồn mới hoặc XML đổi |
