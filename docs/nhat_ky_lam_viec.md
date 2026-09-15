# Nhật ký làm việc — dựng hệ thống node ROS 2 trên Pi 4

*File ghi chép quá trình làm việc. Mỗi phiên ghi: khảo sát gì, quyết định gì, làm gì, kiểm chứng ra sao, còn nợ gì. Ghi theo thứ tự thời gian, không viết lại quá khứ.*

---

## Phiên 1 — 08/09/2026

### 1.1 Khảo sát hiện trạng (trước khi lên kế hoạch)

Đọc `docs/thiet_ke_kien_truc_node_ros2.md` (kiến trúc 6 lớp, 16 node, lộ trình 5 giai đoạn) và
`docs/huong_dan_chi_tiet_cac_node_ros2_pi4.md` (chi tiết từng node: I/O, QoS, tham số, khung code, cách test).
Hai tài liệu này thống nhất với nhau, không có mâu thuẫn cần xử lý.

Ghi chú: cả hai tài liệu đều tham chiếu tới các file không có trong `docs/`
(`phan_tich_thiet_ke_he_thong_van_chuyen.md`, `thiet_ke_cau_truc_du_lieu_he_thong_drone.md`,
`huong_dan_cai_dat_ros2_pi4.md`, `thiet_ke_mo_phong_gazebo_ros2.md`, `huong_dan_tich_hop_mavlink_H743.md`).
=> Tên trường của các struct dùng chung (`mission_plan_t`, `fc_command_t`, `gripper_status_t`,
`failsafe_type_e`...) chỉ suy ra được từ những đoạn trích trong hai tài liệu hiện có.
**Đây là rủi ro lệch tên trường giữa ba tầng FC/Pi4/GCS — cần đối chiếu lại khi có tài liệu cấu trúc dữ liệu.**

Khảo sát máy thật (`ros2_ws` chạy trên chính Pi 4):

| Hạng mục | Kết quả |
|---|---|
| Phần cứng | Raspberry Pi 4 Model B Rev 1.2, 4 nhân, RAM 3.7 GiB |
| Hệ điều hành / ROS | Linux 6.8.0-1064-raspi, ROS 2 **Jazzy**, Python 3.12.3, colcon có sẵn |
| Workspace hiện có | chỉ `src/camera_ros` (đã build vào `build/`, `install/`) |
| Camera | **không có** camera USB/CSI gắn vào (`/dev/video10+` chỉ là node codec của Pi) |
| Flight controller | **không có** thiết bị serial nào (`/dev/ttyUSB*`, `/dev/ttyACM*`, `/dev/serial/by-id` đều trống) |
| Git | workspace **không phải** git repo |

Gói ROS đã cài sẵn: `apriltag_ros`, `apriltag_msgs`, `aruco_opencv`, `mavros`, `mavros_extras`,
`mavros_msgs`, `robot_localization`, `image_proc`, `image_transport`, `cv_bridge`, `v4l2_camera`,
`camera_calibration`, `diagnostic_updater`, `rosbag2`, `tf2_ros`. Python: `cv2`, `numpy`.

Gói **còn thiếu**: `ros-jazzy-diagnostic-aggregator`, `ros-jazzy-usb-cam` (không cần nếu dùng CSI),
`python3-smach`, `pigpio`/`python3-pigpio`, `pymavlink`.

### 1.2 Quyết định đã chốt

1. **Camera: dùng `camera_ros` (Pi Camera CSI)** — vì package này đã có sẵn trong `src/` và đã build.
   Không thêm `usb_cam`. (Suy ra từ hiện trạng workspace, không phải từ tài liệu.)
2. **Marker: AprilTag (`apriltag_ros`, family `tag36h11`)** — người dùng chọn, ưu tiên độ chính xác
   góc khi drone nghiêng nhiều. Tài liệu kiến trúc để ngỏ ArUco/AprilTag nên không mâu thuẫn.
3. **Phạm vi phiên này: khung sườn đủ 8 package** — `package.xml`/`setup.py`, launch, YAML tham số,
   và node stub (đã nối dây pub/sub/param, phần thuật toán để `TODO`).
   Ngoại lệ: `drone_interfaces` được định nghĩa **đầy đủ**, vì đây là phần định nghĩa chứ không phải
   logic — mọi package khác phụ thuộc vào nó nên không thể để stub.
4. **Kênh GCS: phương án (b)** — Pi 4 có đường 4G/LTE riêng, theo đúng lựa chọn đã chốt ở mục 1
   tài liệu kiến trúc.

### 1.3 Giới hạn kiểm chứng của phiên này

Không có camera và không có FC nối vào máy => **không thể** kiểm thử theo mục "kiểm thử độc lập" của
tài liệu (đo pose marker bằng thước, `ros2 topic hz` trên ảnh thật, ACK từ FC...).
Mức kiểm chứng khả thi tối đa phiên này: `colcon build` sạch + `ros2 interface show` đọc được message
+ node khởi động được và tự tắt đúng khi thiếu nguồn dữ liệu. Ghi rõ để không nhầm là đã "chạy được thật".

### 1.4 Kế hoạch đã viết

`docs/ke_hoach_thiet_ke_node.md` — chốt 6 điểm tài liệu gốc còn để ngỏ (camera, marker, kênh GCS,
thư viện FSM, plugin landing target, ngôn ngữ), định nghĩa đầy đủ 10 message + 4 service, và đặc tả
từng node theo khuôn: trách nhiệm → I/O + QoS → tham số mặc định → điểm dễ sai → tiêu chí xong.

Kế hoạch bổ sung **3 node không có tên riêng trong tài liệu gốc** nhưng tài liệu có nêu nhu cầu:
`marker_quality_node` (mục 1.3), `marker_pose_republisher_node` và `ekf_health_node` (mục 2.1).

### 1.5 Đã triển khai

8 package mới trong `src/`. `drone_interfaces` dùng `ament_cmake`, 7 package còn lại `ament_python`.

| Package | Node tự viết | Mức hoàn thiện |
|---|---|---|
| `drone_interfaces` | — | **đầy đủ** (10 msg + 4 srv) |
| `drone_perception` | `optical_flow_node`, `marker_quality_node` | stub |
| `drone_estimation` | `marker_pose_republisher_node`, `ekf_health_node` | stub |
| `drone_control` | `landing_target_bridge_node`, `position_controller_node` | stub (lớp `PID` **viết đầy đủ**) |
| `drone_mission` | `mission_manager_node`, `gripper_controller_node`, `fc_command_bridge_node` | stub (bảng `TRANSITIONS` của FSM **viết đầy đủ**) |
| `drone_comms` | `gcs_link_node`, `telemetry_aggregator_node` | stub |
| `drone_safety` | `failsafe_monitor_node`, `mission_logger_node` | stub |
| `drone_bringup` | — | **đầy đủ** (4 launch + 11 YAML) |

"Stub" ở đây nghĩa là: đã khai đủ tham số, đã tạo đủ publisher/subscriber/service/timer đúng QoS đã
chốt, node khởi động và chạy ổn định; chỉ phần thân thuật toán để `TODO` kèm mô tả chi tiết việc cần làm.

Logic thuần đã tách sẵn ra file riêng để `pytest` được không cần ROS: `optical_flow_estimator.py`,
`pid.py`, `mission_fsm.py`. Lớp `PID` và bảng chuyển trạng thái `TRANSITIONS` đã viết đủ chứ không stub,
vì đây là phần dễ sai âm thầm nhất và cần cố định sớm.

### 1.6 Kiểm chứng đã chạy

| Việc kiểm | Kết quả |
|---|---|
| `colcon build` (8 package) | **sạch**, không cảnh báo |
| `ros2 pkg list \| grep drone` | đủ 8 package |
| `ros2 interface list` | đủ 10 msg + 4 srv; `ros2 interface show MissionPlan` lồng đúng `MissionWaypoint` |
| `import` từng module node (13 node) | **13/13 OK** |
| Khởi động từng node 6 giây (13 node) | **13/13 chạy ổn định**, không node nào crash hay thoát sớm |
| `ros2 launch ... --show-args` (4 launch) | **4/4 parse được** |

**Chưa kiểm chứng được** (không có phần cứng): mọi thứ liên quan tới ảnh thật, pose marker thật, ACK
từ FC, servo gripper. Các con số trong YAML là **giá trị khởi điểm theo tài liệu**, chưa phải giá trị đo.

### 1.7 Ba chỗ đi lệch kế hoạch (có chủ ý)

1. **`qos.py` bị lặp lại ở cả 6 package Python** thay vì đặt vào một package dùng chung. Lý do: package
   dùng chung sẽ là package thứ 9 ngoài kiến trúc 6 lớp đã chốt, và file này chỉ 6 dòng. Đổi lại,
   **sửa quy ước QoS phải sửa 6 chỗ** — nếu về sau QoS phức tạp hơn thì nên gom lại.
2. **Launch dùng `Node` thường, không dùng composable container.** Tài liệu cảnh báo chuỗi ảnh là điểm
   nghẽn CPU Pi 4; `camera_ros`, `image_proc`, `apriltag_ros` đều có sẵn bản composable, gom chung một
   container sẽ bỏ được vài lần copy ảnh. Chưa làm vì chưa đo được tải thật (không có camera) — để lại
   như hướng tối ưu khi `ros2 topic hz` cho thấy thiếu fps.
3. **Gộp 5 lệnh không tham số vào một `FcSimpleCommand.srv`** thay vì 5 file `.srv` gần giống nhau
   (tài liệu liệt kê 8 service riêng). Tập lệnh vẫn đủ 8, chỉ khác cách đóng gói.

### 1.8 Việc còn nợ, theo thứ tự nên làm

1. **Cài gói thiếu**: `ros-jazzy-diagnostic-aggregator`, `python3-pigpio` (+ bật `pigpiod`), `pymavlink`.
2. **Hiệu chỉnh camera** rồi điền `camera_info_url` trong `config/camera.yaml` — bước này **chặn** toàn
   bộ nhánh marker; bỏ qua thì mọi phép đo khoảng cách sai lệch hệ thống.
3. **Đo và sửa `static_transform_publisher` `base_link → camera_link`** trong `estimation.launch.py`
   theo vị trí lắp camera thật (hiện đang là giá trị giả định).
4. **Điền `size` thật của tag** trong `config/apriltag.yaml` và toạ độ bãi đáp trong `known_tags`.
5. **Điền thuật toán 13 node stub** theo đúng thứ tự 8 bước ở mục 8 tài liệu hướng dẫn, mỗi node viết
   `pytest` cho phần logic thuần trước khi chạy trên ROS.
6. **Xác minh firmware FC** với từng plugin MAVROS — rủi ro tương thích lớn nhất của cả hệ thống,
   nên làm sớm chứ không để tới lúc ráp `fc_command_bridge_node`.
7. **PID gain trong `config/control.yaml` đang là 0.0** — cố ý, để không ai vô tình chạy với gain bừa.
   Tune trong Gazebo trước.
8. **Định nghĩa MAVLink dialect tuỳ biến** cho `gcs_link_node` — chưa tài liệu nào mô tả khung gói này.

### 1.9 Điểm cần người quyết, chưa tự chốt được

- `drone_interfaces` được đặt tên trường theo suy đoán từ các đoạn trích trong hai tài liệu hiện có.
  **Cần đối chiếu với `thiet_ke_cau_truc_du_lieu_he_thong_drone.md`** (không có trong `docs/`) trước khi
  ba tầng FC/Pi4/GCS bắt đầu trao đổi thật — lệch tên trường ở giai đoạn sau sẽ tốn công sửa cả ba phía.
- Workspace **chưa phải git repo**. Từ giờ code đã nhiều, nên `git init` để theo dõi thay đổi.

---

## Phiên 2 — 08/09/2026 (chiều)

### 2.1 Người dùng báo đã cắm camera — chẩn đoán

Camera **chưa hoạt động**. Nguyên nhân gốc, đọc từ kernel log của **lần boot hiện tại** (`journalctl -k -b 0`):

```
ov9282 10-0060: fail to write MIPI_CTRL00
ov9282 10-0060: failed to power-on the sensor
ov9282: probe of 10-0060 failed with error -5      # -5 = EIO
```

Chuỗi bằng chứng:

| Kiểm tra | Kết quả | Nghĩa |
|---|---|---|
| `/dev/video0` | không tồn tại | unicam không tạo được video node |
| `cam -l` | `Available cameras:` **rỗng** | libcamera không thấy camera nào |
| `media-ctl -p -d /dev/media0` | `Device topology` **rỗng** | không entity nào, sensor chưa vào graph |
| `/sys/bus/i2c/drivers/ov9282/` | chỉ có `bind`/`unbind`, **không device nào bound** | driver nạp rồi nhưng probe hỏng |
| `/sys/bus/i2c/devices/10-0060/` | có, kèm `waiting_for_supplier` | overlay đã tạo node, sensor không đáp ứng |
| `config.txt` mtime vs boot time | sửa 07/09 01:30, boot 08/09 21:32 | **overlay đã active, không phải quên reboot** |

`dtoverlay=ov9281` + `camera_auto_detect=0` trong `config.txt` là **đúng** cho module này.
Driver `ov9282` (mainline, dùng chung OV9281/OV9282) đã nạp. Vấn đề nằm ở tầng điện: lệnh I²C
đầu tiên để bật nguồn sensor không được ACK.

**Kết luận**: CSI **không hot-plug được** — kernel chỉ probe sensor lúc boot. Camera cắm sau khi
máy đã chạy thì phải **reboot** (hoặc bind lại driver bằng quyền root) mới nhận. Nếu reboot xong
vẫn `error -5` thì là cáp chưa cắm hết/cắm ngược hoặc module hỏng.
Không tự chạy được `sudo` trong phiên này (yêu cầu mật khẩu) nên chưa thử bind lại.

### 2.2 Phát hiện quan trọng hơn — quyết định chọn camera ở Phiên 1 là SAI

Người dùng chỉ tới `/home/pc/PiDrone/` — đây chính là repo chứa `huong_dan_cai_dat_ros2_pi4.md`
mà hai tài liệu thiết kế tham chiếu nhưng Phiên 1 không tìm thấy. Trong đó `docs/CAMERA.md` (35 KB,
đã kiểm chứng trên đúng máy này) lật lại quyết định số 1 của kế hoạch:

**`camera_ros`/libcamera KHÔNG dùng được trên máy này.** `docs/CAMERA.md` mục 9 ghi nhận hai lỗi
**không sửa được từ phía ứng dụng** trên Ubuntu 24.04 + libcamera 0.2.0:
`ipa_base.cpp assertion "it != buffers_.end()" failed in prepareIsp()`, và bản `ros-jazzy-libcamera`
0.7.1 chạy IPA isolated thì crash `-110`. Đường **đã kiểm chứng chạy được** là đọc thẳng V4L2.

Phiên 1 chọn `camera_ros` chỉ vì thấy nó có sẵn và đã build trong `src/` — một suy luận từ hiện
trạng workspace, không có tài liệu chống lưng. **Bài học: `src/` có sẵn thứ gì không có nghĩa thứ
đó chạy được.**

Bốn thông số khác cũng sai theo, vì Phiên 1 mặc định camera màu phổ thông:

| | Phiên 1 giả định | Thực tế (OV9281) |
|---|---|---|
| Loại sensor | màu | **mono, global shutter** |
| Định dạng | `RGB888` | `GREY` → `mono8` |
| Độ phân giải | 640×480 | gốc 1280×800; chọn **640×400** (chế độ binned) |
| FPS khả dụng | 15 | ~246 ở 640×400, ~95 ở 1280×800 |

Sensor mono hoá ra **có lợi**: không Bayer nên không cần ISP/debayer/IPA, AprilTag vốn không cần
màu, và khuyến nghị "chạy rectify ở mono8 cho nhẹ CPU" trong tài liệu gốc giờ thành mặc định.

Một ràng buộc vận hành mới, không có trong hai tài liệu thiết kế: **phải chạy
`camera_v4l2_setup.sh` lại sau MỖI LẦN REBOOT** (ép subdev về `Y8_1X8` và đặt exposure/gain tay —
không có IPA thì không có auto-exposure). Bỏ qua thì topic vẫn ra đúng nhịp nhưng **mọi khung hình
toàn số 0**, và `ros2 topic hz` **không phát hiện được** lỗi này — phải kiểm thống kê pixel.

### 2.3 Đã sửa theo phát hiện trên

- `config/camera.yaml` — viết lại cho `v4l2_camera`: `/dev/video0`, `GREY`, `mono8`, 640×400.
- `launch/perception.launch.py` — thay `camera_ros` bằng `v4l2_camera`, thêm remapping
  `image_raw`/`camera_info`, và ghi rõ trong comment yêu cầu chạy `camera_v4l2_setup.sh` trước.
- `launch/perception.launch.py` docstring — bổ sung bước **kiểm thống kê pixel**, vì `topic hz`
  không bắt được lỗi ảnh toàn 0.
- `drone_bringup/package.xml` — `exec_depend` đổi `camera_ros` → `v4l2_camera`.
- `config/apriltag.yaml` — `size` 0.15 → **0.13**, khớp bản in có sẵn trong `/home/pc/PiDrone/tags/`
  (`apriltag36h11_id0_130mm_a4.svg`, `pad_apriltag_130_30.svg`). Vẫn phải đo thước sau khi in.

`src/camera_ros` giữ nguyên, không xoá (không phải thứ tôi tạo ra), nhưng hệ thống không còn dùng.

### 2.4 Kiểm chứng phiên này

| Việc kiểm | Kết quả |
|---|---|
| `colcon build` sau khi sửa | sạch |
| 4 launch file parse lại | 4/4 OK |
| Chạy thật `perception.launch.py` | **5/5 node khởi động**; chỉ camera lỗi đúng như dự đoán: `Failed opening device /dev/video0: No such file or directory` |

Chuỗi cảm nhận đã nối dây đúng và sẵn sàng; **nút chặn duy nhất là sensor chưa probe được**.

### 2.5 Việc cần người dùng làm (không tự làm được)

1. **Reboot máy** (cần quyền root). Sau đó `ls /dev/video0` — có thì camera đã nhận.
2. Nếu reboot xong vẫn không có `/dev/video0`: kiểm tra cáp FPC — mặt tiếp xúc màu xanh quay đúng
   phía, đẩy hết cỡ rồi khoá chốt (đúng dòng cuối bảng xử lý sự cố của `docs/CAMERA.md`).
3. Sau mỗi lần reboot, trước khi launch:
   `/home/pc/PiDrone/scripts/camera_v4l2_setup.sh --vblank 110 --exposure 800 --gain 120`

---

## Phiên 3 — 08/09/2026 (tối)

### 3.1 Camera đã chạy — nút chặn của Phiên 2 được gỡ

Người dùng reboot xong và yêu cầu test camera. `/dev/video0` đã tồn tại, `ov9281 10-0060` probe
được trên `/dev/media0`. Chạy `camera_v4l2_setup.sh` rồi kiểm theo đúng ba bước đã ghi trong
docstring của `perception.launch.py`:

| Việc kiểm | Kết quả |
|---|---|
| Bắt 10 khung thẳng qua V4L2 | 2.560.000 byte = đúng 640×400×10, 8-bit |
| **Thống kê pixel** (bước `topic hz` không bắt được) | `mean=189.8 std=84.0`, khung khác nhau → **không phải ảnh toàn 0** |
| Node `v4l2_camera` | `mono8 640x400`, `frame_id=camera_optical_frame` |
| `ros2 topic hz /camera/image_raw` | 245.7 Hz, jitter 0,17 ms |

Ảnh chụp ra nhìn rõ vật thể — camera thật sự hoạt động.

### 3.2 Lỗi 1: `camera_info` publish intrinsics RỖNG

`config/camera.yaml` để `camera_info_url: ""` → node rơi về URL mặc định
`~/.ros/camera_info/unicam.yaml`, mà file đó hiệu chỉnh cho **1280×800** trong khi node stream
**640×400**. Lệch độ phân giải nên `camera_info_manager` **bỏ im lặng**, publish
`k = [0,0,...]`, `d = []`, `distortion_model = ''`. Node **chỉ log INFO**, không cảnh báo gì.

Hậu quả nếu bỏ qua: `rectify` vô nghĩa, `apriltag` không giải được PnP, và
`optical_flow_node` đọc focal length = 0 (node này log rõ *"Nhận focal length từ camera_info"*
— tức là nó dùng trực tiếp intrinsics).

Bản hiệu chỉnh đúng **đã có sẵn từ 07/09** — `~/.ros/camera_info/unicam_640x400.yaml`
(fx=323,62 fy=320,33) — chỉ là chưa được trỏ tới. Đã sửa `camera_info_url` trỏ thẳng file này.
Sau khi sửa, `optical_flow_node` báo đúng *"focal length: 323.6 px"*.

**Bài học: `camera_info_url` phải khớp ĐÚNG độ phân giải đang stream; sai thì hỏng im lặng.**

### 3.3 Lỗi 2: 246 FPS làm vỡ đồng bộ AprilTag

Chạy thật `perception.launch.py` thì `apriltag_node` cảnh báo liên tục:
`Topics '/camera/image_rect' and '/camera/camera_info' do not appear to be synchronized`
— chỉ **15 cặp đồng bộ** trên ~70 ảnh mỗi 10 s.

Đo nguyên nhân: **không phải lỗi QoS, cũng không phải rectify đổi stamp** (đã kiểm: rectify giữ
nguyên stamp). Nguyên nhân là Pi 4 bão hoà — `load average 9.38`, CPU idle **1,3 %**, riêng
`rectify_node` 117 %, `optical_flow` 94 %, `apriltag` 65 %, `v4l2_camera` 47 %. Hai topic bị
rớt khung **độc lập nhau**, nên bộ ghép ExactTime hiếm khi khớp được cặp.

246 FPS là lựa chọn có chủ ý của Phiên 2 ("640×400 cho ~246 FPS") nhưng **đo thực cho thấy nó
phản tác dụng**: sensor chạy được không có nghĩa phần còn lại của hệ thống theo kịp.

Hạ xuống 60 FPS (`--vblank 1779`):

| | 246 FPS (`--vblank 110`) | 60 FPS (`--vblank 1779`) |
|---|---|---|
| Khung khớp stamp | 192/484 (40 %) | **442/443 (99,8 %)** |
| Cặp đồng bộ apriltag | 15 / 10 s | đủ nhịp, **0 cảnh báo** |
| `/apriltag/detections` | — | **58 Hz** |
| load average | 9,38 (idle 1,3 %) | **3,54** |
| CPU 4 node cảm nhận | ~324 % / 400 % | ~193 % / 400 % |

Còn dư ~50 % CPU cho nhánh estimation/control/mavlink chưa chạy — 246 FPS thì không còn gì.

FPS **không đặt trong YAML** mà do `vertical_blanking` của sensor:
`FPS = pixel_rate / ((400+vblank) × (640+890))`, `pixel_rate = 200e6`.
Hạ FPS còn nới trần exposure (`≈ height+vblank−25` dòng) — có lợi khi thiếu sáng.

### 3.4 Đã sửa

- `config/camera.yaml` — `camera_info_url` trỏ `unicam_640x400.yaml`; ghi rõ hậu quả nếu để rỗng
  và nhắc đổi file này khi đổi `image_size`.
- `launch/perception.launch.py` — lệnh setup đổi `--vblank 110` → `--vblank 1779`, kèm số liệu đo
  giải thích vì sao không dùng 246 FPS; docstring thêm **bước kiểm `camera_info` có intrinsics**.
- `docs/ke_hoach_thiet_ke_node.md` — cập nhật cùng nội dung ở mục `camera_node`.
- **Đính chính mục 2.5 ở trên**: lệnh sau mỗi lần reboot thiếu `--width/--height` (script mặc định
  1280×800 trong khi `camera.yaml` xin 640×400) và `--vblank` đã đổi. Lệnh đúng là mục 3.6.

### 3.5 Kiểm chứng phiên này

| Việc kiểm | Kết quả |
|---|---|
| `colcon build` sau khi sửa | sạch |
| `perception.launch.py` chạy thật | **5/5 node**, **0 WARN, 0 ERROR** |
| `camera_info` | 640×400, `plumb_bob`, fx=323,62, d đủ 5 hệ số |
| `/camera/image_raw` | 59,3 Hz, `mono8` |
| `/camera/image_rect` | có ảnh, khớp stamp 442/443 |
| `/apriltag/detections` | 58 Hz |

`optical_flow/velocity` và `marker/tracking_quality` **chưa publish — đúng thiết kế**:
`optical_flow_node` chờ `/mavros/global_position/rel_alt` để quy đổi pixel→mét, mà
`perception.launch.py` không chạy MAVROS. Không phải lỗi camera.

### 3.6 Lệnh đúng sau MỖI LẦN REBOOT

```
/home/pc/PiDrone/scripts/camera_v4l2_setup.sh --width 640 --height 400 \
    --vblank 1779 --exposure 800 --gain 120
```

`--exposure/--gain` phụ thuộc ánh sáng phòng, **không có auto-exposure**. Phòng tối thì tăng
(trần ở vblank=1779 là 2154). Kiểm bằng thống kê pixel, không kiểm bằng `topic hz`.

### 3.7 Việc còn nợ của nhánh camera

1. **Đo lại `size` thật của tag bằng thước sau khi in** — `apriltag.yaml` đang để 0,13 theo tên
   file SVG, chưa ai đo. Sai bao nhiêu % thì khoảng cách sai bấy nhiêu %.
2. **Đo `base_link → camera_link`** trong `estimation.launch.py` (đang là giá trị giả định).
3. Kiểm độ chính xác pose AprilTag bằng thước — chưa làm được vì chưa có tag in trước ống kính.
4. `unicam_640x400.yaml` nằm ngoài repo (`~/.ros/`), không được version. Cân nhắc chép vào
   `drone_bringup/config/` và dùng `package://` nếu muốn cấu hình tự chứa.

### 3.8 Cầu nối quan sát từ xa (Foxglove) — khảo sát và sửa kèm

Người dùng SSH từ máy 192.168.10.124 vào Pi 192.168.10.127, **cùng LAN qua WiFi**
(sóng −44 dBm, rất tốt). Muốn xem toàn hệ thống kể cả video.

**Lỗi 3: topic của `image_transport` rơi sai namespace.** `remappings` chỉ đổi topic **gốc**;
các topic của plugin (`compressed`/`theora`/`zstd`) vẫn giữ tên trước remap, nên chui ra
`/image_raw/compressed` thay vì `/camera/image_raw/compressed`. Lệch namespace thì Foxglove
(và các công cụ khác) **không tự ghép được ảnh với `camera_info`**.

Đã sửa: cho `camera_node` và `image_rectify` chạy trong `namespace='camera'` thay vì remap
đường dẫn tuyệt đối. Kết quả — đủ 10 topic đúng chỗ:
`/camera/image_raw{,/compressed,/theora,/zstd}` và `/camera/image_rect{,/compressed,...}`.
Chuỗi giữ nguyên hoạt động sau khi đổi: 0 WARN/ERROR, apriltag 58 Hz, `camera_info` hợp lệ.
Tên node đổi thành `/camera/camera_node`, `/camera/image_rectify`.

**Băng thông đo thực (640×400 mono8 @ ~58 Hz):**

| Luồng | Băng thông | Mỗi khung |
|---|---|---|
| `image_raw` (thô) | **118 Mbps** | 250 KB |
| `image_raw/compressed` (JPEG mặc định) | **22,2 Mbps** | 47 KB |
| `image_raw/compressed` (JPEG quality 50) | **5,4 Mbps** | 14 KB |

→ **Không bao giờ xem topic thô qua WiFi.** Giá CPU của việc nén JPEG: node camera từ 4 %
lên ~31 % một nhân (chỉ tốn khi CÓ người subscribe — không ai xem thì bằng 0).

Tham số chỉnh chất lượng có **dấu chấm đầu**: `.image_raw.compressed.jpeg_quality`
(gọi không có dấu chấm sẽ báo "was not declared").

**Chưa cài được `ros-jazzy-foxglove-bridge`** vì `sudo` cần mật khẩu — người dùng phải tự chạy.
Gói có sẵn trong apt (3.4.1). Cầu nối do đó **chưa được kiểm chứng chạy thật**.

---

## Phiên 4 — 10/09/2026

### 4.1 Rà lại nhánh camera trên máy thật (không chỉ đọc tài liệu)

Người dùng yêu cầu phân tích lại lệnh cài đặt và lệnh chạy của camera. Chạy lại từng lệnh
sau reboot 08:44 và đối chiếu với tài liệu — **5 chỗ lệch**:

| # | Chỗ lệch | Sự thật đo được |
|---|---|---|
| 1 | `CAMERA.md` §4.6 ghi cứng `/dev/media0` | Số hiệu media **không ổn định**: hôm nay `unicam` là **`/dev/media3`** (media0 = rpivid). Chỉ dùng script tự dò, đừng copy lệnh trong tài liệu |
| 2 | `run_camera_node.sh` | Mặc định 1280×800, publish `/image_raw` (không namespace), **không set `camera_info_url`** → lệch hoàn toàn với `perception.launch.py`. Chỉ là công cụ test camera đơn lẻ, KHÔNG dùng cho pipeline |
| 3 | `--exposure 800 --gain 120` (giá trị đang ghi trong tài liệu) | **Cháy sáng**: `mean=232 max=255`. Hạ xuống `--exposure 400 --gain 60` cho `mean=130 std=84` |
| 4 | Bảng FPS §4.6c coi `pixel_rate`/`vblank min` là hằng số | **Phụ thuộc mode**: lúc boot (1280×720) `pixel_rate=160 MHz`, vblank min=41; ở 640×400 vblank min=**22** (tài liệu ghi 110). Công thức vẫn đúng — tin dòng "FPS trần lý thuyết" script tự in |
| 5 | Tài liệu nói không dùng `camera_ros` | Nhưng `ros-jazzy-camera-ros` 0.6.0 **đã cài qua apt** và `src/camera_ros` (0.5.2) **đã build vào workspace**. Rác — nên đặt `COLCON_IGNORE` và gỡ gói apt |

Ngoài ra `ros-jazzy-foxglove-bridge` 3.4.1 **nay đã được cài** (nợ ở mục 3.8), vẫn chưa chạy thử.

### 4.2 Kiểm chứng chạy thật phiên này

| Việc kiểm | Kết quả |
|---|---|
| `camera_v4l2_setup.sh --width 640 --height 400 --vblank 1779` | FPS trần **60.0** (frame 16.7 ms) |
| Thống kê pixel (exposure 400, gain 60) | `min=6 max=255 mean=130.2 std=83.8`, 14 % pixel bão hoà |
| `v4l2_camera_node` + `camera.yaml` | **57.57 Hz**, jitter 0,0013 s |
| `camera_info` | 640×400, `plumb_bob`, fx=323,62, d đủ 5 hệ số |
| `perception.launch.py` trong môi trường trống (`env -i`) | **5/5 node, 0 WARN, 0 ERROR**, `optical_flow_node` nhận focal 323,6 px |

### 4.3 Khảo sát tự động khởi chạy sau boot

- **Không dùng được `After=dev-video0.device`**: `udevadm info /dev/video0` cho
  `TAGS=:seat:uaccess:` — **không có tag `systemd`** nên `dev-video0.device` luôn `inactive`.
  Bắt buộc phải có **vòng chờ** sensor trong script, vì udev tạo `/dev/media*` sau khi
  systemd khởi động service.
- Cấu hình camera **không cần `sudo`**: `pc` đã ở nhóm `video`.
- `sudo` trên máy vẫn **đòi mật khẩu** → chưa cài được unit systemd, người dùng phải tự chạy.

### 4.4 Đã viết: `PiDrone/scripts/drone_startup.sh`

Script khởi động một lệnh, chạy được **cả hai kiểu** (điểm mấu chốt: `source` bên trong một
script chạy trực tiếp KHÔNG lưu được môi trường ra shell gọi nó):

| Cách chạy | Dùng khi | Hiệu lực môi trường ROS |
|---|---|---|
| `source drone_startup.sh` | ngồi gõ lệnh tay | Ở lại shell hiện tại |
| `./drone_startup.sh` | systemd lúc boot | Chỉ bên trong script → lệnh phải nằm ở MỤC 3 |

Bố cục: MỤC 1 cấu hình camera (kèm vòng chờ 30 s), MỤC 2 nạp ROS + workspace,
**MỤC 3 để trống có chủ ý** cho các lệnh thêm sau (`ros2 launch ...`).
Tham số camera gom lên đầu file, mặc định 640×400 / vblank 1779 / exposure 400 / gain 60.

**Lỗi bắt được khi test:** `set -u` làm chết `/opt/ros/jazzy/setup.bash`
(`AMENT_TRACE_SETUP_FILES: unbound variable`) — file setup của ROS không an toàn với `-u`.
Đã bỏ `-u`, giữ `pipefail`, và không bật `set` khi được source (tránh làm phiền shell tương tác).

Bốn test đã chạy: (1) chạy trực tiếp trong `env -i` → exit 0; (2) `source` → `ros2` và
`drone_bringup` có trong shell gọi, `set -u` không rò rỉ; (3) lỗi khi được source → `return`,
**shell không chết**; (4) lệnh đặt ở MỤC 3 chạy đúng với môi trường ROS.

### 4.5 Việc còn nợ

1. **Unit systemd chưa viết vào repo** — khi triển khai chỉ cần một unit `Type=oneshot`
   (hoặc `simple` nếu MỤC 3 có `ros2 launch`) gọi thẳng `drone_startup.sh`, `User=pc`,
   `SupplementaryGroups=video`. Cần `sudo` nên người dùng tự cài.
2. Gỡ `ros-jazzy-camera-ros` và đặt `COLCON_IGNORE` cho `src/camera_ros` (mục 4.1 #5).
3. Sửa 4 chỗ lệch còn lại trong `docs/CAMERA.md` (mục 4.1 #1–#4) — chưa sửa vì tài liệu đó
   nằm ở repo `PiDrone`, chờ người dùng xác nhận.
4. Ba nợ cũ của nhánh camera ở mục 3.7 vẫn nguyên (đo `size` tag, đo `base_link→camera_link`,
   kiểm độ chính xác pose).

### 4.6 Đã viết unit systemd (bổ sung cho nợ 4.5 #1)

`PiDrone/scripts/systemd/drone-startup.service` — `Type=oneshot` + `RemainAfterExit=yes`,
`User=pc`, `SupplementaryGroups=video`, `ExecStart` gọi thẳng `drone_startup.sh`.
Trong file có ghi sẵn hướng dẫn đổi sang `Type=simple` khi MỤC 3 có `ros2 launch`.

Kiểm chứng không cần `sudo`:

| Việc kiểm | Kết quả |
|---|---|
| `systemd-analyze verify` | Sạch, không cảnh báo |
| Chạy script **dưới systemd thật** (`systemd-run --user --wait`) | **exit 0**, runtime 2,4 s, camera cấu hình xong, ROS nạp xong |

**Chưa cài vào `/etc/systemd/system/`** — bước đó cần `sudo` (đòi mật khẩu), người dùng tự chạy.

---

## Phiên 5 — 10/09/2026

### 5.1 Thêm `foxglove_bridge` vào `full_system.launch.py`

`ros-jazzy-foxglove-bridge` 3.4.1 đã có sẵn trên máy. Thêm node với `port: 8765`,
`address: 0.0.0.0`, các tham số khác giữ mặc định của package (`topic_whitelist` mặc
định là `['.*']` nên thấy toàn bộ topic). Thêm `exec_depend` vào `package.xml`.

Kiểm chứng: `ros2 launch ... --print` resolve được `ExecInPkg(pkg='foxglove_bridge')`.

### 5.2 Trả lời: foxglove có tự chạy lúc boot không → **KHÔNG**

`drone-startup.service` gọi `drone_startup.sh`, mà **MỤC 3 đang để trống**, nên lúc boot
không node ROS nào chạy. Log boot 09:29:41 xác nhận: `[3/3] Xong. Chưa có lệnh nào ở MỤC 3.`
Muốn tự chạy cần **hai** thay đổi, không phải một: điền MỤC 3 **và** đổi unit sang
`Type=simple` (vì `ros2 launch` không thoát, để `oneshot` thì systemd treo chờ).

### 5.3 Vẽ sơ đồ hệ thống — `docs/so_do_he_thong.drawio`

Ba trang: (1) trình tự khởi động boot, (2) cây launch 4 tầng include và node theo giai
đoạn, (3) luồng dữ liệu 19 node / 40 mũi tên, trích thẳng từ `create_subscription` /
`create_publisher` / `create_client` chứ không suy đoán.

**Phát hiện đáng lưu ý:** `position_controller_node` (dòng 57) và `fc_command_bridge_node`
(dòng 42) **cùng publish `PositionTarget` lên `/mavros/setpoint_raw/local`**, cả hai đều
phát theo timer riêng. ROS 2 không phân xử — FC nhận xen kẽ hai luồng. Đã đánh dấu đỏ
trên sơ đồ, **chưa sửa** vì chưa rõ `mission_manager` có chủ ý chỉ cho một nhánh chạy tại
mỗi thời điểm hay không. → **nợ mới**.

### 5.4 Gộp `PiDrone` vào workspace + đưa workspace vào git

Lý do gộp: README của `PiDrone` tự khai nó là repo bring-up camera, "làm nền để sau này
ghép vào pipeline ROS 2" — gộp là hoàn tất đúng ý định ban đầu.

Hướng gộp là `ros2_ws` **hút** `PiDrone` chứ không ngược lại, vì `/home/pc/ros2_ws` đã bị
nung cứng vào 12 file trong `install/`; di chuyển workspace là phải build lại toàn bộ.

`git init` tại `ros2_ws` (nhánh `main`), `.gitignore` loại `build/ install/ log/` — ~50 MB
sinh ra so với 1.2 MB mã nguồn, và không dùng lại được trên máy khác do đường dẫn tuyệt
đối. `src/camera_ros/` cũng gitignore: là clone upstream có `.git` riêng, và **đang không
được dùng** (dự án chọn `v4l2_camera`).

Bố cục mới: `scripts/` (chạy trên drone, ngoài môi trường ROS) · `tools/` (chạy tay lúc
phát triển) · `systemd/` · `assets/{tags,examples}` · `docs/`.

**Vì sao `scripts/` ở gốc workspace chứ không nhét vào package `drone_bringup`:**
`camera_v4l2_setup.sh` phải chạy ở MỤC 1, tức **trước** khi source môi trường ROS ở MỤC 2.
Muốn định vị nó qua `ros2 pkg prefix` thì phải source ROS trước — vòng luẩn quẩn. Thêm nữa,
nhét vào package sẽ khiến mỗi script tồn tại hai bản (`src/` và `install/`), sửa bản này
chạy bản kia.

File được **copy**, không `mv` — `~/PiDrone` giữ nguyên 33 file làm kho lưu trữ đông lạnh.

Đường dẫn đã sửa: `scripts/drone_startup.sh` (`CAM_SETUP` + 2 comment) ·
`systemd/drone-startup.service` (`ExecStart`, `Documentation`, lệnh cài) ·
`src/drone_bringup/{config/apriltag.yaml, launch/perception.launch.py}` ·
`docs/ke_hoach_thiet_ke_node.md` · `docs/{CAMERA,RUNBOOK,cai_dat_ros2_framework_pi4}.md` ·
`docs/so_do_he_thong.drawio`.

**File này (`nhat_ky_lam_viec.md`) cố ý KHÔNG sửa** — 7 chỗ nhắc `PiDrone` ở các phiên
trước là ghi chép lịch sử, sửa đi sẽ làm sai sự thật đã xảy ra. Đường dẫn hiện hành nằm ở
`README.md` và `docs/RUNBOOK.md`.

Kiểm chứng:

| Việc kiểm | Kết quả |
|---|---|
| `bash -n scripts/drone_startup.sh` | Cú pháp sạch |
| Chạy thật `./scripts/drone_startup.sh` từ vị trí mới | **exit 0**, camera Y8_1X8 640×400 @60 FPS, ROS nạp xong |
| `systemd-analyze verify systemd/drone-startup.service` | Sạch |
| `ExecStart` trỏ tới file có thật và executable | OK |
| `colcon build --packages-select drone_bringup` | Finished |
| Còn sót `PiDrone` ngoài nhật ký | Không |

### 5.5 Việc còn nợ

1. **Cài lại unit vào `/etc/systemd/system/`** — bản đang chạy vẫn trỏ `ExecStart` sang
   `/home/pc/PiDrone/scripts/drone_startup.sh` (file cũ còn nguyên nên boot chưa gãy).
   Cần `sudo`, người dùng tự chạy 3 lệnh trong `README.md`.
2. **Hai node cùng publish `/mavros/setpoint_raw/local`** (mục 5.3) — cần đọc máy trạng
   thái `mission_manager_node` để xác định là thiết kế hay lỗi.
3. Nợ 4.5 #2 (`COLCON_IGNORE` cho `src/camera_ros`) — nay đã gitignore, nhưng colcon **vẫn
   đang build** package này. Vẫn nên đặt `COLCON_IGNORE` để khỏi tốn thời gian build.
4. Ba nợ cũ của nhánh camera ở mục 3.7 vẫn nguyên.

---

## Phiên 6 — 14/09/2026

Đặc tả FC ↔ Pi: `docs/GIAO_UOC_FC_ROS2.md` (hợp đồng 1.4). Chi tiết đo đạc nằm trong commit và giao ước;
mục này chỉ tóm tắt và ghi việc còn nợ.

### 6.1 Đã làm

| Việc | Commit | Kết quả chính |
|---|---|---|
| Thử ARM/DISARM trên FC 1.4 (cánh tháo) | `3bebec5` | Nằm bàn nhận, kê 0,88 m từ chối, `21196` cắt; Pi tự chặn khi mất quyền |
| Máy trạng thái lớp FC (P2/P10) | `8078acc` | `~/start` → ARM → TAKEOFF → `~/land` → tự DISARM → IDLE, thử trên bàn đạt |
| Lớp ước lượng: vận tốc camera + FC, marker pose, cờ healthy EKF | `9462cce` | EKF hội tụ về marker; chốt P3 (bỏ mẫu `1e6`) |
| Hiệu chỉnh lại camera 640×400, tag 0,122 m, TF camera nghiêng 20° | `9462cce` | Tag 82 cm → 0,822 m; 32,5 cm → 0,327 m; pháp tuyến tag lệch ≤ 2° |
| Chịu tải cao: TF tag mới nhất, EKF `smooth_lagged_data` | `09b1bcf` | Pose marker 0,2 → 11,5 Hz; EKF hết phân kỳ |
| Giảm tải Pi 4: `EventsExecutor`, camera 30 FPS, bớt plugin MAVROS, optical flow dùng laser | `52846e3` | Idle 0,8 % → 26 %; apriltag 3,9 → 26 Hz; MAVROS 72 → 52 % |
| Đề xuất FC giảm tải MAVROS | `42f7e5f` | Giao ước 11.1 #14 — chờ FC |

### 6.2 Việc còn nợ

1. **Ưu tiên thời gian thực cho tuyến setpoint** (`mavros_node`, `position_controller_node`) — **chưa làm,
   cần sudo.** Hiện user `pc` có `ulimit -r` = 0 nên `chrt` phải chạy bằng root. Làm một lần:
   1. `echo 'pc - rtprio 60' | sudo tee /etc/security/limits.d/99-drone-rtprio.conf`, đăng nhập lại,
      kiểm `ulimit -r` = 60.
   2. Nếu node được khởi động bằng `drone-startup.service`: thêm `LimitRTPRIO=60` vào `[Service]`,
      `sudo systemctl daemon-reload` (systemd không đọc `limits.d`).
   3. Thêm `prefix='chrt -f 50'` vào `Node(...)` của `mavros_node` và `position_controller_node` trong file
      launch; chạy tay thì `ros2 run --prefix 'chrt -f 50' ...`. Kiểm bằng `chrt -p <pid>` → `SCHED_FIFO` 50.
   - **Chỉ** hai tiến trình này (52 % và 7 % một nhân). Không đặt cho apriltag / rectify / optical flow.
   - Tạm thời chạy tay (mất khi node khởi động lại):
     `sudo chrt -f -p 50 $(pgrep -f lib/mavros/mavros_node)` và
     `sudo chrt -f -p 50 $(pgrep -f lib/drone_control/position_controller_node)`.
2. **Chờ FC trả lời giao ước 11.1 #14** (giảm tần số `ATTITUDE`/`HIGHRES_IMU`, ngừng `LOCAL_POSITION_NED`,
   `VFR_HUD`, `GLOBAL_POSITION_INT` 1 Hz). FC nạp xong thì Pi đo lại tần số trên dây, CPU MAVROS, EKF.
3. **Dấu vận tốc optical flow chưa kiểm** — đẩy drone tới trước bằng tay trên nền có vân, phải thấy
   `/optical_flow/velocity` `vx > 0` (đang đứng yên nên chỉ thấy 0).
4. **Hiệu chỉnh camera thiếu mẫu ở góc ảnh** (trên-phải gần như trống, hai góc dưới ít) — chụp bổ sung
   nếu tag gần rìa ảnh mà vị trí lệch. Bản sao lưu: `src/drone_bringup/config/camera_info/`.
5. **`pad_a` (tag ID 1) chưa đo cạnh** — `apriltag.yaml` đang giả định 0,122 m như `pad_home`.
6. **`estimation.launch.py` tự bật MAVROS** — chạy launch này khi MAVROS đã chạy sẽ có hai tiến trình
   giữ `/dev/ttyAMA0`. Cần tách hoặc thêm cờ tắt.
7. **Trục z của EKF chưa có mốc tuyệt đối khi không thấy marker** — cân nhắc đưa `z` của `ODOMETRY` FC vào.
8. **Còn TODO trong code:** trạng thái nhiệm vụ (waypoint, marker, gripper, `on_plan`), `landing_target_bridge_node`
   (P1), watchdog P6. `fc_command_bridge_node` vẫn dùng `MultiThreadedExecutor` (service chờ chặn ACK).

---

## Phiên 7 — 15/09/2026

### 7.1 Đã làm

| Việc | Kết quả chính |
|---|---|
| **Giao nhiệm vụ không GPS** — `config/tags.yaml` dùng chung; vị trí waypoint suy từ `expected_marker_id`; `on_plan` kiểm và nạp kế hoạch (chỉ IDLE, **không tự cất cánh**); công cụ `ros2 run drone_mission send_mission_plan <yaml>`, mẫu `config/missions/ban_home.yaml` | Kế hoạch đúng được nhận; tag lạ bị từ chối kèm lý do |
| **Hạ cánh theo marker (P1)** — `landing_target_bridge_node`, nguồn `SOURCE_LANDING` trong `position_controller_node`, trạng thái PRECISION_LAND + service `~/precision_land` | Pose tag 29 Hz; dấu căn tâm đúng (kp tạm 0,5, đã trả về 0) |
| **Failsafe** — `failsafe_monitor_node` + `failsafe_rules.py`; FSM xử lý mức leo thang (RTH tạm = hạ tại chỗ, LOITER giữ rồi hạ khi hết sự cố, IDLE huỷ yêu cầu cất cánh) | Tắt/bật EKF → sự cố bật/hết đúng; giả lập pin thấp → không tự ARM |
| **Thử cả chuỗi trên bàn** (cánh tháo): kế hoạch → `~/start` → `~/precision_land` → tự DISARM | 1 ARM + 1 DISARM `ACCEPTED`, về IDLE, không cần dự phòng |

Pytest: `drone_mission` 64, `drone_control` 8, `drone_safety` 10.

### 7.2 Vấn đề phát hiện, chưa xử lý

1. **EKF phân kỳ không tự hồi phục.** Qua đêm 14→15/09 EKF trôi tới phương sai 1,2·10¹⁰ m²; phải khởi động
   lại `robot_localization`. Nguyên nhân: khi trạng thái đã lệch xa, `pose0_rejection_threshold: 2.0` loại cả
   đo marker ĐÚNG → không bao giờ kéo về được. `failsafe_monitor_node` bắt đúng (LOITER) nên không nguy hiểm
   trên bàn, nhưng khi bay là mất vị trí vĩnh viễn. Hướng sửa: khi `/ekf/health` không healthy mà vẫn thấy tag
   → gọi service `set_pose` của `robot_localization` bằng pose từ marker; hoặc nới / bỏ ngưỡng loại trừ
   pose0 và đo lại.
2. **Nhánh PRECISION_LAND ở độ cao > 0,35 m chưa kiểm trên phần cứng** (chờ căn tâm, xuống theo tag, mất tag
   → MARKER_SEARCH). Trên bàn laser 0,15 m nên luôn đi nhánh "sát đất". Cần kê drone cao hơn 0,35 m, tag
   trong tầm camera. Đã có pytest.
3. **Gain `landing.*` và `cruise.*` = 0.0** — căn tâm theo tag và bay vị trí chưa có tác dụng; phải tune
   trong Gazebo trước (không tune lần đầu trên drone thật).
4. **Trạng thái nhiệm vụ còn TODO:** ENROUTE, MARKER_SEARCH, RETRY_LOITER, ACTUATE_GRIPPER, RTH. MARKER_SEARCH
   hiện chỉ đứng yên nếu PRECISION_LAND mất tag. ESCALATE_RETRY_LOITER (tìm marker quá lâu, gripper không
   xác nhận) FSM **chưa xử lý** — để các trạng thái đó làm.
5. **Nhánh tạm TAKEOFF → PRECISION_LAND** (service `~/precision_land`) chỉ để thử trên bàn — thay bằng
   ENROUTE → MARKER_SEARCH → PRECISION_LAND khi viết bước 4.
6. **RTH tạm thời = hạ cánh tại chỗ** (pin < 25 %, mất GCS) cho tới khi bay vị trí được.
7. **Chưa có ACK kế hoạch về GCS** — `gcs_link_node` vẫn là khung (chưa định nghĩa dialect MAVLink tuỳ biến);
   kết quả nhận/từ chối hiện chỉ ở `/mission/state.detail` và log.
8. **`mission_logger_node`, `telemetry_aggregator_node` vẫn là khung.**
9. **`failsafe_monitor_node` không còn gọi `fc_command_bridge_node/simple_command`** như thiết kế cũ: leo
   thang = đổi trạng thái FSM (giao ước 5.1, 5.3). `simple_command`, `takeoff`, `goto_waypoint` của bridge
   vẫn trả "chưa hiện thực".
10. Nợ Phiên 6 mục 6.2 vẫn còn nguyên (ưu tiên thời gian thực, dấu optical flow, góc ảnh hiệu chỉnh, `pad_a`,
    `estimation.launch.py` tự bật MAVROS, trục z EKF).

---

## Phiên 8 — 15/09/2026

Làm mục 1–4 trong danh sách ưu tiên sau Phiên 7.

### 8.1 Đã làm

| Việc | Kết quả chính |
|---|---|
| **EKF tự hồi phục (7.2 #1)** — `ekf_health_node` gửi pose marker lên `/set_pose` khi có pose marker mới mà EKF lệch marker > 1,0 m liên tục 1,0 s, hoặc EKF không healthy (`MarkerResetPolicy`, cách nhau ≥ 2 s) | Tái hiện trên bàn: đẩy EKF lệch 20 m → **trước:** loại marker đúng 23 s, không healthy 14 s; **sau:** về marker sau ~1 s, không lần nào báo không healthy, không ép thừa |
| **Watchdog P6** — `position_controller_node` ngừng phát setpoint khi `/mission/state` im quá `mission_timeout_s` = 1,0 s | `SIGSTOP` `mission_manager_node` → 0 setpoint/s sau ~0,8 s; `SIGCONT` → 20 Hz lại |
| **Chuyển nguồn setpoint mượt** — `SetpointRamp` trộn tuyến tính 0,8 s khi đổi nguồn (vận tốc mission / cruise / landing); chỉ PID nguồn đang dùng được cập nhật; vào nguồn PID thì reset kèm sai số hiện tại (không giật D) | Trên bàn: `vz` 0 → 0,5 trong 0,8 s và 0,5 → 0 trong 0,8 s sau khi lệnh quá hạn |
| **Trạng thái nhiệm vụ** — TAKEOFF → ENROUTE (xét khoảng cách ngang) → MARKER_SEARCH (chờ `landing_target_bridge_node` xác thực đúng ID) → PRECISION_LAND; hết giờ tìm / failsafe RETRY_LOITER / mất tag khi hạ = một lần thất bại → RETRY_LOITER giữ 3 s rồi tìm lại; quá `max_retries` → EMERGENCY_LAND tại chỗ | Pytest. **Thử trên bàn có ARM** (cánh tháo, laser 0,21 m, tag 0 dưới mặt bàn ~20 cm, `takeoff_alt_m:=0.1`): ARM `ACCEPTED` → TAKEOFF → ENROUTE (cách 0,16 m) → MARKER_SEARCH → PRECISION_LAND → nhánh sát đất, `vz` ramp tới −0,28 → chạm đất 1,4 s → DISARM `ACCEPTED` → IDLE, tổng 3,6 s, 1 ARM + 1 DISARM. `mission_logger_node` ghi đủ chuỗi vào `~/drone_logs/1/` |
| **`mission_logger_node`** — `mission_log.py`: `<log_root>/<mission_id>/events.jsonl` (MISSION_OPEN, STATE_CHANGE, FAILSAFE, MARKER_MATCH/LOST, GRIP_CONFIRMED/RELEASED) + ảnh xác nhận; chỉ subscribe camera khi ACTUATE_GRIPPER | Chạy với topic giả (remap) + camera thật: đủ sự kiện đúng thứ tự, ảnh JPEG thật có tag |

Pytest: `drone_estimation` 12, `drone_control` 14, `drone_mission` 72, `drone_safety` 16.

### 8.2 Thay đổi hành vi cần biết

1. **Bỏ service `~/precision_land`** và nhánh tạm TAKEOFF → PRECISION_LAND (7.2 #5). Thử chuỗi trên bàn giờ đi
   đường thật: kế hoạch → `~/start` → TAKEOFF → ENROUTE → MARKER_SEARCH → PRECISION_LAND. Laser trên bàn
   0,15–0,21 m nên phải chạy `mission_manager_node` với `-p takeoff_alt_m:=0.1`, drone đặt trên tag.
2. **`retry_count` tính theo từng điểm** — trước đây mọi lần chuyển trạng thái đều xoá về 0 nên giới hạn thử lại
   không bao giờ có tác dụng.
3. **`position_controller_node` bỏ `/mission/setpoint` cũ hơn 0,5 s** (mission phát 5 Hz khi cần bay tới điểm),
   không bám mãi điểm đến cũ.
4. **Chạy `control.launch.py` riêng (không có `mission_manager_node`) sẽ không phát setpoint** — do watchdog P6.
5. Mốc "bắt/mất tag" trong log lấy từ `/landing_target/lost`, không từ `/marker/tracking_quality`
   (`marker_quality_node` vẫn là khung). `mission_manager_node` bỏ subscribe `/marker/tracking_quality`.

### 8.3 Việc còn nợ

1. **Chưa thử trên phần cứng:** nhánh hết giờ tìm → RETRY_LOITER (che tag), nhánh PRECISION_LAND > 0,35 m, và
   vòng ENROUTE phát `/mission/setpoint` (trên bàn ENROUTE / MARKER_SEARCH chỉ kéo dài một chu kỳ).
2. Vận tốc ngang sau khi lệnh mission hết hạn giờ về 0 trong 0,5 s + 0,8 s ramp (trước: 0,5 s) — xem lại khi
   tune trong Gazebo.
3. `max_vel_mps` của waypoint được kiểm nhưng **chưa áp** vào `position_controller_node`.
4. MARKER_SEARCH chỉ đứng giữ tại điểm, chưa có quỹ đạo tìm (xoắn ốc…).
5. Ép `/set_pose` giữa chuyến bay làm vị trí EKF nhảy → sai số cruise nhảy theo; cân nhắc khi tune.
6. Nợ Phiên 7 còn lại: ACTUATE_GRIPPER, RTH, gain = 0, `gcs_link_node`, `telemetry_aggregator_node`, nợ 6.2.

### 8.4 Mô phỏng Gazebo để tune `cruise.*` (cùng phiên)

| Việc | Kết quả chính |
|---|---|
| `position_controller_node` đổi gain khi đang chạy (`ros2 param set`, giữ tích phân, từ chối giá trị âm/nan) | Trên bàn: đặt `cruise.x.kp` áp ngay; `-1.0` bị từ chối kèm lý do |
| Package **`drone_sim`** (chạy trên PC): `sim_fc.py` (FC giả theo giao ước), `sim_fc_bridge_node`, `step_test`, world X3 + `MulticopterVelocityControl`, `sim_tune.launch.py`, `sim_mission.launch.py`, hướng dẫn `src/drone_sim/README.md` | Tên topic, khung lệnh vận tốc (khung thân) đọc từ mã nguồn gz-sim8. Pi không có Gazebo nên kiểm bằng xe giả bậc nhất (τ 0,3 s) trên `ROS_DOMAIN_ID=77`: tune (`step_test` z/x, có trễ 0,1 s + nhiễu 3 cm) và cả nhiệm vụ `sim_tag1` ARM → ENROUTE 10 m → tìm/thử lại → hạ tại chỗ → DISARM → IDLE đạt. Hai launch chạy được với `ros_gz_*` giả |

Chưa kiểm: Gazebo thật trên PC (tải model X3 từ Fuel, tên topic, dấu lệnh — README mục 2 có bước kiểm dấu).
Pytest `drone_control` 15, `drone_sim` 11.

