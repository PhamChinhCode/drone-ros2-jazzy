# Việc cần làm — tích hợp MAVLink FC ↔ Pi 4

> **Đặc tả duy nhất: [GIAO_UOC_FC_ROS2.md](GIAO_UOC_FC_ROS2.md) (từ 2026-09-13).** File này
> chỉ còn là sổ theo dõi việc. Các link tới `thiet_ke_mavlink_fc.md`, `tra_loi_*`,
> `phan_hoi_*` bên dưới là tài liệu **đã cũ** — khi mâu thuẫn, giao ước đúng.

Danh sách theo dõi, đi kèm [thiet_ke_mavlink_fc.md](thiet_ke_mavlink_fc.md).
Cập nhật lần cuối: **2026-09-13** (sau [tra_loi_dot_2_pi4.md](tra_loi_dot_2_pi4.md);
phản hồi của Pi: [phan_hoi_dot_2_fc.md](phan_hoi_dot_2_fc.md)).

Phân nhóm theo **điều kiện thực hiện**, vì drone đang ở giai đoạn bàn test —
chưa cấp điện tầng công suất động cơ.

---

## 0. Đã nghiệm thu ngày 2026-09-12 (bàn test)

Firmware FC đã nạp bản mới. Đo thật qua `/dev/ttyAMA0` @921600.

| Hạng mục | Kết quả |
|---|---|
| `HEARTBEAT` | 1.0 Hz |
| `ATTITUDE` | 50.0 Hz |
| `HIGHRES_IMU` | 50.0 Hz |
| `LOCAL_POSITION_NED` | 30.3 Hz |
| `VFR_HUD` | 10.0 Hz |
| `GLOBAL_POSITION_INT` | 10.0 Hz |
| `SYS_STATUS` | 2.1 Hz |
| `BATTERY_STATUS` | 1.0 Hz |
| `EXTENDED_SYS_STATE` | 1.0 Hz |
| `AUTOPILOT_VERSION` | đã trả lời lệnh 520, hết cảnh báo `VER` |
| Băng thông | **7,59 KB/s = 8,4%** (tài liệu ước 7,9 KB/s — chính xác) |

Nội dung khớp đặc tả mục 3.2:

- `GLOBAL_POSITION_INT`: `lat=0`, `lon=0`, `hdg=65535` đúng quy ước "không biết";
  `rel_alt=222 mm`, `alt=36139 mm`.
- `HIGHRES_IMU`: `fields_updated=0x1BFF` — acc/gyro/mag/baro/nhiệt hợp lệ, bit
  áp suất vi sai = 0 (đúng, không có cảm biến tốc độ gió). `acc.z = -9.90` đúng
  quy ước FRD khi đặt phẳng.
- `EXTENDED_SYS_STATE`: `landed_state=1` (`ON_GROUND`) — đúng.
- `LOCAL_POSITION_NED`: `z = -0.223 m` khớp `rel_alt = 222 mm`. Nhất quán.
- `SYS_STATUS`: `sensors_health = 0x14F` — gyro/accel/mag/baro/**optical
  flow**/laser đều khoẻ. RC và pin chưa khoẻ (chưa bật tay điều khiển, chưa gắn pin).

Topic ROS đo qua MAVROS — tất cả đều phát:

```
/mavros/imu/data                    49.99 Hz
/mavros/imu/data_raw                49.99 Hz
/mavros/imu/mag                     48.05 Hz
/mavros/local_position/pose         30.30 Hz
/mavros/local_position/velocity_local 30.31 Hz
/mavros/global_position/rel_alt      9.997 Hz
/mavros/global_position/global       9.995 Hz
/mavros/sys_status                   2.00 Hz
/mavros/battery                      1.00 Hz
/mavros/extended_state               1.00 Hz
/mavros/state                        connected: true, armed: false, mode CMODE(2)
```

### Đo lại ngày 2026-09-13 (pymavlink, 20 s + 30 s + 30 s)

Tần số 9 bản tin **không đổi**, 7,8 KB/s (8,5%), 0 khung hỏng. Khác so với 12/9:

- `sensors_health = 0x1010F`: bit flow `0x40` **tắt** (bản vá B11 đã chạy), bit RC
  `0x10000` bật. 1/61 mẫu `0x1010B` (bit từ kế tắt thoáng qua).
- `custom_mode = 1` (ANGLE) — do vị trí công tắc.
- `fields_updated` vẫn ~99% `0x1BFF` → **B10 chưa sửa**.
- `LOCAL_POSITION_NED.vx/vy` và `GLOBAL_POSITION_INT.vx/vy` khác 0 ở **100%** mẫu
  khi bit flow tắt — FC xác nhận đó là hợp đồng (xem B12).
- `AUTOPILOT_VERSION`: `capabilities = 0x2000`, mọi trường phiên bản = 0; lệnh 512
  trả `UNSUPPORTED` — FC hứa sửa (B16).

---

## A. Làm được ngay trên bàn — FC chỉ cần cấp điện

### A1 — Sửa tài liệu đặc tả *(phía Pi, không cần phần cứng)* — **XONG**

- [x] **A1.1** *(xong 2026-09-12)* Đã thêm `'vfr_hud'` vào `plugin_allowlist` thay vì
      bỏ dòng khỏi tài liệu — FC vốn đã phát `VFR_HUD` 10 Hz, phơi ra là có ích.
      Xác minh: `/mavros/vfr_hud` phát 10.004 Hz.
- [x] **A1.2** *(xong 2026-09-12)* Sửa 7 link firmware hỏng ở đầu tài liệu và phụ lục
      (`../Mavlink/mav_link.c`, `../State/fc_state.h`, `../Control/ctrl_poshold.h`,
      `../Estimator/estimator.c`, `../Control/arming.c`, `../Mavlink/mav_link.h`,
      `../Mavlink/mav_port.c`). Firmware là repo riêng, không nằm trong `ros2_ws`.
      Ghi đường dẫn repo, hoặc trích nguyên văn đoạn quan trọng vào phụ lục.
- [x] **A1.3** *(xong 2026-09-12)* Sửa nhãn bit 9 trong bảng `type_mask` mục 4.3. `FORCE` không phải
      bit "bỏ qua" — nó nghĩa là "af mang lực thay vì gia tốc".
- [x] **A1.4** *(xong 2026-09-12)* Mục 8.6: ghi rõ tên trường phía ROS là **`sensors_health`**, không
      phải `onboard_control_sensors_health` như tên bên MAVLink.
- [x] **A1.5** *(xong 2026-09-12)* Thêm vào mục 8: xung đột hai publisher (xem A3.1).
- [x] **A1.6** *(xong 2026-09-12)* Tách mục 9 thành **9.A bàn test** và **9.B cần điện động cơ**.

### A2 — Sửa cấu hình MAVROS *(phía Pi)* — **XONG**

- [x] **A2.1** *(xong 2026-09-12)* Đã bỏ `'battery'` và thêm `'vfr_hud'`. Bỏ `'battery'` khỏi `plugin_allowlist` trong
      [mavros.yaml](../src/drone_bringup/config/mavros.yaml) — **không phải tên
      plugin hợp lệ**. Allowlist có 9 mục nhưng chỉ 8 plugin được nạp.
      `/mavros/battery` do `sys_status` sinh ra.

### A3 — Sửa mã nguồn *(phía Pi)*

- [x] **A3.1** *(XONG 2026-09-12)* Xung đột hai publisher. Chốt **`position_controller_node`
      độc quyền** publish `/mavros/setpoint_raw/local` (đúng mục 2: Pi gửi vận tốc).
      Đã gỡ publisher setpoint + timer republish + `active_setpoint` khỏi
      `fc_command_bridge_node`; node này chỉ còn cổng lệnh arm/takeoff/mode.
      `on_goto_waypoint` giờ chuyển target cho position_controller qua `/mission/setpoint`.
      **Còn lại (phase C):** `position_controller_node:55` vẫn subscribe
      `/mavros/landing_target/raw` (topic không tồn tại, nay mồ côi sau A3.2) — phải
      chốt kiến trúc đóng vòng hạ cánh (Pi vòng vận tốc vs FC `LANDING_TARGET`) khi làm C6.
- [x] **A3.2** *(XONG 2026-09-12)* Sai topic hạ cánh chính xác:
      [landing_target_bridge_node.py](../src/drone_control/drone_control/landing_target_bridge_node.py)
      đã đổi publish từ `mavros_msgs/LandingTarget` @ `/mavros/landing_target/raw`
      (topic **không tồn tại** — xác minh trực tiếp với mavros_node) sang
      `geometry_msgs/PoseStamped` @ `/mavros/landing_target/pose`.
- [x] **A3.3** *(XONG 2026-09-12)* Bẫy pin. `failsafe_monitor_node.on_battery` nay coi
      `percentage < 0` là "không biết" (`battery_pct = None`) thay vì `-1.0`, chặn
      `periodic_check` kích `ESCALATE_EMERGENCY_LAND` oan trước khi thân hàm được viết.

### A5 — Luồng đọc dữ liệu FC → Pi *(rà soát 2026-09-12)*

- [x] **A5.1 — Covariance IMU** *(XONG 2026-09-12)* — đã đặt `orientation_stdev: 0.01745`
      (1°, phía FC chốt), `linear_acceleration_stdev: 0.0185`, `angular_velocity_stdev: 0.00183`
      trong `mavros.yaml`. Xác minh trên `/mavros/imu/data`: cov = 3.045e-4 / 3.42e-4 / 3.35e-6.
      **Cảnh báo gốc:** `ekf.yaml` lấy
      orientation + angular_velocity + linear_acceleration từ `/mavros/imu/data`,
      nhưng **không tham số covariance nào được đặt** trong `mavros.yaml` — MAVROS
      đang dùng mặc định hardcode. Đo nhiễu thật 896 mẫu/20 s lúc drone đứng im:

      | Tham số (node `/mavros/imu`) | MAVROS mặc định | Đo được khi đứng im | Sai lệch |
      |---|---|---|---|
      | `linear_acceleration_stdev` | 0.0003 | **0.0185** m/s² | lạc quan 62× |
      | `angular_velocity_stdev` | 0.000349 | **0.00183** rad/s | lạc quan 5.2× |
      | `orientation_stdev` | 1.0 (57,3°) | nhiễu 0.00056 rad (0,032°) | bi quan ~1800× |

      Hệ quả: EKF **gần như bỏ qua** góc nghiêng từ FC (thứ đáng tin nhất) và
      **tin gần như tuyệt đối** vào gia tốc thô (thứ nhiễu nhất). Đúng ngược.

      **Lưu ý khi chọn số:** 0,032° là *nhiễu ngắn hạn*, không phải sai số tuyệt
      đối của góc — đặt `orientation_stdev` bằng con số đó sẽ khiến EKF tin thái
      quá. Giá trị hợp lý tính cả trôi bias là 0,5–2° (0,01–0,035 rad), riêng yaw
      còn phụ thuộc hiệu chuẩn từ kế (A4.6). Tương tự, 0,0185 m/s² là sàn khi
      đứng im; có rung động cơ sẽ cao hơn nhiều, phải đo lại ở giai đoạn C.

- [x] **A5.2 — `LOCAL_POSITION_NED.x/y` luôn bằng 0.** *(ĐÃ GIẢI THÍCH 2026-09-12,
      [tra_loi_cau_hoi_pi4.md](tra_loi_cau_hoi_pi4.md))* — FC chỉ tích phân vị trí khi
      flow còn khoá; trên bàn không khoá nên đứng ở 0. Đúng thiết kế.
      *Mô tả gốc:* Đo 6 mẫu liên tiếp: `x` và
      `y` đúng bằng `0.0` trong khi `vx = -0.133 m/s`. FC **không tích phân vị
      trí**, trái với mô tả ở mục 3.2 của tài liệu ("tích phân vận tốc optical
      flow"). Cần xác nhận với phía FC: chưa hiện thực, hay chỉ tích phân khi đã
      arm? Sửa lại mục 3.2 cho khớp thực tế.

- [x] **A5.3 — `NavSatFix` chở độ cao rác** *(XONG 2026-09-12)*. `telemetry_aggregator_node.on_global_pos`
      nay bỏ qua (`global_pos = None`) khi `status.status < 0` (`NO_FIX`), để GCS không
      thấy toạ độ 0,0 ở độ cao ~37 m khi thân `publish_packet` được viết.

- [ ] **A5.4 — Cân nhắc bật thêm plugin đọc dữ liệu.** Các plugin sau **có sẵn**
      nhưng đang bị `ignored` vì không có trong `plugin_allowlist`. Chỉ bật khi FC
      thật sự phát bản tin tương ứng:

      | Plugin | Bản tin cần | Dùng để làm gì |
      |---|---|---|
      | `distance_sensor` | `DISTANCE_SENSOR` (132) | Laser MTF01P thô + cờ ngoài tầm — quan trọng cho hạ cánh chính xác |
      | `odometry` | `ODOMETRY` (331) | Thay `LOCAL_POSITION_NED`, **có mang covariance** |
      | `px4flow` / `optical_flow` | `OPTICAL_FLOW_RAD` (106) | Chất lượng flow của FC, đối chiếu với flow từ camera Pi |
      | `rc_io` | `RC_CHANNELS` (65) | Trạng thái tay điều khiển |
      | `vibration` | `VIBRATION` (241) | Chẩn đoán rung — cần cho giai đoạn C |
      | `altitude` | `ALTITUDE` (141) | Nhiều nguồn độ cao tách bạch |

      *(2026-09-13)* Đã bật **`debug_value`** để nhận `NAMED_VALUE_INT`
      `OB_STATE`/`OB_AUTH`/`OB_EXIT` (B13). Xác minh: plugin `initialized`, có topic
      `/mavros/debug_value/named_value_int`. FC chưa phát nên topic còn rỗng.

- [x] **A5.5 — Đồng bộ thời gian: đã kiểm tra, KHÔNG phải vấn đề.** Plugin
      `sys_time` đang bị bỏ nên không có `/mavros/time_reference` và không trao
      đổi `TIMESYNC`. Nhưng đo thực tế `/mavros/imu/data`: nhịp **50.000 Hz**, lệch
      chuẩn **0,43 ms**, độ trễ header so với giờ Pi **2 ms** (min 1, max 3). Mọi
      mốc thời gian đều dùng giờ Pi nhận gói nên **tự nhất quán**. Không cần bật
      `sys_time` ở giai đoạn này.

### A4 — Kiểm chứng thêm trên bàn *(cần FC cấp điện, drone đứng im hoặc cầm tay)*

- [ ] **A4.1** **Dấu trục optical flow.** Cầm drone dịch ngang trên sàn có vân,
      xem `LOCAL_POSITION_NED.vx/vy` đổi đúng chiều không. Đây là cách rẻ nhất
      để bắt lỗi hệ trục (B1) trước khi bay.
- [ ] **A4.2** **Trôi optical flow khi đứng yên.** Đo được `vx = -0.133 m/s`,
      `vy = -0.031 m/s` trong khi drone nằm im. **13 cm/s là đáng kể** — cần xác
      minh đây là nhiễu nền hay lỗi hiệu chuẩn.
- [ ] **A4.3** **Bit sức khoẻ flow.** Che cảm biến flow, xem bit
      `MAV_SYS_STATUS_SENSOR_OPTICAL_FLOW` (0x40) trong `sensors_health` có hạ
      không. Xác minh A1.4 / mục 8.6 chạy được thật.
- [ ] **A4.4** **Chiều `relative_alt`.** Nhấc drone bằng tay, xem
      `/mavros/global_position/rel_alt` tăng đúng chiều.
- [ ] **A4.5** **Bảng chế độ bay.** Gạt công tắc chế độ trên tay điều khiển, đối
      chiếu `CMODE(n)` với bảng mục 5. **Đổi chế độ không cần arm.**
- [ ] **A4.6** **Từ kế.** `fields_updated` đang bật bit từ kế (6–8), nghĩa là
      firmware khai QMC6309 vừa khoẻ vừa **đã hiệu chuẩn**. Kiểm tra lại xem đã
      hiệu chuẩn thật chưa — mục 4.3 dự định chuyển sang `FRAME_LOCAL_NED` khi
      yaw từ từ kế đáng tin.

---

## B. Đã có trả lời từ phía FC — [tra_loi_cau_hoi_pi4.md](tra_loi_cau_hoi_pi4.md), 2026-09-12

- [x] **B1 — Hệ trục: FRD, xác nhận.** `vy+` = sang phải, `vz+` = đi xuống. Firmware
      không diễn giải FLU ở bất kỳ đâu. MAVROS đã chuyển FLU→FRD giúp nên hai bên khớp.
      **Vẫn phải chạy quy trình kiểm dấu** (B5) để đóng bằng phép đo.
- [x] **B2 — Yaw: câu hỏi hết hiệu lực.** FC **không đọc `yaw`** ở bất kỳ dạng nào —
      firmware không có vòng góc yaw. Chỉ dùng `yaw_rate` (rad/s, dương = quay phải).
      **→ Sinh ra B6 bên dưới: `type_mask` phải đổi.**
- [x] **B3 — OFFBOARD: chỉ bằng công tắc RC.** `SET_MODE` chưa hiện thực, đừng gửi.
      Sáu điều kiện phải đúng đồng thời; quan trọng nhất: **Pi phải đang phát setpoint
      TRƯỚC khi người lái gạt công tắc**. OFFBOARD rơi ra thì vào trạng thái `KHOA`,
      **không tự phục hồi** — phải gạt công tắc xuống rồi lên lại.
- [ ] **B4 — Pin.** Hoãn theo yêu cầu, làm sau.

### Việc mới phát sinh từ đợt trả lời

- [ ] **B5 — Chạy quy trình kiểm dấu.** Phía FC cung cấp cách đóng B1/B2 **bằng phép
      đo, không cần arm, không cần cánh quạt**: Pi phát setpoint lặp, gõ `offboard` trên
      console USART1, đối chiếu `vtoi_mms` / `vphai_mms` / `vlen_mms` / `yaw_mdps`.
      Kèm ba bộ đếm `nhan` / `loai` / `kep` để tách ba kiểu hỏng. **Việc đáng làm nhất
      hiện nay.**
- [x] **B6 — `type_mask` đổi `0x0BC7` → `0x07C7`** *(XONG 2026-09-12)*. Đã sửa
      [thiet_ke_mavlink_fc.md](thiet_ke_mavlink_fc.md) mục 4.3 (bảng trường, khối
      `type_mask`, bit 10/11) và mục 8.2: `yaw` tuyệt đối → `yaw_rate`, `0x0BC7` →
      `0x07C7`. Cũng sửa chú thích lạc hậu QMC6309 → QMC5883P trong mục 4.3. Hai tài
      liệu trong repo nay khớp nhau.
- [ ] **B7 — `mission_manager_node` theo đúng giao thức OFFBOARD.** Phát setpoint vận
      tốc 0 làm nhịp giữ chỗ **trước**; không gửi `SET_MODE`; xử lý được trạng thái
      `KHOA` không tự phục hồi (node restart **không** làm máy bay chạy lại).
      **Cập nhật 2026-09-13 (đợt 2, mục 0 và 2.1):**
      - Pi **được arm** khi người lái đã gạt ch8 lên + ch5 lên. Thứ tự: phát setpoint
        10–20 Hz → gửi ARM → OFFBOARD tự vào.
      - ACK ARM/DISARM: `ACCEPTED (0)` tiếp tục; `DENIED (2)` = mất quyền, **không thử
        lại**, chờ người lái; `TEMPORARILY_REJECTED (1)` = thử lại sau. Đọc trường
        `result` của `/mavros/cmd/arming`, không chỉ `success`.
      - Mất quyền (chạm cần / mất sóng) thì Pi **không ra lệnh gì được, kể cả DISARM**.
        Chỉ lấy lại khi người lái gạt ch8 xuống-lên.
      - Quyết định dựa vào **`OB_AUTH`** / `OB_STATE` (`/mavros/debug_value/named_value_int`),
        **không** suy quyền từ `RC_CHANNELS` hay `custom_mode`.
      - `OB_STATE = 2` mà `custom_mode ≠ 4` = OFFBOARD còn chạy nhưng FC đã tụt về
        ANGLE vì mất flow — setpoint của Pi đang **không** được dùng.
      - **Chặn bởi:** FC chưa phát `NAMED_VALUE_INT` (việc #1 phía FC).
- [x] **B8 — Tầm đo MTF01P.** *(FC trả lời 2026-09-13, đợt 2 mục 2.2)* `max_distance =
      8,0 m` (giới hạn firmware), EKF tin tới 6,0 m, nghiêng tối đa 25°, σ = 0,05 m,
      100 Hz. **`min_distance` chưa có** — FC phải đo cận dưới (theo dõi ở B17).
- [x] **B9 — Chốt thứ tự ưu tiên.** *(chốt 2026-09-13, [phan_hoi_dot_2_fc.md](phan_hoi_dot_2_fc.md))*
      Đồng ý thứ tự FC đề xuất: trạng thái OFFBOARD (`NAMED_VALUE_INT` + `STATUSTEXT`) →
      `AUTOPILOT_VERSION` + lệnh 512 → `DISTANCE_SENSOR` → `ODOMETRY` → `RC_CHANNELS`.
      `OPTICAL_FLOW_RAD` và `VIBRATION` để sau.

### Cần trả lời ngược lại phía FC

- [ ] **B10 — `fields_updated` KHÔNG khớp firmware. Họ yêu cầu báo ngay.**
      *(2026-09-13: đo lại vẫn 1482 `0x1BFF` / 18 `0x1A3F`. Đợt 2 **không nhắc tới** —
      đã hỏi lại trong [phan_hoi_dot_2_fc.md](phan_hoi_dot_2_fc.md).)* Đo trên dây
      15 giây: **`0x1BFF` 743 mẫu, `0x1A3F` 7 mẫu**. Họ kỳ vọng `0x1A3F` (bit từ kế TẮT
      vì chưa hiệu chuẩn). Chênh lệch đúng bằng `0x01C0` = bit 6–8 (từ kế). Tức **99,1%
      khung đang bật cờ từ kế**, cổng `calibrated` không được áp dụng. Giá trị từ kế
      cũng là số thật, không phải 0.
- [x] **B11 — Số hex `sensor_health` họ trích là biến nội bộ, không phải trường MAVLink.**
      *(XONG — đo 2026-09-13: `onboard_control_sensors_health = 0x1010F`, bit flow `0x40`
      nay tắt đúng khi bộ ước lượng mất tin cậy. FC xác nhận ở đợt 2 mục 1.1.)*
      Họ ghi `sensor_health = 0x051F`. Trên dây, `onboard_control_sensors_health` =
      **`0x0000014F`** (present/enabled = `0x0201014F`). Chẩn đoán của họ vẫn **đúng**:
      bit `OPTICAL_FLOW` (`0x40`) **đang bật** trong khi vận tốc vô hiệu. Nhưng bản vá
      một dòng phải nhắm đúng trường MAVLink, không phải biến nội bộ.
- [ ] **B12 — EKF của Pi hiện KHÔNG dùng vận tốc của FC.** Khuyến nghị "bỏ mẫu `vx`/`vy`
      khi bit flow tắt" áp cho đường dữ liệu chưa nối: `ekf.yaml` lấy `twist0:
      /optical_flow/velocity` — do `optical_flow_node` tính từ **camera OV9281 trên Pi**,
      không phải `LOCAL_POSITION_NED`. Nên trôi 13 cm/s hiện **không ảnh hưởng EKF**.
      Khuyến nghị vẫn đúng cho tương lai nếu nối đường đó.
      *(2026-09-13, đợt 2 mục 1.1)* FC **không** đưa vận tốc về 0 khi bit flow tắt — đó
      là hợp đồng. Khi viết bộ lọc, bỏ **chỉ** `vx`/`vy`/`x`/`y` (và `groundspeed`),
      **giữ** `z`/`vz`/`relative_alt`/`alt`/`climb`. Không bỏ cả khung
      `GLOBAL_POSITION_INT`. Hiện `optical_flow_node` đọc độ cao qua topic riêng
      `/mavros/global_position/rel_alt` nên không bị ảnh hưởng.

### Việc mới phát sinh từ đợt 2 — [tra_loi_dot_2_pi4.md](tra_loi_dot_2_pi4.md), 2026-09-13

- [x] **B13 — Bật plugin `debug_value`** *(XONG 2026-09-13)* trong
      [mavros.yaml](../src/drone_bringup/config/mavros.yaml). Đã build lại `drone_bringup`
      và xác minh plugin nạp, có `/mavros/debug_value/named_value_int`.
      `STATUSTEXT` đã có sẵn ở `/mavros/statustext/recv` (plugin `sys_status`).
- [x] **B14 — Launch MAVROS crash khi khởi động.** *(XONG 2026-09-13 — giao ước 11.1 #4.)*
      Nguyên nhân thật là `name='mavros'`: launch áp remap `__node` cho **mọi** node con
      trong process (router, từng plugin) → trùng tên, plugin đè topic nhau. Đã **bỏ
      `name=`**, thêm `namespace='mavros'`, giống `mavros/node.launch`. Xác minh bằng
      `ros2 launch` (chỉ node MAVROS): `connected: true`, 0 lỗi, có
      `/mavros/debug_value/named_value_int`, `/mavros/setpoint_raw/local`.
      *Mô tả gốc:*
      [estimation.launch.py:24](../src/drone_bringup/launch/estimation.launch.py#L24)
      chạy `mavros_node` với `name='mavros'` nhưng **không có** `namespace='mavros'`. Chạy
      như vậy (có hoặc không có `debug_value`) đều crash:
      `create_subscription() called for existing topic name rt/mavros/mavros/local with
      incompatible type`. Chạy với `-r __ns:=/mavros` thì bình thường, đúng như
      `mavros/launch/node.launch` gốc (`namespace` mặc định `mavros`). Cần thêm
      `namespace='mavros'` rồi chạy thử `ros2 launch`.
- [ ] **B15 — Cảnh báo bộ đếm lỗi theo TỐC ĐỘ TĂNG.** `errors_count1..4` là bộ đếm cộng
      dồn `uint16` (tràn vòng ở 65 535): 1 = IMU ICM20602, 2 = CRSF, 3 = baro BMP388,
      4 = MTF01P. Cảnh báo khi tăng quá N lần / 10 s, không theo giá trị. Có sẵn trong
      `mavros_msgs/SysStatus` trên `/mavros/sys_status` (`errors_count1..4`).
- [ ] **B16 — Kiểm lại `AUTOPILOT_VERSION` khi FC nạp bản mới.** Kỳ vọng
      `capabilities = 0x2080`, `flight_sw_version = 0x00010000`, `flight_custom_version`
      = 8 byte git hash, lệnh 512 (`param1 = 148`) được trả lời.
- [ ] **B17 — `DISTANCE_SENSOR`: chờ FC đo `min_distance`.** Khi có bản tin: bật plugin
      `distance_sensor`, kiểm `signal_quality = 1` khi ngoài tầm, `orientation = PITCH_270`.
- [ ] **B18 — Cập nhật mục 6 [thiet_ke_mavlink_fc.md](thiet_ke_mavlink_fc.md).** Đã gắn
      ghi chú "lạc hậu" ở mục 5 và 6 (2026-09-13); viết lại toàn bộ khi FC chốt xong
      giao thức arm từ Pi.

## B~ (cũ) — Cần xác nhận từ phía FC trước khi viết code

- [ ] **B1** **Quy ước hệ trục — chặn, mức độ rơi máy bay.** Đo thật cho thấy
      MAVROS **không** truyền nguyên giá trị:

      FRAME_BODY_NED (8):   (vx, vy, vz) -> (vx, -vy, -vz);  yaw -> -yaw
      FRAME_LOCAL_NED (1):  (vx, vy, vz) -> (vy, vx, -vz);   yaw -> pi/2 - yaw

      Phía Pi publish theo chuẩn ROS (FLU/ENU), MAVROS tự đổi sang FRD/NED.
      Tài liệu **không có một chữ nào** về việc này, trong khi mục 3.2 lại ghi
      "hệ NED" — người viết node sẽ tính theo NED rồi publish thẳng, khiến `vy`
      và `vz` đảo dấu trên dây. Cần xác nhận firmware hiểu `vy` dương là sang
      phải (FRD) rồi bổ sung một mục vào 4.3.

- [ ] **B2** **Yaw tuyệt đối hay tương đối.** Mục 4.3 ghi `yaw` là "góc yaw
      tuyệt đối" nhưng lại chọn `FRAME_BODY_NED`. Trong hệ thân, yaw tuyệt đối
      là mâu thuẫn ngữ nghĩa. Hai frame còn có phép biến đổi yaw khác nhau (xem B1).

- [ ] **B3** **Đường vào OFFBOARD.** Ba chỗ trong tài liệu không khớp: mục 4.2
      nói `SET_MODE` "chưa làm"; mục 4.3 nói OFFBOARD vào bằng công tắc RC; mục 5
      nói mode 4 "dành sẵn". Công tắc RC là **điều kiện đủ**, hay chỉ mở cổng rồi
      Pi vẫn phải gửi `SET_MODE`? Quyết định này định hình `mission_manager_node`.

- [ ] **B4** **Pin chưa kiểm chứng được.** Chưa gắn pin nên `BATTERY_STATUS`
      trả `voltages[] = 65535` toàn bộ và `SYS_STATUS.voltage_battery = 0 mV`.
      Không xác minh được mô tả ở mục 3.2 ("điền điện áp tổng chia số cell vào N
      ô đầu"). Đo lại khi có pin. Lưu ý `voltage_battery = 0` là số hợp lệ nhưng
      sai — cân nhắc dùng `UINT16_MAX` cho trạng thái "không biết".

---

## C. Chờ cấp điện tầng công suất động cơ

Không thực hiện, không đề xuất, cho tới khi có điện động cơ.

- [ ] **C1** Arm bằng công tắc RC → `/mavros/state` đổi `armed: true`.
- [x] **C2a** *(XONG 2026-09-12)* Hợp đồng `COMMAND_ACK` đã kiểm chứng trên bàn theo
      cách phía FC chỉ: `NAV_TAKEOFF (22)` → `UNSUPPORTED`, `DISARM (400,p1=0)` →
      `ACCEPTED`, cả hai dưới 0,01 s; `armed` giữ nguyên `False` trước và sau.
- [ ] **C2b** Hợp đồng `COMMAND_ACK` đường ARM. **Đã đổi theo đợt 2 mục 0** — tiêu chí
      cũ ("ARM luôn `success: false`") không còn đúng. Tiêu chí mới: ch8/ch5 chưa lên →
      `DENIED (2)`; có quyền nhưng ga chưa giữa → `TEMPORARILY_REJECTED (1)`; đủ điều
      kiện → `ACCEPTED (0)`; sau khi người lái chạm cần → cả ARM lẫn DISARM đều `DENIED`.
- [ ] **C3** `EXTENDED_SYS_STATE` chuyển `ON_GROUND` ↔ `IN_AIR` (ngưỡng 0,5 m).
- [ ] **C4** Giai đoạn 2 phía FC: `SET_POSITION_TARGET_LOCAL_NED`, `SET_MODE`,
      `LANDING_TARGET`, `MAV_CMD_NAV_TAKEOFF`.
- [ ] **C5** Tune PID trong [control.yaml](../src/drone_bringup/config/control.yaml)
      — toàn bộ gain hiện là `0.0`. Tune trong Gazebo trước, không bao giờ tune
      lần đầu trên drone thật.
- [ ] **C6** Hạ cánh chính xác theo AprilTag (phụ thuộc A3.2 và C4).
