# Giao ước MAVLink giữa Flight Controller và Raspberry Pi 4

> **⚠ ĐÃ CŨ — KHÔNG DÙNG LÀM ĐẶC TẢ (từ 2026-09-13).** Đã hợp nhất vào [GIAO_UOC_FC_ROS2.md](GIAO_UOC_FC_ROS2.md). Khi mâu thuẫn, giao ước đúng. Chỉ giữ lại để tra *lý do* của quyết định.

Tài liệu này là **hợp đồng hai chiều**, không phải yêu cầu một phía. Mỗi mục đều
ghi rõ trạng thái: đã chạy, hay còn phải làm, và làm ở bên nào.

Bản gốc do phía Pi soạn ngày 2026-09-12, mô tả những gì ROS 2 cần. Bản này đã
đối chiếu với firmware STM32H743 thật và sửa lại những chỗ giả định sai — chủ
yếu ở mục 4 (giao diện điều khiển) và mục 5 (bảng chế độ bay).

Nguồn sự thật phía firmware: `Mavlink/mav_link.c`, `State/fc_state.h`,
`Control/ctrl_poshold.h`, `Estimator/estimator.c`.

> Firmware nằm ở **repo riêng**, không thuộc `ros2_ws`, nên các đường dẫn trên là
> tham chiếu văn bản chứ không mở được từ tài liệu này. Cần trích nguyên văn đoạn
> quan trọng vào phụ lục, hoặc ghi URL repo firmware.

---

## 1. Liên kết vật lý — đã thông, không cần đổi

| Hạng mục | Giá trị |
|---|---|
| Đường truyền | UART, Pi GPIO14 (TX) ↔ FC **PE0/RX**, GPIO15 (RX) ↔ FC **PE1/TX**, **GND chung** |
| Ngoại vi phía FC | **UART8**, DMA2 (TX Normal, RX Circular) |
| Thiết bị trên Pi | `/dev/ttyAMA0` (PL011, bật bằng `dtoverlay=disable-bt`) |
| Baudrate | 921600, 8N1, không flow control |
| Phiên bản giao thức | MAVLink v2 (byte mở đầu `0xFD`) |
| System ID / Component ID của FC | `1` / `1` (`MAV_COMP_ID_AUTOPILOT1`) |

Cấu hình phía Pi nằm ở [mavros.yaml](../src/drone_bringup/config/mavros.yaml).

Lưu ý: `/dev/ttyS0` (mini UART) **không** dùng được ở 921600 vì baud của nó bám
theo xung nhịp VPU và sẽ trôi.

UART8 dùng riêng cho MAVLink. USART3 đang chở khung nhị phân riêng của dự án tới
ESP32, USART1 là console CLI — không trộn.

---

## 2. Nguyên tắc phân chia trách nhiệm

Mục này quyết định mọi thứ còn lại, nên đặt lên trước.

**Vòng nào đóng ở đâu:**

```
  Pi 4 (ROS 2, Linux không thời gian thực)
    └─ nhiệm vụ, AprilTag, VIO ─> VÒNG VỊ TRÍ ─> lệnh VẬN TỐC ─┐
                                                   10-20 Hz     │
                                                                ▼
  FC (STM32H743, vòng lặp cứng)                        SET_POSITION_TARGET
    └─ VÒNG VẬN TỐC ─> góc nghiêng ─> VÒNG GÓC ─> VÒNG TỐC ĐỘ GÓC ─> trộn
       (ctrl_poshold)                  (ctrl_angle)   (ctrl_rate)
```

**Pi gửi xuống VẬN TỐC — không phải góc nghiêng, không phải vị trí.** Ba lý do,
đều rút ra từ code hiện có chứ không phải lý thuyết chung:

1. **FC đã có sẵn đúng vòng đó.** `ctrl_poshold.h`
   tuy mang tên "poshold" nhưng chính header nói rõ *"Đây là giữ VẬN TỐC, chưa
   phải giữ VỊ TRÍ"*. Chuỗi của nó là `vận tốc mong muốn → P+I → góc nghiêng`.
   Cho Pi điều khiển vận tốc chỉ là đổi nguồn setpoint, mọi tầng dưới và mọi
   tham số PID đã chỉnh đều giữ nguyên.

2. **Lệnh vị trí bị chính bộ ước lượng cấm.** `est.position_m.x/y` là tích phân
   vận tốc optical flow — dẫn đường suy tính thuần tuý. `estimator.c`
   ghi: sai số tích luỹ và *"KHÔNG BAO GIỜ tự hết"*. Ra lệnh vị trí tuyệt đối
   lên một ước lượng đang trôi nghĩa là máy bay bay để bù cho phần trôi không
   có thật. Vòng P vị trí trên FC cũng chưa tồn tại.

3. **Lệnh góc sai ở chế độ hỏng.** Gửi góc tức là đẩy vòng vận tốc lên Pi, qua
   ROS 2 trên Linux không thời gian thực. Khi Pi khựng: lệnh góc → máy bay giữ
   nguyên độ nghiêng và **tăng tốc đi mất**; lệnh vận tốc → FC hết hạn setpoint,
   đặt mục tiêu về 0 → **phanh lại rồi treo**. Khác biệt này là toàn bộ vấn đề.

Thêm một điểm không hiển nhiên: `ctrl_poshold_update()` trả `false` khi mất
optical flow, và `ctrl_angle` tự lùi về chế độ ANGLE. **Lưới an toàn đó chỉ tồn
tại ở tầng vận tốc.** Nếu Pi ra lệnh góc, nó hoàn toàn không biết flow đã chết.

Hệ quả: **vòng vị trí vẫn có, nhưng nằm trên Pi** — nơi có AprilTag và VIO làm
nguồn tham chiếu tuyệt đối. Pi quy sai số vị trí ra lệnh vận tốc. Vì vòng ngoài
vốn chậm, Pi chỉ cần 10–20 Hz và miễn nhiễm với jitter đường truyền.

---

## 3. FC → Pi: bảng phát

### 3.1 Đang phát

Toàn bộ bảng **đã đo thật** bằng `pymavlink` trên `/dev/ttyAMA0` ngày 2026-09-12
sau khi firmware nạp bản mới. Cột "Đo được" là tần số quan sát trong 10 giây.

| Message | ID | Tần số | Trạng thái | Topic ROS sinh ra |
|---|---|---|---|---|
| `HEARTBEAT` | 0 | 1 Hz | **1.0** | `/mavros/state` |
| `SYS_STATUS` | 1 | 2 Hz | **2.1** | `/mavros/battery`, `/mavros/sys_status` |
| `ATTITUDE` | 30 | 50 Hz | **50.0** | `/mavros/imu/data` |
| `VFR_HUD` | 74 | 10 Hz | **10.0** | `/mavros/vfr_hud` |
| `GLOBAL_POSITION_INT` | 33 | 10 Hz | **10.0** | `/mavros/global_position/rel_alt`, `/global` |
| `LOCAL_POSITION_NED` | 32 | 30 Hz | **30.3** | `/mavros/local_position/pose`, `/velocity_local` |
| `HIGHRES_IMU` | 105 | 50 Hz | **50.0** | `/mavros/imu/data_raw`, `/mavros/imu/mag` |
| `BATTERY_STATUS` | 147 | 1 Hz | **1.0** | `/mavros/battery` |
| `EXTENDED_SYS_STATE` | 245 | 1 Hz | **1.0** | `/mavros/extended_state` |
| `COMMAND_ACK` | 77 | theo sự kiện | *chưa kiểm chứng* — mục 9.B.1 | kết quả `/mavros/cmd/*` |
| `AUTOPILOT_VERSION` | 148 | khi được hỏi | **đã trả lời** | hết cảnh báo `VER` |

Tổng ước tính **7,9 KB/s**; **đo thật được 7,59 KB/s = 8,4%** băng thông ở 921600
baud. Còn rất rộng, không cần cắt giảm tần số.

### 3.2 Ghi chú từng bản tin

**`GLOBAL_POSITION_INT`.** Bo mạch **không có GPS**. Bản tin vẫn phát vì
[optical_flow_node.py:45](../src/drone_perception/drone_perception/optical_flow_node.py#L45)
dùng `relative_alt` (mm, so với mặt đất lúc khởi động) để quy đổi dịch chuyển
pixel ra mét.

Quy ước "không biết" theo đúng đặc tả: `lat = lon = 0`, `hdg = 65535`. Chỉ ba
nhóm trường sau mang thông tin thật:

| Trường | Nguồn | Ghi chú |
|---|---|---|
| `relative_alt` | `est.altitude_m` × 1000 | EKF hợp nhất baro + laser MTF01P |
| `alt` | `baro.altitude_m` × 1000 | độ cao áp suất, **không phải MSL thật** |
| `vx`/`vy`/`vz` | `est.velocity_mps` × 100 | hệ NED, cm/s |

**`LOCAL_POSITION_NED`.** Độ tin cậy **không đồng đều giữa các trục**:

- `z`, `vz` — từ EKF độ cao, đáng tin.
- `x`, `y` — tích phân vận tốc optical flow. Chỉ dùng được để biết "đã rời chỗ
  cũ bao xa" trong vài chục giây gần đây. **Tuyệt đối không dùng làm gốc toạ độ
  cho bay theo lộ trình hay quay về điểm xuất phát.**
- `vx`, `vy` — từ optical flow, đáng tin **khi còn flow**. Mất flow thì
  `est.position_valid` hạ xuống, nhưng MAVLink không có chỗ chở cờ này. Phía Pi
  theo dõi gián tiếp qua `SYS_STATUS`: bit `MAV_SYS_STATUS_SENSOR_OPTICAL_FLOW`
  (`0x40`) trong `onboard_control_sensors_health`. **Phía ROS trường này tên là
  `sensors_health`** trong `mavros_msgs/SysStatus` trên `/mavros/sys_status`.

**`HIGHRES_IMU` thay cho `SCALED_IMU`.** Bản gốc đề nghị `SCALED_IMU` (26) hoặc
`HIGHRES_IMU` (105). Chọn cái sau vì `SCALED_IMU` nhét gyro vào `int16` đơn vị
mrad/s, mà 2000 dps = 34900 mrad/s — **tràn kiểu ngay trong dải đo bình thường
của ICM20602**. `HIGHRES_IMU` dùng `float` nên không có bẫy đó, và chở luôn cả
từ kế lẫn khí áp trong cùng một khung.

Trường `fields_updated` nói rõ trường nào có thật. Bit từ kế (6–8) **chỉ bật khi
QMC6309 vừa `healthy` vừa `calibrated`** — từ kế chưa hiệu chuẩn lệch hướng hàng
chục độ, báo "có dữ liệu" lúc đó tệ hơn báo "không có". Phía Pi phải đọc
`fields_updated` chứ đừng giả định mọi trường đều hợp lệ.

**`BATTERY_STATUS`.** Dự án đo điện áp tổng và dòng điện, **không đo từng cell**.
`voltages[]` điền điện áp tổng chia số cell vào đúng N ô đầu để MAVROS hiện đúng
số cell; các ô còn lại là `UINT16_MAX`. `battery_remaining = -1` ("không biết")
là cố ý: suy phần trăm từ điện áp lúc đang tải sai lệch lớn, báo không biết
trung thực hơn báo một con số bịa.

**`EXTENDED_SYS_STATE`.** Chưa arm → `ON_GROUND`. Đã arm → dựa vào độ cao với
ngưỡng 0,5 m. Không có `est.altitude_valid` thì trả `UNDEFINED`.

### 3.3 Đã bỏ, kèm lý do

**`GPS_RAW_INT` (24) — không phát.** Bản gốc xếp vào nhóm "nên có" với lý do
*"thiếu thì `NavSatFix.status` luôn báo no fix"*. Nhưng bo mạch không có GPS, nên
bản tin sẽ mang `fix_type = 0`, và MAVROS cũng cho ra đúng "no fix". Phát một
bản tin rỗng vĩnh viễn không đổi được gì, chỉ thêm rác trên đường truyền. Khi nào
lắp GPS thật thì bật lại.

---

## 4. Pi → FC: giao diện điều khiển

### 4.1 Đang xử lý

| Message / lệnh | ID | FC làm gì |
|---|---|---|
| `HEARTBEAT` | 0 | mốc theo dõi đường truyền, hết 3000 ms coi như mất Pi |
| `COMMAND_LONG` / `MAV_CMD_COMPONENT_ARM_DISARM` (400) | 76 | **chỉ chiều DISARM** — xem mục 6 |
| `COMMAND_LONG` / `MAV_CMD_REQUEST_AUTOPILOT_CAPABILITIES` (520) | 76 | trả `AUTOPILOT_VERSION` |

**Mọi `COMMAND_LONG` đều được trả `COMMAND_ACK`**, kể cả lệnh không hỗ trợ
(`MAV_RESULT_UNSUPPORTED`). Đây là hợp đồng bắt buộc: MAVROS chặn service
`/mavros/cmd/*` cho tới khi có ACK, thiếu ACK thì mọi lệnh treo tới timeout và
nhìn từ ROS sẽ giống "lệnh thất bại" dù FC đã làm xong.

`COMMAND_ACK` chứa đúng `command` vừa nhận (không để 0), và `target_system` /
`target_component` lấy thẳng từ `sysid` / `compid` của khung gửi tới — với MAVROS
là 255 / 190.

### 4.2 Chưa xử lý — giai đoạn 2

| Message | ID | Sinh từ | Tình trạng |
|---|---|---|---|
| `SET_POSITION_TARGET_LOCAL_NED` | 84 | `/mavros/setpoint_raw/local` | **chưa làm** |
| `SET_MODE` | 11 | `/mavros/set_mode` | **chưa làm** |
| `LANDING_TARGET` | 149 | `/mavros/landing_target/pose` | **chưa làm** |
| `COMMAND_LONG` / `MAV_CMD_NAV_TAKEOFF` (22) | 76 | `/mavros/cmd/takeoff` | ACK `UNSUPPORTED` |

Hiện gọi `/mavros/set_mode` sẽ **không có tác dụng và cũng không báo lỗi rõ**:
`SET_MODE` không dùng `COMMAND_ACK`, MAVROS xác nhận bằng cách chờ `custom_mode`
trong `HEARTBEAT` đổi — mà FC chưa đổi. Đừng dựa vào nó cho tới khi mục này xong.

### 4.3 `SET_POSITION_TARGET_LOCAL_NED` — đặc tả chốt trước khi viết code

Ghi ở đây để hai bên viết song song mà vẫn khớp.

**FC chỉ đọc vận tốc (`vx`,`vy`,`vz`) và `yaw_rate`. Bỏ qua vị trí, gia tốc, và `yaw` tuyệt đối.**

| Trường | FC dùng | Ánh xạ |
|---|---|---|
| `vx`, `vy` | có | mục tiêu vận tốc ngang → `ctrl_poshold` |
| `vz` | có | mục tiêu tốc độ lên → `ctrl_althold` (`vz = 0` tự chốt độ cao) |
| `yaw_rate` | có | tốc độ quay (rad/s, `+` = quay phải) → `ctrl_rate` |
| `x`,`y`,`z` | **bỏ** | lý do ở mục 2 |
| `afx`,`afy`,`afz` | **bỏ** | không có vòng gia tốc |
| `yaw` | **bỏ** | FC không có vòng góc yaw; ra lệnh `yaw` tuyệt đối → **loại cả khung** |

**`coordinate_frame` = `MAV_FRAME_BODY_NED` (8).** `ctrl_poshold` vốn xoay vận
tốc về hệ thân để tính, và sai số AprilTag cũng sinh ra ở hệ camera — dùng hệ
thân bỏ được một phép xoay ở mỗi đầu. Chuyển sang `MAV_FRAME_LOCAL_NED` (1) sau,
khi yaw từ từ kế QMC5883P đã chứng minh đáng tin.

**`type_mask` — chốt theo câu trả lời FC** (xem
[tra_loi_cau_hoi_pi4.md](tra_loi_cau_hoi_pi4.md) Câu 2). FC **không đọc `yaw`** ở
bất kỳ dạng nào: firmware không có vòng góc yaw, chỉ điều khiển *tốc độ* yaw. Vì
thế **bit 10 (`yaw`) phải BẬT** (bỏ qua) và **bit 11 (`yaw_rate`) phải TẮT**
(dùng) — ngược với bản trước của tài liệu này. Mask đúng là:

```
type_mask = 0b0000011111000111 = 0x07C7
```

| Bit | Trường | Giá trị | Nghĩa |
|---|---|---|---|
| 0–2 | `x`, `y`, `z` | `111` | bỏ qua |
| 3–5 | `vx`, `vy`, `vz` | `000` | **dùng** |
| 6–8 | `afx`, `afy`, `afz` | `111` | bỏ qua |
| 9 | `force` | `1` | cờ "af mang lực thay vì gia tốc" — vô hại vì af đã bị bỏ |
| 10 | `yaw` | `1` | **bỏ qua** — đặt `0` sẽ bị FC loại cả khung |
| 11 | `yaw_rate` | `0` | **dùng** |

Sai bất kỳ bit nào so với `0x07C7` → FC **loại cả khung**, bộ đếm `loai` tăng, và
sau `offboard_timeout_ms` (500 ms) FC hết hạn setpoint rồi lùi về POSHOLD. FC
**bắt buộc** tôn trọng `type_mask` — đọc cả ô Pi cố tình để trống là lỗi kinh
điển, máy bay sẽ lái theo số rác.

**Tần số:** Pi gửi 10–20 Hz là đủ. Vòng vận tốc đóng trên FC ở tốc độ vòng lặp
chính nên không cần Pi chạy nhanh.

**Hết hạn setpoint — phần quan trọng nhất, không được bỏ.** Quá **500 ms** không
nhận được `SET_POSITION_TARGET_LOCAL_NED`, FC đặt mục tiêu vận tốc về 0 (phanh
và treo), rồi lùi về chế độ ANGLE. Đây chính là thứ làm cho lệnh vận tốc an toàn
hơn lệnh góc.

**Công tắc cho phép trên tay điều khiển.** OFFBOARD chỉ vào được khi người lái
gạt một công tắc cho phép. Cùng triết lý với arming (mục 6): người đứng cạnh máy
bay luôn giữ quyền phủ quyết bằng phần cứng, không qua phần mềm.

---

## 5. Bảng `custom_mode` — đã sửa theo firmware

Firmware khai `autopilot = MAV_AUTOPILOT_GENERIC`, nên MAVROS **không biết** tên
chế độ bay và `/mavros/state` trả về `mode: "CMODE(2)"` thay vì một cái tên.

Hệ quả cho phía Pi: gọi `/mavros/set_mode` phải truyền `custom_mode` dạng **số**
và để `base_mode = 0`. Truyền chuỗi `"GUIDED"` hay `"OFFBOARD"` không có tác dụng.

> **Bản gốc đọc nhầm chỗ này.** Nó thấy `custom_mode = 2` rồi suy ra là POSITION,
> và dựng một bảng 6 mục quanh giả định đó. Thực tế `custom_mode` chở thẳng
> `flight_mode_t` của firmware, và **2 = ALTHOLD**. Máy bay lúc đo đang ở chế độ
> giữ độ cao, không phải giữ vị trí.

Bản gốc cũng đã thống nhất *"firmware chốt con số, Pi bám theo"*. Con số thật,
lấy từ `fc_state.h`:

| `custom_mode` | Tên firmware | Ý nghĩa | Tình trạng |
|---|---|---|---|
| 0 | `FLIGHT_MODE_ACRO` | điều khiển tốc độ góc trực tiếp | đã có |
| 1 | `FLIGHT_MODE_ANGLE` | tự cân bằng theo góc nghiêng | đã có |
| 2 | `FLIGHT_MODE_ALTHOLD` | giữ độ cao (baro + laser) | đã có |
| 3 | `FLIGHT_MODE_POSHOLD` | giữ vận tốc bằng optical flow | đã có |
| 4 | `FLIGHT_MODE_OFFBOARD` | nhận lệnh vận tốc từ Pi | **đã có** — xem ghi chú dưới |

> **Cập nhật 2026-09-13** ([tra_loi_dot_2_pi4.md](tra_loi_dot_2_pi4.md) mục 2.1):
> `custom_mode = 4` **đã được phát** khi OFFBOARD chạy. Mất flow trong lúc OFFBOARD
> thì `custom_mode` tụt về `1` dù máy trạng thái OFFBOARD chưa thoát. `custom_mode`
> không cho biết Pi còn quyền hay không — dùng `NAMED_VALUE_INT` `OB_AUTH`.

Không có LAND và RTL. RTL cần một nguồn vị trí tuyệt đối mà bo mạch không có
(xem mục 2) — đừng đưa vào bảng cho tới khi có GPS hoặc VIO.

**`base_mode`.** FC luôn bật `MAV_MODE_FLAG_CUSTOM_MODE_ENABLED` (1) và
`MANUAL_INPUT_ENABLED` (64); bật thêm `STABILIZE_ENABLED` (16) ở mọi chế độ trừ
ACRO. Bit `MAV_MODE_FLAG_SAFETY_ARMED` (128) bám đúng `motor.armed` — đây là
nguồn duy nhất cho trường `armed` của `/mavros/state`.

Giá trị 81 đo được trong bản gốc (`64 | 16 | 1`) là trạng thái chưa arm, ứng với
một chế độ có cân bằng. Đúng như thiết kế.

---

## 6. Arming — vì sao Pi không arm được

> **LẠC HẬU từ 2026-09-13.** Phương án "còn để ngỏ" cuối mục này **đã được hiện
> thực**: Pi arm được khi người lái gạt ch8 + ch5 lên. Hợp đồng mới và bảng mã
> `COMMAND_ACK` ở [tra_loi_dot_2_pi4.md](tra_loi_dot_2_pi4.md) mục 0. Khi mâu thuẫn,
> tài liệu đó đúng.

**FC từ chối mọi lệnh ARM đến qua MAVLink. Đây là chủ ý, không phải thiếu sót.**

Lệnh `MAV_CMD_COMPONENT_ARM_DISARM` với `param1 = 1` luôn nhận
`MAV_RESULT_TEMPORARILY_REJECTED`. Chiều DISARM (`param1 = 0`) thì luôn được chấp
nhận — cắt khẩn cấp từ Pi phải luôn hoạt động.

Lý do đầy đủ nằm trong `mav_link.c`, tóm tắt: toàn bộ
`arming.c` đặt trên một quy tắc — arm phải là chủ ý của
người đang đứng cạnh máy bay, thể hiện bằng cách gạt công tắc trên tay điều
khiển. Cho arm qua MAVLink là mở đúng cái cửa đó: một node ROS 2 lỗi, hoặc chỉ
đơn giản là khởi động lại đúng lúc, sẽ làm cánh quạt quay khi có người đang cầm
máy bay.

**Nghĩa là với phía Pi:**

- `/mavros/cmd/arming` với `value: true` sẽ trả về **nhanh** (có ACK, không treo)
  nhưng `success: false`. **Đây là hành vi đúng, đừng báo lỗi.**
- `/mavros/cmd/arming` với `value: false` hoạt động bình thường — dùng làm nút
  cắt khẩn cấp.
- Quy trình bay: người lái arm bằng công tắc RC, sau đó Pi mới ra lệnh vận tốc.
  OFFBOARD **không cần** arm từ xa.

**Còn để ngỏ.** Nếu sau này thật sự cần arm từ Pi, cách giữ được nguyên tắc an
toàn là lấy công tắc RC làm cổng đồng ý: FC chỉ chấp nhận lệnh ARM khi công tắc
arming trên tay điều khiển **đang bật** — Pi chỉ "bấm nút" sau khi người lái đã
cho phép. Cần sửa `arming.c` thêm một API arm có điều kiện. Chưa làm.

---

## 7. Điểm cần lưu ý khác

**`AUTOPILOT_VERSION`.** MAVROS gửi `MAV_CMD_REQUEST_AUTOPILOT_CAPABILITIES`
(520) lúc kết nối, thử 5 lần rồi bỏ cuộc với cảnh báo `your FCU don't support
AUTOPILOT_VERSION, switched to default capabilities`. FC nay đã trả lời.

FC **chỉ khai `MAV_PROTOCOL_CAPABILITY_MAVLINK2`, không khai gì thêm** — cố ý.
Khai thừa (MISSION_FLOAT, FTP, SET_POSITION_TARGET_LOCAL_NED...) sẽ khiến MAVROS
bật những plugin mà firmware chưa hỗ trợ, rồi chờ phản hồi không bao giờ tới.
Khi giai đoạn 2 xong thì bổ sung đúng cờ tương ứng.

**Timestamp.** `ATTITUDE`, `LOCAL_POSITION_NED`, `GLOBAL_POSITION_INT` dùng
`time_boot_ms` lấy từ `HAL_GetTick()`. `HIGHRES_IMU` dùng `time_usec` tính bằng
mili giây × 1000 — **cố ý không dùng `micros()`**, vì hàm đó đọc TIM2 32 bit nên
tràn vòng sau 71 phút, đủ để EKF trên Pi sắp sai thứ tự phép đo.

**Ngắt UART8.** TX chạy DMA chế độ Normal, nên `HAL_UART_TxCpltCallback` tới từ
ngắt UART8 chứ không phải ngắt DMA. **UART8 global interrupt bắt buộc phải bật**,
nếu không đường gửi chết sau đúng một gói. Đã bật (ưu tiên 10).

---

## 8. Việc phải sửa ở phía Pi

Ghi lại ở đây để hai bên không đổ lỗi nhầm chỗ khi nghiệm thu.

**8.1 — Bảng `custom_mode`.** Sửa theo mục 5. Bảng 6 mục trong bản gốc
(STABILIZE / ALT_HOLD / POSITION / OFFBOARD / LAND / RTL) không khớp firmware.
Đặc biệt: đừng coi `custom_mode = 2` là "giữ vị trí".

**8.2 — `type_mask`.** Theo mục 4.3: `0x07C7` — bit 10 (`yaw`) = 1 (bỏ qua),
bit 11 (`yaw_rate`) = 0 (dùng). FC không có vòng góc yaw nên ra lệnh `yaw` tuyệt
đối bị loại cả khung; xem [tra_loi_cau_hoi_pi4.md](tra_loi_cau_hoi_pi4.md) Câu 2.

**8.3 — Vòng điều khiển ra vận tốc, không ra vị trí.** Node điều khiển phải quy
sai số vị trí (từ AprilTag / VIO) thành lệnh vận tốc rồi mới gửi xuống. Đừng gửi
`x`,`y`,`z` — FC bỏ qua. Lý do ở mục 2.

**8.4 — Sai topic hạ cánh chính xác.**
[landing_target_bridge_node.py:44](../src/drone_control/drone_control/landing_target_bridge_node.py#L44)
đang publish `mavros_msgs/LandingTarget` lên `/mavros/landing_target/raw`. Topic
đó **không tồn tại** trong MAVROS. Plugin `landing_target` thật sự cung cấp:

| Topic | Kiểu | Chiều |
|---|---|---|
| `/mavros/landing_target/pose` | `geometry_msgs/PoseStamped` | Pi → FC (sinh `LANDING_TARGET`) |
| `/mavros/landing_target/pose_in` | `geometry_msgs/PoseStamped` | FC → Pi |
| `/mavros/landing_target/lt_marker` | `geometry_msgs/Vector3Stamped` | FC → Pi |

Sai cả tên topic lẫn kiểu message. Ở trạng thái hiện tại dữ liệu AprilTag không
bao giờ tới được FC. Phải sửa trước khi thử hạ cánh chính xác — dù FC cũng chưa
xử lý `LANDING_TARGET` (mục 4.2), nên hai bên cùng còn việc.

**8.5 — Kỳ vọng đúng về arming.** `/mavros/cmd/arming{value: true}` trả
`success: false` là **đúng thiết kế**, không phải lỗi. Xem mục 6.

**8.6 — Hai node cùng publish setpoint, không có phân xử.**
`position_controller_node` và `fc_command_bridge_node` **cùng** publish
`/mavros/setpoint_raw/local`, mỗi bên một timer 20 Hz riêng. Timeout 500 ms ở mục
4.3 **sẽ không bảo vệ được**: luôn có gói tới nên FC không bao giờ hết hạn, nó chỉ
nhận setpoint mâu thuẫn xen kẽ nhau. Phải chốt một node duy nhất được quyền
publish. Việc này còn mâu thuẫn với docstring của `fc_command_bridge_node` — nó tự
khai là "điểm DUY NHẤT gọi API MAVROS".

**8.7 — Bẫy phần trăm pin, ưu tiên cao.** FC gửi `battery_remaining = -1` ("không
biết", cố ý — mục 3.2). MAVROS quy đổi thành `percentage = -0.01`.
`failsafe_monitor_node.on_battery` tính `battery_pct = percentage * 100 = -1.0`,
trong khi `critical_battery_pct = 25`/`15`. Nghĩa là **`periodic_check` sẽ kích
`ESCALATE_EMERGENCY_LAND` ngay khi vừa được hiện thực**. Hiện hàm đó còn là `TODO`
nên lỗi chưa phát tác. Phải coi `percentage < 0` là "không biết" trước khi viết thân hàm.

**8.8 — `optical_flow_node` nên kiểm tra sức khoẻ flow.** `vx`/`vy` trong
`LOCAL_POSITION_NED` mất ý nghĩa khi hết flow. Theo dõi bit
`MAV_SYS_STATUS_SENSOR_OPTICAL_FLOW` (`0x40`) trong trường **`sensors_health`**
của `/mavros/sys_status` (kiểu `mavros_msgs/SysStatus`) thay vì giả định số luôn
hợp lệ. Đã xác minh trường này đọc được qua MAVROS.

---

## 9. Checklist nghiệm thu

Chia hai giai đoạn theo **điều kiện thực hiện**. Drone hiện chưa cấp điện tầng
công suất động cơ, nên chỉ nhóm **9.A** thực hiện được.

Danh sách việc đầy đủ: [viec_can_lam_mavlink.md](viec_can_lam_mavlink.md).

## 9.A — Bàn test: FC chỉ cần cấp điện, drone đứng im

### 9.A.1 FC phát đủ bản tin

```bash
python3 -c "
from pymavlink import mavutil; from collections import Counter; import time
m = mavutil.mavlink_connection('/dev/ttyAMA0', baud=921600); m.wait_heartbeat()
c = Counter(); t0 = time.time()
while time.time()-t0 < 10:
    x = m.recv_match(blocking=True, timeout=2)
    if x: c[x.get_type()] += 1
for k,v in sorted(c.items()): print('%-24s %5.1f Hz' % (k, v/10.0))
"
```

- [x] `GLOBAL_POSITION_INT` ~10 Hz — đo **10.0**
- [x] `LOCAL_POSITION_NED` ~30 Hz — đo **30.3**
- [x] `HIGHRES_IMU` ~50 Hz — đo **50.0**
- [x] `BATTERY_STATUS` / `EXTENDED_SYS_STATE` ~1 Hz — đo **1.0** / **1.0**
- [x] Hết cảnh báo `VER: ... switched to default capabilities`
- [ ] `relative_alt` đổi đúng chiều khi **nhấc drone bằng tay**
- [ ] Dấu `vx`/`vy` đúng khi **dịch drone ngang bằng tay** trên sàn có vân *(A4.1)*
- [ ] Bit `MAV_SYS_STATUS_SENSOR_OPTICAL_FLOW` hạ khi **che cảm biến flow** *(A4.3)*

### 9.A.2 Topic ROS có dữ liệu

```bash
ros2 run mavros mavros_node --ros-args \
  --params-file install/drone_bringup/share/drone_bringup/config/mavros.yaml
ros2 topic hz /mavros/global_position/rel_alt
ros2 topic hz /mavros/imu/data_raw
ros2 topic echo /mavros/state --once
```

- [x] `/mavros/state` báo `connected: true`
- [x] `/mavros/global_position/rel_alt` ổn định — đo **9.997 Hz**
- [x] `/mavros/imu/data_raw` phát — đo **49.99 Hz**
- [x] `/mavros/vfr_hud` phát — đo **10.004 Hz**
- [ ] `mode` hiện `CMODE(n)` khớp bảng mục 5 khi **gạt công tắc chế độ RC**
      (đổi chế độ không cần arm)

## 9.B — Cần cấp điện tầng công suất động cơ

Không thực hiện, không đề xuất, cho tới khi có điện động cơ.

### 9.B.1 Luồng lệnh có ACK

```bash
ros2 service call /mavros/cmd/arming mavros_msgs/srv/CommandBool "{value: true}"
ros2 service call /mavros/cmd/arming mavros_msgs/srv/CommandBool "{value: false}"
```

- [ ] Lệnh `true` trả về **dưới 1 giây** với `success: false` — có ACK, không treo
- [ ] Lệnh `false` trả về `success: true` và máy bay disarm thật
- [ ] Arm bằng công tắc RC làm `/mavros/state` đổi `armed: true`
- [ ] `EXTENDED_SYS_STATE` chuyển `ON_GROUND` ↔ `IN_AIR` qua ngưỡng 0,5 m

### 9.B.2 Giai đoạn 2 — chưa áp dụng

Bỏ qua cho tới khi mục 4.2 hoàn thành: `/mavros/set_mode`,
`/mavros/setpoint_raw/local`, `/mavros/cmd/takeoff`, hạ cánh theo AprilTag.

---

## Phụ lục — tra cứu nhanh

- Định nghĩa bản tin chuẩn: <https://mavlink.io/en/messages/common.html>
- Ánh xạ plugin ↔ topic của MAVROS: <https://wiki.ros.org/mavros>
- Cấu hình plugin đang bật: [mavros.yaml](../src/drone_bringup/config/mavros.yaml)
- Lớp bản tin phía FC: `mav_link.c` / `mav_link.h`
- Lớp truyền tải UART8: `mav_port.c`
