# Phản hồi của phía Pi 4 cho đợt 2

> **⚠ ĐÃ CŨ — KHÔNG DÙNG LÀM ĐẶC TẢ (từ 2026-09-13).** Đã hợp nhất vào [GIAO_UOC_FC_ROS2.md](GIAO_UOC_FC_ROS2.md). Khi mâu thuẫn, giao ước đúng. Chỉ giữ lại để tra *lý do* của quyết định.

Phía Pi trả lời [tra_loi_dot_2_pi4.md](tra_loi_dot_2_pi4.md), ngày 2026-09-13.

Mọi số liệu dưới đây đo trên dây `/dev/ttyAMA0` @921600 (pymavlink) hoặc kiểm
trực tiếp trên MAVROS đang chạy trên Pi. Chỗ nào chưa kiểm được thì ghi rõ.

**Tóm tắt:** trả lời đủ 3 câu hỏi của các bạn, cả ba đều "đồng ý / có". Còn **2
việc cần các bạn trả lời** (mục 2) và **4 điểm cần chốt** trước khi viết code
cho trạng thái OFFBOARD và `DISTANCE_SENSOR` (mục 3).

---

## 1. Trả lời 3 câu hỏi

### 1.1 Đồng ý thứ tự?

> **Đáp: Đồng ý, giữ nguyên thứ tự các bạn đề xuất.**

| # | Việc |
|---|---|
| 1 | Trạng thái OFFBOARD lên dây (`NAMED_VALUE_INT` + `STATUSTEXT`) |
| 2 | `AUTOPILOT_VERSION` đầy đủ + `REQUEST_MESSAGE` (512) |
| 3 | `DISTANCE_SENSOR` (132) |
| 4 | `ODOMETRY` (331) |
| 5 | `RC_CHANNELS` (65) |

Đồng ý với lý do đặt việc 1 lên đầu: `STATUSTEXT` là sự kiện, node restart sẽ lỡ
mất, nên chỉ bản tin định kỳ mới cho node biết trạng thái hiện tại.
`OPTICAL_FLOW_RAD` và `VIBRATION` để sau, như các bạn nói.

### 1.2 Plugin `debug_value` có bật không?

> **Đáp: Trước đây TẮT. Nay ĐÃ BẬT và đã kiểm chứng.**

- Đã thêm `'debug_value'` vào `plugin_allowlist` trong
  [mavros.yaml](../src/drone_bringup/config/mavros.yaml).
- Chạy MAVROS với FC thật: log có `Plugin debug_value initialized`, và có topic
  `/mavros/debug_value/named_value_int` (kiểu `mavros_msgs/DebugValue`: `name`,
  `value_int`, `type = 4`).
- FC chưa phát `NAMED_VALUE_INT` nên topic hiện còn rỗng. Đúng như mong đợi.
- `STATUSTEXT` đã có sẵn ở `/mavros/statustext/recv` (plugin `sys_status`), không
  cần bật thêm gì.

Không cần thống nhất kênh khác. Cứ dùng `NAMED_VALUE_INT` như đề xuất.

### 1.3 Pi sẽ giữ `relative_alt` khi bỏ vận tốc?

> **Đáp: Có. Xác nhận theo đúng bảng ở mục 1.1 của các bạn.**

- Khi bit `OPTICAL_FLOW` tắt, Pi **chỉ bỏ** `vx`, `vy`, `x`, `y` (và
  `groundspeed`). **Giữ** `z`, `vz`, `relative_alt`, `alt`, `climb`. Không bỏ cả
  khung `GLOBAL_POSITION_INT`.
- Hiện tại không có rủi ro ấy: `optical_flow_node` đọc độ cao từ topic riêng
  `/mavros/global_position/rel_alt`, tách hẳn khỏi vận tốc.
- EKF trên Pi hiện lấy vận tốc từ camera của Pi, **chưa** dùng vận tốc của FC. Quy
  tắc trên sẽ áp khi nối vận tốc FC vào, hoặc chuyển sang `ODOMETRY`.

Đồng ý không đưa vận tốc về 0. Bộ đếm lỗi `errors_count1..4`: Pi sẽ cảnh báo
theo tốc độ tăng như các bạn gợi ý. Cảm ơn bảng ánh xạ.

---

## 2. Cần các bạn trả lời

### 2.1 `fields_updated` vẫn bật cờ từ kế — đợt 2 chưa nhắc tới

Đây là câu các bạn yêu cầu "báo ngay" ở đợt 1. Đợt 2 không có mục nào trả lời.

Đo lại ngày 2026-09-13, 30 giây:

| `HIGHRES_IMU.fields_updated` | Số mẫu | Tỉ lệ |
|---|---|---|
| `0x1BFF` (bit từ kế 6–8 **BẬT**) | 1482 | 98,8 % |
| `0x1A3F` (bit từ kế tắt) | 18 | 1,2 % |

Các bạn kỳ vọng `0x1A3F` vì từ kế chưa hiệu chuẩn (`calibrated = false`). Tức
cổng `calibrated` vẫn chưa được áp dụng cho trường này.

Thêm một quan sát cùng vùng: 1/61 mẫu `SYS_STATUS` có `health = 0x1010B`, bit từ
kế (`0x04`) tắt thoáng qua rồi bật lại. Có thể liên quan.

**Hỏi:** đây là lỗi đã biết đang chờ sửa, hay firmware đang cố ý khai từ kế hợp lệ?

### 2.2 "Chưa kiểm được đường arm từ Pi vì cần Pi nối UART8"

Pi **đang nối** với cổng MAVLink của FC. Theo mục 1 của
[thiet_ke_mavlink_fc.md](thiet_ke_mavlink_fc.md), cổng đó chính là UART8. Mọi phép
đo ở đợt 1, đợt 2 và tài liệu này đều đi qua đường đó.

**Hỏi:** câu trên có nghĩa là bàn test phía các bạn chưa có Pi, hay đường arm đi
qua một cổng khác UART8? Cần chốt để khỏi nhầm cổng.

**Lưu ý:** drone phía Pi hiện **chưa cấp điện tầng công suất động cơ**. Pi sẽ chưa
thử arm cho tới giai đoạn đó.

---

## 3. Cần chốt trước khi viết code

### 3.1 Trạng thái OFFBOARD (việc #1)

Pi sẽ viết `mission_manager_node` theo đúng bảng `OB_STATE` / `OB_AUTH` / `OB_EXIT`
của các bạn. Xin xác nhận 4 điểm:

1. **Tên chính xác.** `name` của `NAMED_VALUE_INT` dài tối đa 10 ký tự, không có
   ký tự kết thúc nếu đủ 10. Pi sẽ so khớp nguyên văn `OB_STATE`, `OB_AUTH`,
   `OB_EXIT`, phân biệt hoa thường. Xin giữ nguyên, đừng đổi.
2. **Phát ở mọi trạng thái.** Cả ba giá trị phát 2 Hz **kể cả khi chưa arm, ch8
   xuống, và `offboard_switch_channel = -1`**? Pi cần phân biệt được "FC chưa phát"
   với "FC báo không có quyền".
3. **`OB_AUTH` cập nhật ngay.** Khi Pi mất quyền, gói `NAMED_VALUE_INT` ngay kế tiếp
   có báo `OB_AUTH = 0` không, hay có độ trễ lọc nào khác? Pi chấp nhận trễ tới
   0,5 giây.
4. **Mất flow trong OFFBOARD.** Pi sẽ hiểu `OB_STATE = 2` và `custom_mode ≠ 4` là
   "OFFBOARD còn chạy nhưng setpoint của Pi không được dùng". Xin xác nhận cách hiểu
   này đúng, và `OB_EXIT` **không** đổi trong trường hợp đó.

Đề nghị thêm: `STATUSTEXT` dùng `severity` phân biệt, ví dụ `NOTICE` cho
`DANG_CHAY` và `TAT (CONG_TAC)`, `WARNING` cho mọi lần vào `KHOA`. Như vậy log trên
Pi lọc được theo mức.

### 3.2 `DISTANCE_SENSOR` (việc #3)

Đồng ý toàn bộ đề xuất cách điền: `type = LASER`, `orientation = PITCH_270`,
`covariance = 25`, `signal_quality = 1` khi không hợp lệ, không dùng mẹo
`max_distance + 1`.

- **`min_distance`:** chờ các bạn đo cận dưới thật. Phía Pi không có thông số này.
  Nếu phải phát bản tin trước khi đo xong, xin ghi rõ trong tài liệu con số đang
  điền là tạm, để Pi không dùng nó làm ngưỡng.
- **Tầm tin cậy 6 m vs 8 m:** Pi sẽ dùng `max_distance = 800` như đề xuất, và tự áp
  thêm ngưỡng 6 m khi dùng cho hạ cánh.

### 3.3 `AUTOPILOT_VERSION` (việc #2)

Đồng ý cả 4 thay đổi. Khi nạp bản mới, Pi sẽ kiểm: `capabilities = 0x2080`,
`flight_sw_version = 0x00010000`, `flight_custom_version` = git hash, và lệnh 512
(`param1 = 148`) được trả lời.

**Xin báo trước khi nạp.** Lần đo 2026-09-13 thấy `time_boot_ms` tăng liên tục, tức
FC không khởi động lại giữa hai lần đo. Biết lúc nào nạp thì Pi đo lại đúng lúc.

---

## 4. Việc phía Pi đã làm theo đợt 2

| Việc | Tình trạng |
|---|---|
| Bật `debug_value` trong `mavros.yaml` | **Xong**, đã kiểm chứng (mục 1.2) |
| Đánh dấu mục 5 và 6 của [thiet_ke_mavlink_fc.md](thiet_ke_mavlink_fc.md) là lạc hậu, trỏ về đợt 2 | **Xong** |
| Sửa tiêu chí nghiệm thu ARM theo bảng mã `COMMAND_ACK` mới | **Xong** (trong [viec_can_lam_mavlink.md](viec_can_lam_mavlink.md), C2b) |
| `mission_manager_node` theo hợp đồng arm + `OB_AUTH` | Chờ FC phát `NAMED_VALUE_INT` |
| Bộ lọc vận tốc theo bit flow (giữ độ cao) | Chưa cần — EKF chưa dùng vận tốc FC |

Theo dõi chi tiết ở [viec_can_lam_mavlink.md](viec_can_lam_mavlink.md).
