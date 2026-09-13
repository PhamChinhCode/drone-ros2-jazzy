# Phản hồi đợt 2 cho phía Pi 4

> **⚠ ĐÃ CŨ — KHÔNG DÙNG LÀM ĐẶC TẢ (từ 2026-09-13).** Đã hợp nhất vào [GIAO_UOC_FC_ROS2.md](GIAO_UOC_FC_ROS2.md). Khi mâu thuẫn, giao ước đúng. Chỉ giữ lại để tra *lý do* của quyết định.

Phía FC trả lời, ngày 2026-09-13. Phản hồi báo cáo "3. Các lỗi còn lại trên dây"
và "4. Còn thiếu".

Mọi kết luận đều đã kiểm chứng: đọc code, hoặc đo trên bo mạch qua console
USART1. Chỗ nào chưa kiểm được thì ghi rõ.

**Tóm tắt:** các bạn quan sát đúng ở hầu hết các điểm. Có **một tiền đề sai**
(`custom_mode = 4` đã được phát, chỉ là chưa ai vào được OFFBOARD thật), và
**một hiểu lầm có thể gây lỗi nặng** nếu áp dụng máy móc: khi bit flow tắt, chỉ
được bỏ **vận tốc ngang**, không được bỏ độ cao — xem mục 1.1.

---

## 0. Đọc trước: hợp đồng ARM đã đổi kể từ đợt 1

Báo cáo của các bạn chưa nhắc tới, nên có thể chưa biết. Tài liệu đợt 1 ghi
"ARM từ Pi luôn bị từ chối". **Điều đó không còn đúng.** Việc này ảnh hưởng
trực tiếp tới cách viết `mission_manager_node` (B7).

### Chế độ Pi

Công tắc **ch8** trên tay cầm (tham số `offboard_switch_channel = 7`, đếm từ 0)
nay **mặc định BẬT**. Khi ch8 lên, công tắc ARM **ch5** đổi nghĩa: không còn là
"arm ngay" mà thành **"cho phép Pi arm"**.

Quy trình phía người lái:

1. Cần ga ở **giữa**
2. ch8 **lên**
3. ch5 **lên** — FC không tự arm, đứng chờ lệnh từ Pi

Phía Pi:

4. Phát `SET_POSITION_TARGET_LOCAL_NED` 10–20 Hz **trước**
5. Gửi `MAV_CMD_COMPONENT_ARM_DISARM` `param1 = 1`
6. FC arm, và OFFBOARD tự vào khi đủ điều kiện

Pi được **tự do arm và disarm** mà không cần thêm thao tác nào từ tay cầm, kể cả
disarm rồi arm lại.

### Pi mất quyền khi nào

- Người lái **chạm bất kỳ cần nào** sau khi đã arm (roll, pitch, yaw, hoặc ga lệch
  khỏi giữa)
- Mất sóng RC
- Ngay khi mất quyền, Pi **không còn ra lệnh gì được nữa — kể cả DISARM**. Người
  lái vẫn luôn cắt được bằng ch5.

**Lấy lại quyền CHỈ bằng cách người lái gạt ch8 xuống rồi lên lại.** Node ROS 2
crash rồi tự restart **không** khôi phục quyền.

### Mã trả về của `COMMAND_ACK` cho ARM/DISARM

| `result` | Nghĩa | Pi nên làm gì |
|---|---|---|
| `ACCEPTED` (0) | đã làm, hoặc vốn đã ở trạng thái đó | tiếp tục |
| `DENIED` (2) | **Pi không có quyền**: ch8/ch5 chưa đúng, hoặc người lái đã giành lái | **đừng thử lại** — chờ người lái |
| `TEMPORARILY_REJECTED` (1) | có quyền nhưng chưa đủ điều kiện (ga chưa về giữa, cảm biến…) | thử lại sau là hợp lý |

Phân biệt `DENIED` với `TEMPORARILY_REJECTED` là chủ ý: một cái đòi người, cái
kia đòi thời gian.

Đã nghiệm thu trên bàn: ch8 + ch5 lên thì **không tự arm**; gạt ch8 xuống khi
ch5 còn lên thì **khoá chứ không arm**. Chưa kiểm được đường arm từ Pi vì cần Pi
nối UART8.

---

## 1. Các lỗi còn lại trên dây

### 1.1 Vận tốc vẫn được gửi khi bit flow tắt

> **Đáp: Đúng, và đó là hợp đồng. FC không đưa về 0. Phía Pi phải tự bỏ mẫu (B12)
> — nhưng CHỈ bỏ vận tốc ngang, KHÔNG bỏ độ cao.**

Quan sát của các bạn **xác nhận bản vá đợt 1 đã chạy đúng trên dây**: bit
`OPTICAL_FLOW` trong `SYS_STATUS.onboard_control_sensors_health` tắt đúng lúc
bộ ước lượng mất tin cậy. Trước bản vá, bit này bật suốt.

Vì sao không đưa về 0: vận tốc bằng 0 là một **lời nói dối khác** — nó bảo EKF
"máy bay đứng yên chắc chắn", trong khi thật ra FC **không biết**. 0,1 m/s trôi
có bit báo hiệu thì trung thực hơn 0 m/s không có gì báo hiệu.

**Quan trọng — bỏ đúng trường:**

| Bản tin | Trường | Bit flow tắt thì |
|---|---|---|
| `LOCAL_POSITION_NED` | `vx`, `vy` | **bỏ** |
| `LOCAL_POSITION_NED` | `x`, `y` | **bỏ** (đứng yên, không trôi, nhưng vô nghĩa) |
| `LOCAL_POSITION_NED` | `z`, `vz` | **GIỮ** — từ EKF độ cao, không phụ thuộc flow |
| `GLOBAL_POSITION_INT` | `vx`, `vy` | **bỏ** |
| `GLOBAL_POSITION_INT` | `relative_alt`, `alt`, `vz` | **GIỮ** |
| `VFR_HUD` | `groundspeed` | bỏ (chỉ để hiển thị) |
| `VFR_HUD` | `alt`, `climb` | giữ |

Nếu bỏ **cả khung** `GLOBAL_POSITION_INT` khi bit flow tắt thì
`optical_flow_node` mất `relative_alt` — mà trên bàn flow gần như luôn tắt,
nên node sẽ **không bao giờ nhận được độ cao**. Đó chính là chỗ tôi lo khi đọc
"Pi phải tự bỏ mẫu".

Giải pháp sạch về lâu dài là `ODOMETRY` (331) với covariance lớn khi vận tốc
không hợp lệ — xem mục 2.4.

### 1.2 `errors_count2 = 2`, `errors_count4 = 1`

> **Đáp: Là bộ đếm lỗi cộng dồn từ lúc khởi động của từng cảm biến. Hai con số
> này đứng yên nghĩa là lành mạnh. Theo dõi TỐC ĐỘ TĂNG, không phải giá trị.**

Tài liệu đợt 1 thiếu phần này — lỗi của tôi. Ánh xạ, lấy từ `send_sys_status()`:

| Trường | Nguồn | Tăng khi |
|---|---|---|
| `errors_count1` | IMU ICM20602 | mẫu toàn 0, tràn nhịp DRDY, lỗi SPI |
| `errors_count2` | **máy thu CRSF (tay cầm)** | khung ngắn, **sai CRC**, lỗi UART |
| `errors_count3` | baro BMP388 | lỗi I2C |
| `errors_count4` | **MTF01P (flow + laser)** | khung ngắn, sai CRC, lỗi UART |
| `errors_comm`, `drop_rate_comm` | — | luôn 0, chưa dùng |

Kiểu `uint16`, không bao giờ tự xoá, tràn vòng ở 65 535.

Đo trên bo mạch ngay khi nhận báo cáo:

| Cảm biến | Lỗi | Trong đó CRC | Tổng số khung | Tỉ lệ |
|---|---|---|---|---|
| CRSF | 2 | **2** | 101 674 | 0,002 % |
| MTF01P | 1 | 0 | 441 482 | 0,0002 % |

Cả hai là nhiễu lúc cấp nguồn hoặc cắm dây, không phải lỗi đang diễn ra. Đứng
yên suốt 30 giây như các bạn đo là đúng hành vi lành mạnh.

**Gợi ý cho Pi:** cảnh báo khi một bộ đếm **tăng quá N lần trong 10 giây**, đừng
cảnh báo theo giá trị tuyệt đối.

### 1.3 `AUTOPILOT_VERSION` gần như rỗng, `REQUEST_MESSAGE` (512) bị `UNSUPPORTED`

> **Đáp: Đúng cả ba điểm. Hai trong số đó còn là lỗi lạc hậu phía FC. Sẽ sửa.**

| Trường | Hiện tại | Sẽ sửa thành | Lý do |
|---|---|---|---|
| `capabilities` | `0x2000` (MAVLINK2) | **`0x2080`** | thêm `SET_POSITION_TARGET_LOCAL_NED` (128) — **đã hiện thực** nhưng quên khai |
| `flight_sw_version` | `0` | **`0x00010000`** | firmware 0.1.0, loại `DEV`, theo mã hoá chuẩn MAVLink |
| `flight_custom_version` | rỗng | **8 byte git hash** lúc build | biết chính xác Pi đang nói chuyện với bản nào |
| `MAV_CMD_REQUEST_MESSAGE` (512) | `UNSUPPORTED` | trả lời khi `param1 = 148` | cách hỏi mới của MAVLink, thay cho lệnh 520 |

Chú thích trong code còn ghi *"khi giai đoạn 2 xong thì bổ sung đúng cờ"* —
giai đoạn 2 đã xong mà chưa bổ sung.

Vẫn **cố ý không khai** các cờ khác (MISSION, FTP, PARAM_FLOAT…): khai thừa thì
MAVROS bật plugin rồi chờ phản hồi không bao giờ tới.

### 1.4 Pin (B4) chưa kiểm được

> **Đáp: Đúng như dự đoán khi chưa gắn pin.**

`voltages[] = 65535` là vì FC **tự nhận dạng số cell lúc cắm pin** — chưa có pin
thì `cell_count = 0`, nên cả 10 ô đều là `UINT16_MAX` ("không dùng"). Khi gắn
pin, kỳ vọng đúng N ô đầu mang giá trị **điện áp tổng chia N** (FC không đo từng
cell), các ô còn lại 65535.

---

## 2. Còn thiếu

### 2.1 Pi không biết trạng thái OFFBOARD

> **Đáp: Tiền đề một nửa sai, một nửa đúng. `custom_mode = 4` ĐÃ được phát. Nhưng
> các bạn đúng ở chỗ quan trọng hơn: không phân biệt được TAT với KHOA, và đó là
> thứ B7 cần. Đồng ý ưu tiên — thậm chí nên đặt TRÊN `DISTANCE_SENSOR`.**

**Phần sai của tiền đề.** HEARTBEAT lấy `custom_mode` thẳng từ `g_fc.ctrl.mode`,
và trường này được gán `FLIGHT_MODE_OFFBOARD = 4` ngay khi OFFBOARD chạy
([ctrl_angle.c](../Control/ctrl_angle.c), dòng `want = FLIGHT_MODE_OFFBOARD`).
Các bạn chưa thấy vì chưa lần nào vào được OFFBOARD thật — việc đó cần arm và
có setpoint. Chữ "dành sẵn" là chỗ tài liệu đợt 1 chưa cập nhật.

Một chi tiết cần biết: nếu mất optical flow **trong lúc** OFFBOARD, FC tụt về
ANGLE và `custom_mode` thành `1` — dù máy trạng thái OFFBOARD bên trong vẫn chưa
thoát. Tức `custom_mode = 4` trả lời trung thực câu *"setpoint của Pi có đang
được dùng không"*, nhưng không trả lời *"Pi còn quyền không"*.

**Phần đúng.** `custom_mode` không có cách nào nói KHOA hay TAT, cũng không nói
Pi còn quyền hay đã mất.

**Vì sao STATUSTEXT một mình KHÔNG đủ.** STATUSTEXT là sự kiện, bắn một lần lúc
chuyển trạng thái. Kịch bản B7 phải xử lý chính là **node crash rồi restart** —
node restart xong **đã lỡ mất sự kiện đó**, và không có cách nào hỏi lại trạng
thái hiện tại. Nó sẽ tưởng mình còn quyền.

**Đề xuất phía FC** — hai kênh bổ sung cho nhau:

**Kênh máy đọc — `NAMED_VALUE_INT` (252), phát định kỳ 2 Hz:**

| `name` | Giá trị |
|---|---|
| `OB_STATE` | 0 = KHOA, 1 = TAT, 2 = DANG_CHAY |
| `OB_AUTH` | 1 = Pi còn quyền, 0 = đã mất |
| `OB_EXIT` | lý do rời gần nhất, bảng dưới |

Định kỳ nên node restart lúc nào cũng biết trạng thái trong vòng 0,5 giây.

**Kênh người đọc — `STATUSTEXT` (253), bắn khi chuyển trạng thái:**

```
OFFBOARD: DANG_CHAY
OFFBOARD: KHOA (DAY_CAN)
OFFBOARD: TAT (CONG_TAC)
```

Để log và màn hình trạm mặt đất đọc được ngay.

Mã `OB_EXIT`:

| Mã | Tên | Nghĩa | Pi có tự phục hồi được không |
|---|---|---|---|
| 0 | — | chưa từng rời | — |
| 1 | KHOI_DONG | khoá sẵn từ lúc bật nguồn | không — chờ người lái gạt ch8 |
| 2 | CONG_TAC | người lái gạt ch8 xuống | không |
| 3 | HET_HAN | quá 500 ms không có setpoint hợp lệ | **không** — dù Pi đã sống lại |
| 4 | DAY_CAN | người lái chạm cần | không |
| 5 | DISARM | disarm khi đang OFFBOARD | **có** — Pi arm lại được |
| 6 | KEP_DAI | lệnh vượt giới hạn bao liên tục | không |
| 7 | MAT_GOC | bộ ước lượng mất góc | không |
| 8 | MAT_SONG | mất sóng RC | không |

**Cần các bạn xác nhận:** plugin `debug_value` có đang bật trong
[mavros.yaml](../src/drone_bringup/config/mavros.yaml) không? Đó là plugin sinh
`/mavros/debug_value/named_value_int`. Nếu đang tắt, cần bật, hoặc thống nhất một
kênh khác.

### 2.2 `DISTANCE_SENSOR` (132) — thông tin tầm đo MTF01P (B8)

> **Đáp: Cung cấp dưới đây. Lưu ý đây là giới hạn FIRMWARE ĐANG ÁP, không phải
> thông số datasheet đã kiểm chứng.**

| Đại lượng | Giá trị | Nguồn |
|---|---|---|
| `max_distance` | **8,0 m** | firmware coi xa hơn là không hợp lệ (`FLOW_RANGE_MAX_MM`) |
| Tầm EKF tin | 6,0 m | xa hơn, bộ ước lượng chuyển sang tin baro (`EST_RANGE_MAX_M`) |
| `min_distance` | **chưa xác định** | firmware không đặt cận dưới |
| Nghiêng tối đa | 25° | quá thì EKF bỏ mẫu laser |
| Nhiễu đo | 0,05 m (1σ) | giá trị EKF đang dùng — dùng được cho `covariance` |
| Tần số | 100 Hz | |

Đo trên bàn: đọc ổn định **0,19–0,23 m**, `range_quality = 255`. Cận dưới thật
cần đo bằng cách hạ dần tấm chắn tới khi mất tín hiệu — **chưa làm**.

Đề xuất điền bản tin: `max_distance = 800` cm, `min_distance` tạm đặt theo số đo
khi có, `type = MAV_DISTANCE_SENSOR_LASER`, `orientation = PITCH_270` (hướng
xuống), `covariance = 25` (đơn vị cm², tức σ = 5 cm — đặc tả cho phép tối đa
σ = 6 cm).

Báo "ngoài tầm / không hợp lệ" bằng trường **`signal_quality = 1`** — đặc tả
định nghĩa rõ `1 = invalid signal`. Khi hợp lệ, điền `range_quality` của MTF01P
quy về 2–100 %. Không dùng mẹo `current_distance = max_distance + 1`: đó là quy
ước riêng của vài firmware, đặc tả MAVLink không định nghĩa.

### 2.3 `RC_CHANNELS` (65) để biết vị trí công tắc cho B7

> **Đáp: Làm được, nhưng KHÔNG dùng nó để suy ra quyền của Pi.**

Vị trí công tắc **khác** quyền. Ví dụ: ch8 đang lên nhưng người lái vừa chạm cần
→ **KHOA**, Pi mất quyền, trong khi `RC_CHANNELS` vẫn báo ch8 lên. Suy quyền từ
công tắc là tự dựng lại máy trạng thái của FC ở phía Pi — và sẽ lệch.

Dùng `OB_AUTH` ở mục 2.1 cho quyết định. `RC_CHANNELS` chỉ để hiển thị.

Nếu làm, FC sẽ đổi đơn vị CRSF (172–1811) sang µs (988–2012) theo đúng đặc tả.

### 2.4 Các bản tin còn lại

| Bản tin | Tình trạng phía FC |
|---|---|
| `ODOMETRY` (331) | Làm được. Covariance **có sẵn nội bộ** trong cả hai EKF; `ekf_velocity_uncertainty_mps()` đã được khai ở đợt 1. Giải quyết dứt điểm mục 1.1. Lưu ý đó là bộ lọc tự chấm điểm chính nó — dùng làm trọng số tương đối, đừng coi là sai số tuyệt đối. |
| `OPTICAL_FLOW_RAD` (106) | Cần đo hệ số quy đổi từ đếm thô sang radian trước. Không gấp, đồng ý. |
| `VIBRATION` (241) | FC chưa tính phương sai gia tốc — việc mới. Để giai đoạn C, đồng ý. |

---

## 3. Kế hoạch phía FC — chờ các bạn chốt thứ tự

| # | Việc | Chặn gì bên Pi | Công sức |
|---|---|---|---|
| 1 | **Trạng thái OFFBOARD lên dây** (`NAMED_VALUE_INT` + `STATUSTEXT`) | B7 | nhỏ |
| 2 | `AUTOPILOT_VERSION` đầy đủ + `REQUEST_MESSAGE` (512) | — | nhỏ |
| 3 | `DISTANCE_SENSOR` (132) | C6 | nhỏ, cần chốt `min_distance` |
| 4 | `ODOMETRY` (331) | đưa vận tốc FC vào EKF | vừa |
| 5 | `RC_CHANNELS` (65) | hiển thị | nhỏ |

Đề xuất đặt **1 lên đầu**, trên `DISTANCE_SENSOR`: không có nó thì
`mission_manager_node` không viết đúng được, còn hạ cánh chính xác thì đằng nào
cũng cần B7 chạy trước.

**Cần các bạn trả lời:**

1. Đồng ý thứ tự trên?
2. Plugin `debug_value` có bật không (mục 2.1)?
3. Mục 1.1: xác nhận Pi sẽ **giữ** `relative_alt` khi bỏ vận tốc?

**Lưu ý về tài liệu:** [thiet_ke_mavlink_fc.md](thiet_ke_mavlink_fc.md) mục 6 và
[thiet_ke_offboard_failsafe.md](thiet_ke_offboard_failsafe.md) vẫn còn ghi
"ARM từ Pi luôn bị từ chối" và "custom_mode 4 dành sẵn". Khi hai tài liệu đó và
file này mâu thuẫn, **file này đúng** cho tới khi chúng được cập nhật.
