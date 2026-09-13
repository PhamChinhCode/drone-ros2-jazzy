# Trả lời 9 câu hỏi từ phía Pi 4

> **⚠ ĐÃ CŨ — KHÔNG DÙNG LÀM ĐẶC TẢ (từ 2026-09-13).** Đã hợp nhất vào [GIAO_UOC_FC_ROS2.md](GIAO_UOC_FC_ROS2.md). Khi mâu thuẫn, giao ước đúng. Chỉ giữ lại để tra *lý do* của quyết định.

Phía FC trả lời, ngày 2026-09-12.

**Cách đọc:** mỗi câu có một dòng **Đáp** ngắn gọn, rồi mới tới căn cứ. Nếu chỉ
cần con số để viết code, đọc dòng Đáp là đủ.

Mọi câu đều đã kiểm chứng — hoặc đọc thẳng code, hoặc đo trên bo mạch thật qua
console USART1. Chỗ nào chưa kiểm được thì ghi rõ là chưa.

Có **ba chỗ các bạn quan sát đúng mà tài liệu của tôi sai**, và một chỗ quan sát
của các bạn không khớp với firmware. Đều nêu ở dưới.

---

# Nhóm 1 — Chặn việc viết code điều khiển

## Câu 1. Dấu trục với `coordinate_frame = 8` (BODY_NED)

> **Đáp: FRD, đúng như các bạn đoán. `vy` dương = bay sang PHẢI. `vz` dương = đi
> XUỐNG. Firmware không diễn giải theo FLU ở bất kỳ chỗ nào.**

Căn cứ, lần theo đúng đường dữ liệu:

| Bước | Code | Kết quả |
|---|---|---|
| 1 | `mav_link.c` → `ctrl_offboard_set_target(sp.vx, sp.vy, sp.vz, …)` | truyền thẳng, không đổi dấu |
| 2 | [ctrl_offboard.c](../Control/ctrl_offboard.c) | `fwd = vx`, `right = vy`, `climb = **-vz**` |
| 3 | [ctrl_poshold.c](../Control/ctrl_poshold.c) | `tgt_fwd = ext.x`, `tgt_right = ext.y` |
| 4 | `ctrl_poshold.c` | `roll_deg = +(kp·err_right + I)` — sai số sang phải sinh nghiêng phải |

Dấu trừ ở bước 2 chính là chỗ đổi từ "Z xuống dương" của NED sang "lên dương"
của vòng giữ độ cao. Nó nằm ở **một chỗ duy nhất** trong toàn bộ đường dữ liệu.

**Về lo ngại MAVROS tự đảo dấu.** Lo ngại đó đúng chỗ, nhưng hợp đồng giữa hai
bên nằm ở **byte trên dây**, không phải ở ngữ nghĩa message ROS. FC đọc đúng
FRD. Nếu MAVROS đã chuyển FLU→FRD giúp các bạn thì hai bên khớp nhau; nếu node
của các bạn tự đóng gói theo FLU rồi bỏ qua lớp chuyển đổi thì lệch dấu.

**Đừng suy luận — đo.** Xem mục "Quy trình kiểm dấu" bên dưới: kiểm được toàn bộ
dấu của cả bốn trục mà **không cần arm, không cần cánh quạt**.

## Câu 2. Trường `yaw` — tuyệt đối hay tương đối

> **Đáp: Câu hỏi này đã hết hiệu lực. FC KHÔNG đọc `yaw` ở bất kỳ dạng nào.
> Khung nào ra lệnh `yaw` tuyệt đối sẽ bị TỪ CHỐI CẢ KHUNG. Chỉ dùng `yaw_rate`,
> đơn vị rad/s, dương = quay phải (chiều kim đồng hồ nhìn từ trên).**

Các bạn bắt đúng một mâu thuẫn thật: *"hệ thân mà yaw tuyệt đối là mâu thuẫn ngữ
nghĩa"*. Tài liệu cũ của tôi sai ở chỗ đó. Nhưng nguyên nhân gốc sâu hơn:

**Firmware không có vòng góc yaw.** [ctrl_angle.c:186](../Control/ctrl_angle.c#L186)
chỉ điều khiển *tốc độ* yaw, kèm lý do đã ghi sẵn từ trước: *"Không có la bàn thì
yaw ước lượng trôi dần, giữ hướng theo nó là tự làm máy bay quay đi."* Và từ kế
hiện **chưa hiệu chuẩn** (Câu 7), nên kể cả muốn cũng chưa có hướng bắc đáng tin.

`type_mask` chốt là **`0x07C7`**, trong đó **bit 10 (`yaw`) phải BẬT** = bỏ qua:

| Bit | Trường | Phải là | Nghĩa |
|---|---|---|---|
| 0–2 | `x`,`y`,`z` | `111` | bỏ qua |
| 3–5 | `vx`,`vy`,`vz` | `000` | **dùng** |
| 6–8 | `afx`,`afy`,`afz` | `111` | bỏ qua |
| 9 | `force` | `1` | bỏ qua |
| 10 | `yaw` | `1` | **bỏ qua** |
| 11 | `yaw_rate` | `0` | **dùng** |

Bit 11 được phép bật (không ra lệnh yaw) — đó là trường hợp hợp lệ, không phải
lỗi. Mọi sai khác khác → khung bị loại, bộ đếm `loai` tăng, và FC hết hạn
setpoint rồi về POSHOLD (phanh, treo).

**Lợi thế ngoài dự tính:** vì FC không đụng tới `yaw`, toàn bộ mớ biến đổi yaw
khác nhau giữa frame 1 và frame 8 của MAVROS trở thành vô hại. Còn `yaw_rate` là
đại lượng thuần hệ thân, không phụ thuộc frame.

**Dấu `yaw_rate`:** suy ra từ code chứ không phải từ quy ước chung. Trong
[ctrl_poshold.c](../Control/ctrl_poshold.c), phép xoay là `v_fwd = vn·cos(yaw) +
ve·sin(yaw)`. Đặt yaw = +90°: `v_fwd = ve`, tức mũi hướng đông. Vậy yaw đo theo
chiều kim đồng hồ từ hướng bắc — chuẩn NED. `yaw_rate` dương do đó là **mũi quay
sang phải**.

## Câu 3. Vào OFFBOARD bằng đường nào

> **Đáp: CHỈ bằng công tắc RC. `SET_MODE` chưa hiện thực — đừng gửi, nó không có
> tác dụng và cũng không báo lỗi. Nhưng công tắc chưa đủ: Pi phải ĐANG PHÁT
> setpoint TRƯỚC khi người lái gạt công tắc.**

Các bạn nói đúng, ba mục trong tài liệu cũ mâu thuẫn nhau. Tôi đã sửa. Đây là
điều kiện thật, lấy từ nhánh `OFFBOARD_DISABLED` trong
[ctrl_offboard.c](../Control/ctrl_offboard.c) — **phải đúng đủ cả sáu cùng lúc**:

1. công tắc `offboard_switch_channel` đang bật, và sóng RC còn đọc được
2. đã arm (`motor.armed` và `mode == FC_MODE_ARMED`)
3. `est.attitude_valid`
4. **đã nhận được ít nhất một setpoint hợp lệ**
5. setpoint đó còn trong hạn `offboard_timeout_ms` (500 ms)
6. người lái không đang đẩy cần

Điều kiện 4 và 5 là thứ ảnh hưởng trực tiếp tới `mission_manager_node`:

> **Pi phải bắt đầu phát `SET_POSITION_TARGET_LOCAL_NED` ở 10–20 Hz TRƯỚC, rồi
> người lái mới gạt công tắc.** Gạt công tắc lúc Pi còn im lặng thì không có
> chuyện gì xảy ra — FC ở yên trạng thái `TAT`, không báo lỗi. Đây là chủ ý: vào
> chế độ rồi hết hạn ngay sau đó là kiểu hỏng khó chẩn đoán hơn nhiều.

Node nên phát setpoint vận tốc 0 làm nhịp giữ chỗ, rồi mới chuyển sang lệnh thật.

**Và OFFBOARD không tự vào lại.** Nếu nó rơi ra vì bất kỳ hỏng hóc nào (hết hạn,
mất sóng, đẩy cần, kẹp dải), trạng thái thành `KHOA` và **chỉ** thoát được bằng
cách gạt công tắc xuống rồi lên lại. Node ROS 2 crash rồi tự restart sẽ **không**
làm máy bay chạy lại. Chi tiết ở
[thiet_ke_offboard_failsafe.md](thiet_ke_offboard_failsafe.md).

**Mặc định `offboard_switch_channel = -1`, tức TẮT HẲN.** Phải bật bằng tay:
`set offboard_switch_channel=7` rồi `save`.

---

## Quy trình kiểm dấu — trả lời dứt điểm Câu 1 và 2, không cần cánh quạt

Đây là phần quan trọng nhất của tài liệu này. Các bạn nói đúng rằng sai dấu
"không lộ ra khi test trên bàn" — nhưng **có một cách làm nó lộ ra**.

`ctrl_offboard_set_target()` nhận và lưu setpoint **bất kể máy bay đang ở trạng
thái nào**, kể cả `KHOA` và chưa arm. Còn lệnh CLI `offboard` in ra mục tiêu đã
nhận **bằng từ vựng của FC**, không phải của MAVLink:

```
vtoi_mms   = mục tiêu bay TỚI,     mm/s (dương = tới)
vphai_mms  = mục tiêu bay SANG PHẢI, mm/s (dương = phải)
vlen_mms   = mục tiêu ĐI LÊN,       mm/s (dương = lên)
yaw_mdps   = mục tiêu quay,      mđộ/s (dương = phải)
```

**Cách làm:** cắm FC (cánh quạt đã tháo, không cần arm), mở console USART1 ở
921600. Từ Pi, phát lặp một setpoint, rồi gõ `offboard` trên console.

| Pi ra lệnh | Kỳ vọng trên console | Nếu thấy dấu ngược |
|---|---|---|
| bay tới 0,5 m/s | `vtoi_mms = 500` | sai dấu trục X |
| bay sang phải 0,5 m/s | `vphai_mms = 500` | **MAVROS đang đảo dấu Y** |
| đi xuống 0,3 m/s | `vlen_mms = -300` | **MAVROS đang đảo dấu Z** |
| quay phải 30 °/s | `yaw_mdps = 30000` | sai dấu yaw |

Đồng thời xem ba bộ đếm để tách biệt ba kiểu hỏng:

- `nhan` tăng, `loai` = 0 → định dạng đúng, chỉ còn chuyện dấu
- `loai` tăng → sai `type_mask` hoặc `coordinate_frame`, khung bị vứt
- `kep` tăng → đúng định dạng nhưng vượt giới hạn bao

Làm xong bước này thì Câu 1 và Câu 2 đóng lại bằng phép đo, không còn phải suy
luận qua tầng biến đổi của MAVROS.

---

# Nhóm 2 — Ảnh hưởng chất lượng EKF

## Câu 4. `LOCAL_POSITION_NED.x` và `.y` luôn bằng 0

> **Đáp: Firmware CÓ hiện thực tích phân vị trí, và không liên quan tới arm. Nó
> chỉ tích phân khi `ekf_velocity_is_valid()` đúng — tức khi optical flow còn
> khoá. Trên bàn không có khoá flow, nên `x`/`y` đứng yên ở 0.**

Đây là đoạn quyết định, [estimator.c:276](../Estimator/estimator.c#L276):

```c
g_fc.est.velocity_mps.x = ekf_velocity_north();   /* ghi VÔ ĐIỀU KIỆN */
g_fc.est.velocity_mps.y = ekf_velocity_east();

if (ekf_velocity_is_valid()) {                    /* chỉ tích phân khi hợp lệ */
    if (s_pos_primed) {
        g_fc.est.position_m.x += g_fc.est.velocity_mps.x * dt;
        ...
```

**Vận tốc được phát ra vô điều kiện, vị trí thì không.** Đó chính xác là thứ các
bạn đo được: `vx = -0.133` nhưng `x = 0.0`. Hai trường này không mâu thuẫn nhau —
chúng đang nói hai chuyện khác nhau, và FC hiện **không có cách nào nói cho các
bạn biết điều đó**. Xem Câu 6.

`ekf_velocity_is_valid()` đòi một mẫu flow được chấp nhận gần đây; quá
`est_flow_timeout_ms` là hết hạn. Trên bàn, cột `tin` của console đúng là báo
`HET`.

## Câu 5. Sai số góc thật của bộ ước lượng

> **Đáp: 1° cho roll/pitch là lựa chọn ĐÚNG, cứ dùng. Yaw thì đừng dùng làm
> hướng tuyệt đối ở bất cứ giá trị nào — từ kế chưa hiệu chuẩn nên yaw không có
> tham chiếu tuyệt đối nào cả.**

**Và đây là cảnh báo quan trọng nhất mục này:** firmware CÓ phát ra một con số
tên là "độ không tin cậy góc", đo được **0,071°** trên bàn
(`ekf_attitude_uncertainty_deg()`, cột `sig_deg` ở console mode 16). **Đừng dùng
nó làm `orientation_stdev`.** Nó là *hiệp phương sai nội bộ của bộ lọc* — bộ lọc
tự chấm điểm chính nó, và nó không mô hình hoá lệch lắp đặt IMU, trôi bias gia
tốc kế, hay sai số do rung khi có điện động cơ. Con số 0,032° các bạn đo được là
cùng một cái bẫy, nhìn từ phía khác.

Trực giác của các bạn đúng: dùng nhầm sẽ khiến EKF tin thái quá.

**Số đo thật tôi vừa lấy trên bàn**, 45 giây, bo mạch nằm im, console mode 16:

| Đại lượng | Đo được | Ghi chú |
|---|---|---|
| Trôi yaw sau `yawzero` | **0,00°/phút** (dưới độ phân giải 0,01°) | ở trạng thái nghỉ, nhiệt độ ổn định |
| `sig_deg` (bộ lọc tự chấm) | 0,071° | **không phải sai số tuyệt đối** |
| `sig_m` (độ cao) | 0,01 m | cùng cảnh báo |
| Bias gyro còn lại | 0,00 / 0,01 / 0,01 °/s | sau hiệu chuẩn |

Trôi yaw 0°/phút **khi nằm im** nghe rất tốt, nhưng **không suy ra được cho lúc
bay**: rung làm bias gyro đổi, nhiệt độ tăng cũng vậy. Chưa ai đo trôi yaw khi
có điện động cơ.

**Chưa từng có ai đo sai số góc TUYỆT ĐỐI trên bo mạch này** — sẽ cần bàn xoay
hoặc mặt phẳng chuẩn. Vậy nên: **dùng 1°** cho roll/pitch.

Có một cách làm tốt hơn cho tương lai, xem Câu 8: firmware đã tính sẵn
`ekf_velocity_uncertainty_mps()` (1σ, sống theo thời gian thực) nhưng chưa khai
trong header nên chưa phát ra được. Phát nó ra thì các bạn có covariance thật
thay vì hằng số đoán.

## Câu 6. Trôi optical flow 13 cm/s khi nằm im

> **Đáp: KHÔNG phải sàn nhiễu của MTF01P, và KHÔNG phải lỗi hiệu chuẩn. Đó là
> ước lượng mà FC đang tự đánh dấu là KHÔNG HỢP LỆ, nhưng vẫn phát ra. Đây là
> LỖI CỦA FC, không phải của các bạn.**

Khi `ekf_velocity_is_valid()` sai, bộ lọc không còn phép đo nào để hiệu chỉnh và
chỉ còn **tích phân gia tốc kế thuần tuý** — thứ trôi rất nhanh. 13 cm/s sau vài
chục giây nằm im là đúng tầm của kiểu trôi đó. Trên mặt bàn cách sàn 22 cm,
MTF01P gần như chắc chắn không khoá được flow.

**Đừng tăng covariance để bù.** Con số đó không phải nhiễu — nó là một ước lượng
đã hỏng. Tăng covariance chỉ làm EKF của các bạn nuốt nó chậm hơn thay vì loại
hẳn.

**Lỗi của tôi, hai tầng:**

1. `send_local_position_ned()` phát `velocity_mps` vô điều kiện, không có cách
   nào báo hiệu tính hợp lệ.
2. Tài liệu mục 3.2 bảo các bạn theo dõi bit `MAV_SYS_STATUS_SENSOR_OPTICAL_FLOW`
   trong `SYS_STATUS`. **Cách đó không dùng được.** Bit ấy lấy từ
   `sys.sensor_health`, nghĩa là "MTF01P còn nói chuyện", chứ không phải "EKF tin
   vận tốc của nó". Trên bàn, `sensor_health = 0x051F` — **bit flow đang BẬT**
   trong khi vận tốc vô hiệu. Bit đó trả lời sai câu hỏi.

**Cách khắc phục tôi đề nghị** (chưa làm, chờ các bạn xác nhận là đúng thứ cần):

> Đổi bit `OPTICAL_FLOW` trong trường `onboard_control_sensors_health` của
> `SYS_STATUS` để nó phản ánh `est.position_valid` — vốn được gán đúng bằng
> `ekf_velocity_is_valid()`. Sửa một dòng, và biến bit đang trả lời sai câu hỏi
> thành bit trả lời đúng câu hỏi.

Đến lúc đó, quy tắc cho phía Pi là: **`vx`/`vy` của `LOCAL_POSITION_NED` chỉ dùng
được khi bit flow trong `SYS_STATUS` bật.** Bit tắt thì bỏ hẳn mẫu đó, đừng hạ
trọng số.

Giải pháp sạch hơn về lâu dài là `ODOMETRY` (331) có covariance đi kèm — xem
Câu 8.

## Câu 7. Từ kế đã hiệu chuẩn chưa

> **Đáp: CHƯA. Và quan sát "bit 6–8 đang bật" của các bạn không khớp với firmware
> — cổng chặn đang đóng đúng như thiết kế.**

Log khởi động vừa đọc từ bo mạch, nguyên văn:

```
Tu ke qua sensor hub: TAT (mag_source khac SHUB).
QMC5883P @ 0x2C
  CHUA HIEU CHUAN - chay DBG_MODE_MAGCAL truoc khi dung cho giu huong.
```

`g_fc.mag.calibrated` được tính từ tham số hiệu chuẩn trong
[mag_i2c.c:251](../Drivers/mag_i2c.c#L251): "đã hiệu chuẩn" = bộ offset/scale
**không còn là ma trận đơn vị**. Hiện vẫn là đơn vị → `calibrated = false` →
`send_highres_imu()` **không bật** bit 6–8.

Nhờ các bạn kiểm lại: đọc trực tiếp `fields_updated` trên dây (kỳ vọng
`0x1A3F`, tức chỉ gia tốc + con quay + khí áp), hay con số đó suy ra từ tài liệu
của tôi? Nếu trên dây thật sự thấy bit từ kế bật thì có lỗi ở phía tôi và tôi
cần biết ngay.

**Hai chỗ tài liệu của tôi sai, phát hiện khi kiểm câu này:**

- Chip là **QMC5883P** trên I2C trực tiếp, **không phải** QMC6309 qua sensor hub
  của LSM6DSV. Chú thích trong [fc_state.h](../State/fc_state.h) đã lạc hậu và
  tôi chép nhầm theo.
- Vậy nên: **`MAV_FRAME_LOCAL_NED` vẫn bị chặn.** Cổng đúng như mục 4.3 đã nói,
  và nó đang đóng. Muốn mở thì chạy `DBG_MODE_MAGCAL`, `save`, rồi kiểm lại bit
  từ kế trong `fields_updated` — bit đó bật lên chính là tín hiệu cổng đã mở.

---

# Nhóm 3 — Mở rộng dữ liệu

## Câu 8. Năm bản tin đề nghị

Xếp theo tỉ lệ giá trị trên công sức:

| Bản tin | Dữ liệu đã có? | Công sức | Đánh giá |
|---|---|---|---|
| **`DISTANCE_SENSOR` (132)** | đủ: `range_mm`, `range_valid`, `range_quality` | thấp | **Làm. Đồng ý đây là món đáng giá nhất.** |
| **`ODOMETRY` (331)** | pose/twist có; covariance **có nội bộ, chưa phơi ra** | trung bình | **Làm sau `DISTANCE_SENSOR`.** Xem ghi chú dưới. |
| `RC_CHANNELS` (65) | có, nhưng đơn vị CRSF (172..1811) | thấp | Làm được, phải đổi sang µs |
| `OPTICAL_FLOW_RAD` (106) | thiếu hệ số quy đổi sang radian | trung bình | Cần đo hệ số trước |
| `VIBRATION` (241) | **chưa có** — phải tính phương sai gia tốc | cao | Để lúc có điện động cơ |

**`ODOMETRY` giải quyết luôn Câu 5 và Câu 6, và rẻ hơn vẻ ngoài.** Cả hai EKF
đều đã giữ ma trận hiệp phương sai nội bộ, và `ekf_velocity.c` **đã viết sẵn**:

```c
float ekf_velocity_uncertainty_mps(void)
{
    const float v = 0.5f * (s_n.P[0][0] + s_e.P[0][0]);
    return sqrtf(fmaxf(v, 0.0f));
}
```

Hàm này **chưa được khai trong `ekf_velocity.h`** nên chưa ai gọi được. Khai một
dòng là có ngay 1σ vận tốc sống theo thời gian thực. `ekf_attitude_uncertainty_deg()`
thì đã khai rồi.

Nhưng nhắc lại cảnh báo ở Câu 5: đó là **bộ lọc tự chấm điểm chính nó**. Dùng làm
covariance tương đối (lúc nào tin nhiều hơn lúc nào) thì tốt; coi là sai số tuyệt
đối thì không.

**`DISTANCE_SENSOR` cần các bạn cho thêm một thông tin:** MAVROS muốn
`min_distance`/`max_distance`. Tầm thật của MTF01P là bao nhiêu trong điều kiện
bay của dự án? Tôi sẽ điền theo đó thay vì chép datasheet.

**Chưa làm cái nào cả** — chờ các bạn chốt thứ tự ưu tiên, vì mỗi bản tin thêm
vào là thêm một thứ phải nghiệm thu.

## Câu 9. `COMMAND_ACK` đã đầy đủ chưa

> **Đáp: Rồi. Mọi `COMMAND_LONG` gửi tới đúng địa chỉ đều được trả ACK, kể cả
> lệnh không hỗ trợ. VÀ các bạn kiểm chứng được ngay bây giờ, không cần arm,
> không cần điện động cơ.**

Trong `handle_command_long()`, biến `result` khởi tạo bằng
`MAV_RESULT_UNSUPPORTED`, và lệnh đóng gói ACK nằm **sau** toàn bộ chuỗi if/else,
không nằm trong nhánh nào. Đường duy nhất thoát ra mà không ACK là khi
`target_system` không phải của FC — đúng như đặc tả.

`target_system`/`target_component` của ACK lấy thẳng từ `sysid`/`compid` của
khung gửi tới, tức 255/190 với MAVROS.

**Ba phép thử an toàn tuyệt đối** (không đụng tới phần cứng nào):

| Gửi | Kỳ vọng | Kiểm được điều gì |
|---|---|---|
| `MAV_CMD_REQUEST_AUTOPILOT_CAPABILITIES` (520) | `ACCEPTED` + `AUTOPILOT_VERSION` | đường ACK chiều thành công |
| `MAV_CMD_NAV_TAKEOFF` (22) | `UNSUPPORTED` | ACK cho lệnh không hỗ trợ |
| `MAV_CMD_COMPONENT_ARM_DISARM` (400), `param1 = 0` | `ACCEPTED` | disarm — vô hại khi đang không arm |

Lệnh thứ ba là **disarm**, không phải arm: gửi lúc máy bay đang không arm thì
không có gì xảy ra, nhưng vẫn kiểm được trọn đường ACK. Không cần chờ tới giai
đoạn có điện động cơ.

Còn ARM (`param1 = 1`) sẽ trả `TEMPORARILY_REJECTED` — **đúng thiết kế**, xem
[thiet_ke_mavlink_fc.md](thiet_ke_mavlink_fc.md) mục 6.

---

# Tổng kết: việc phát sinh từ đợt hỏi này

**Phía FC — tôi sẽ làm, chờ các bạn xác nhận thứ tự:**

1. **Sửa bit `OPTICAL_FLOW` trong `SYS_STATUS`** cho nó phản ánh
   `est.position_valid` thay vì sức khoẻ driver (Câu 6). Ưu tiên cao nhất —
   đang chặn việc chỉnh EKF của các bạn.
2. Khai `ekf_velocity_uncertainty_mps()` trong header (Câu 5, 8).
3. `DISTANCE_SENSOR` (132), sau khi có thông số tầm đo.
4. `ODOMETRY` (331) kèm covariance.
5. Sửa chú thích lạc hậu: QMC5883P chứ không phải QMC6309.

**Phía Pi:**

1. Chạy **quy trình kiểm dấu** ở Nhóm 1 → đóng Câu 1 và 2 bằng phép đo.
2. `mission_manager_node`: phát setpoint **trước** khi gạt công tắc; đừng gửi
   `SET_MODE`; xử lý được việc OFFBOARD vào `KHOA` và không tự phục hồi.
3. Đặt `orientation_stdev = 1°` cho roll/pitch. Không dùng yaw làm hướng tuyệt
   đối cho tới khi hiệu chuẩn xong từ kế.
4. Bỏ hẳn mẫu `vx`/`vy` khi bit flow tắt — đừng hạ trọng số.
5. Xác nhận giúp `fields_updated` đọc được trên dây (Câu 7).
6. Cho biết tầm đo thật của MTF01P (Câu 8).

**Chưa ai đo, cả hai bên nên biết:**

- Sai số góc tuyệt đối (cần bàn xoay/mặt phẳng chuẩn)
- Trôi yaw **khi có điện động cơ** — con số 0°/phút ở trên chỉ đúng lúc nằm im
- Tần số thật của năm bản tin mới, đo từ phía Pi
