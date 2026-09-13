# Giao ước FC ↔ ROS 2 — bản hợp nhất

**Hợp đồng liên thông giữa flight controller STM32H743 và ROS 2 trên Raspberry Pi 4.**

| | |
|---|---|
| Phiên bản hợp đồng | **1.1** |
| Ngày | 2026-09-13 |
| Trạng thái | Đang hiệu lực |
| Phạm vi | Mọi thứ đi qua đường dây MAVLink giữa FC và Pi. Kiến trúc nội bộ mỗi bên nằm ngoài phạm vi. |

---

## 0. Cách dùng tài liệu này

### 0.1 Vì sao có bản này

Trước bản này, hợp đồng FC↔Pi nằm rải ở năm tài liệu viết theo lối hỏi–đáp qua lại
theo thời gian. Hệ quả: **cùng một câu hỏi có ba câu trả lời khác nhau ở ba chỗ**, và
chỗ đúng lại là chỗ mới nhất chứ không phải chỗ có tiêu đề nghe chính thức nhất.
Ba ví dụ thật đã xảy ra:

- `type_mask` là `0x0BC7` (bản đầu) hay `0x07C7` (bản sửa).
- "ARM từ Pi **luôn** bị từ chối" (đợt 1) hay "Pi arm được sau khi người lái gạt ch8+ch5"
  (đợt 2).
- `custom_mode = 4` là "dành sẵn" (đợt 1) hay "đã phát" (đợt 2).

Bản này gộp cả năm tài liệu đó thành **một nguồn sự thật duy nhất**, mỗi câu hỏi
đúng một câu trả lời.

### 0.2 Thứ tự ưu tiên khi mâu thuẫn

1. **Phép đo trên dây** — luôn thắng mọi tài liệu, kể cả bản này.
2. **Tài liệu này.**
3. Năm tài liệu nguồn ở mục 0.5 — chỉ dùng khi cần biết *lý do* một quyết định, không
   dùng làm đặc tả.

Ai đo được một chỗ tài liệu này sai thì **sửa ngay vào đây**, đừng viết một tài liệu
phản hồi mới. Bài học rút ra từ lịch sử dự án: mỗi tài liệu phản hồi mới lại sinh thêm
một nguồn sự thật cạnh tranh.

> **BẢN GỐC DUY NHẤT (chốt 2026-09-13):**
> **`drone-ros2-jazzy` → `docs/GIAO_UOC_FC_ROS2.md`**
> (<https://github.com/PhamChinhCode/drone-ros2-jazzy/blob/main/docs/GIAO_UOC_FC_ROS2.md>).
>
> Repo firmware giữ một **bản sao chỉ đọc** ở `App/Docs/GIAO_UOC_FC_ROS2.md`, đồng bộ từ bản
> gốc. **Mọi sửa đổi — kể cả của phía FC — làm ở bản gốc**, rồi chép sang bản sao.
>
> Lý do phải chốt: ngày 09-13 hai bên sửa hai bản song song (FC lên 1.1 ở repo firmware, Pi sửa
> `sysid` và thêm câu #6, #7 ở repo ROS). Hệ quả thấy ngay: Pi hỏi lại hai mâu thuẫn mà FC đã
> sửa. Hai bản đã được hợp nhất ba chiều (bản gốc 1.0 + hai nhánh), **3 chỗ xung đột** được
> phân giải bằng tay, không bên nào mất sửa đổi.
>
> Mỗi lần sửa ghi **một dòng vào bảng lịch sử cuối tài liệu**, kể cả sửa không tăng số, để hai
> bên so được mình đang cầm bản nào.

### 0.3 Quy ước trạng thái

Mỗi mục đặc tả mang một trong bốn nhãn:

| Nhãn | Nghĩa | Được viết code dựa vào chưa |
|---|---|---|
| **[CHỐT]** | Hai bên đã thống nhất **và** đã chạy thật trên dây | Được |
| **[THOẢ THUẬN]** | Hai bên đã thống nhất, chưa hiện thực xong ở ít nhất một bên | Viết được, chưa test được |
| **[ĐANG LÀM]** | Đã giao việc cho một bên, đang chờ | Không |
| **[ĐỀ XUẤT]** | Một bên đề nghị, bên kia chưa trả lời | Không |

Không có nhãn = **[CHỐT]**.

### 0.4 Quy ước đặt tên trong tài liệu

- **FC** = firmware STM32H743 (repo riêng, không nằm trong `ros2_ws`).
- **Pi** = ROS 2 Jazzy trên Raspberry Pi 4, gồm cả MAVROS.
- "Trên dây" = byte MAVLink thật giữa hai bên. Hợp đồng nằm ở đây, **không** nằm ở
  ngữ nghĩa của message ROS.

### 0.5 Tài liệu nguồn đã hợp nhất

| Tài liệu | Ngày | Vai trò còn lại |
|---|---|---|
| `thiet_ke_mavlink_fc.md` | 09-12 | lịch sử; mục 5, 6 đã lạc hậu |
| `tra_loi_cau_hoi_pi4.md` | 09-12 | lý do kỹ thuật của Câu 1–9, quy trình kiểm dấu |
| `tra_loi_dot_2_pi4.md` | 09-13 | hợp đồng ARM mới, bảng `OB_EXIT` |
| `phan_hoi_dot_2_fc.md` | 09-13 | xác nhận của Pi |
| `viec_can_lam_mavlink.md` | 09-13 | **vẫn dùng** làm sổ theo dõi việc; bảng mục 11 dưới đây là bản rút gọn |

Tài liệu kiến trúc ROS 2 (`thiet_ke_kien_truc_node_ros2.md`, `ke_hoach_thiet_ke_node.md`,
`huong_dan_chi_tiet_cac_node_ros2_pi4.md`, `CAMERA.md`, `RUNBOOK.md`) **không** bị bản này
thay thế — chúng mô tả phần nội bộ phía Pi. Chỗ nào chúng chạm vào hợp đồng thì mục 8
dưới đây là bản đúng.

---

## 1. Ranh giới trách nhiệm — vòng nào đóng ở đâu

Mục này quyết định mọi mục còn lại, nên đặt lên đầu.

```
  Pi 4 — ROS 2, Linux không thời gian thực
    nhiệm vụ, AprilTag, VIO
        └─> VÒNG VỊ TRÍ ─> lệnh VẬN TỐC ──┐  10–20 Hz
                                          │  SET_POSITION_TARGET_LOCAL_NED
                                          ▼
  FC — STM32H743, vòng lặp cứng
    VÒNG VẬN TỐC ─> góc nghiêng ─> VÒNG GÓC ─> VÒNG TỐC ĐỘ GÓC ─> trộn động cơ
     (ctrl_poshold)                (ctrl_angle)   (ctrl_rate)
```

**Pi gửi xuống VẬN TỐC. Không phải vị trí, không phải góc nghiêng.** Ba lý do, rút từ
code thật:

1. **FC đã có sẵn đúng vòng đó.** `ctrl_poshold` tuy mang tên "poshold" nhưng chuỗi của
   nó là `vận tốc mong muốn → P+I → góc nghiêng`. Cho Pi điều khiển vận tốc chỉ là đổi
   nguồn setpoint; mọi tầng dưới và mọi gain đã tune giữ nguyên.

2. **Lệnh vị trí bị chính bộ ước lượng cấm.** `est.position_m.x/y` là tích phân vận tốc
   optical flow — dẫn đường suy tính thuần tuý, sai số tích luỹ **không bao giờ tự hết**.
   Ra lệnh vị trí tuyệt đối lên một ước lượng đang trôi = máy bay bay để bù cho phần trôi
   không có thật. FC cũng chưa có vòng P vị trí.

3. **Lệnh góc sai ở chế độ hỏng.** Gửi góc tức là đẩy vòng vận tốc lên Linux không thời
   gian thực. Khi Pi khựng: lệnh góc → máy bay **giữ nguyên độ nghiêng và tăng tốc đi
   mất**; lệnh vận tốc → FC hết hạn setpoint, mục tiêu về 0 → **phanh rồi treo**. Khác
   biệt này là toàn bộ vấn đề.

Thêm một điểm không hiển nhiên: `ctrl_poshold_update()` trả `false` khi mất optical flow
và `ctrl_angle` tự lùi về ANGLE. **Lưới an toàn đó chỉ tồn tại ở tầng vận tốc.** Pi ra
lệnh góc thì hoàn toàn không biết flow đã chết.

Hệ quả: **vòng vị trí vẫn có, nhưng nằm trên Pi** — nơi có AprilTag và VIO làm tham
chiếu tuyệt đối. Pi quy sai số vị trí ra lệnh vận tốc. Vòng ngoài vốn chậm nên 10–20 Hz
là đủ và miễn nhiễm với jitter đường truyền.

### 1.1 Bảng phân vai, dùng để tra khi có tranh cãi

| Việc | Chủ | Ghi chú |
|---|---|---|
| Vòng tốc độ góc, vòng góc, trộn động cơ | FC | Pi không chạm |
| Vòng vận tốc (ngang + lên/xuống) | FC | Pi chỉ đặt setpoint |
| Vòng vị trí, dẫn đường, nhiệm vụ | Pi | |
| Hiệu chuẩn IMU/từ kế, EKF góc, EKF độ cao | FC | |
| EKF vị trí/vận tốc mức hệ thống | Pi | `robot_localization` |
| Quyết định ARM cuối cùng | **Người lái, qua RC** | Pi chỉ "bấm nút" sau khi được cho phép |
| Quyết định vào/rời OFFBOARD | FC | Pi không có `SET_MODE` |
| Failsafe mức cơ (mất sóng, mất góc, kẹp dải) | FC | |
| Failsafe mức nhiệm vụ (pin, mất GCS, mất marker) | Pi | |

---

## 2. Liên kết vật lý và phiên bản giao thức

| Hạng mục | Giá trị |
|---|---|
| Đường truyền | UART. Pi GPIO14 (TX) ↔ FC **PE0/RX**; Pi GPIO15 (RX) ↔ FC **PE1/TX**; **GND chung** |
| Ngoại vi phía FC | **UART8**, DMA2 (TX Normal, RX Circular) |
| Thiết bị phía Pi | `/dev/ttyAMA0` (PL011, bật bằng `dtoverlay=disable-bt`) |
| Baudrate | **921600**, 8N1, không flow control |
| Giao thức | **MAVLink v2** (byte mở đầu `0xFD`) |
| `sysid`/`compid` của FC | `1` / `1` (`MAV_COMP_ID_AUTOPILOT1`) |
| `sysid`/`compid` của Pi (MAVROS) | **`1` / `191`** (`MAV_COMP_ID_ONBOARD_COMPUTER`) — mặc định MAVROS, `mavros.yaml` không đặt. Đọc bằng `ros2 param dump /mavros/mavros` ngày 09-13. *(Bản 1.0 ghi `255/190` — đó là địa chỉ của script `pymavlink` dùng để đo, không phải MAVROS.)* **FC không lọc theo địa chỉ nguồn nên cả hai đều chạy.** Xem 11.1 #6 |
| `HEARTBEAT` của Pi | 1 Hz, `type = ONBOARD_CONTROLLER` (`/mavros/sys` `heartbeat_rate = 1.0`). **Lưu ý:** FC hiện **không** dùng heartbeat này làm cổng an toàn — timeout 3000 ms có trong code nhưng không được gọi (11.1 #8) |
| Băng thông đang dùng | **9,04 KB/s = 9,8 %** — Pi đo 40 s ngày 09-13 sau khi FC thêm `DISTANCE_SENSOR` và nhóm `NAMED_VALUE_INT` (trước đó 7,6–7,8 KB/s). Khớp ước tính ≈ 10 %, vẫn rất rộng |

**Không dùng `/dev/ttyS0`** (mini UART): baud của nó bám xung nhịp VPU nên trôi ở 921600.

UART8 dành riêng cho MAVLink. USART3 chở khung nhị phân riêng tới ESP32, USART1 là console
CLI. **Không trộn ba đường này.**

Cấu hình phía Pi: `src/drone_bringup/config/mavros.yaml`.

---

## 3. Hệ quy chiếu và dấu — phần dễ rơi máy bay nhất

Đọc kỹ mục này trước khi viết bất kỳ dòng code điều khiển nào.

### 3.1 Trên dây là FRD / NED, không bàn cãi

| Trục | Dương nghĩa là |
|---|---|
| `vx` | bay **tới trước** |
| `vy` | bay **sang phải** |
| `vz` | đi **XUỐNG** |
| `yaw_rate` | **quay phải** (chiều kim đồng hồ nhìn từ trên), rad/s |

Firmware không diễn giải FLU ở bất kỳ đâu. Đường dữ liệu phía FC:
`mav_link.c → ctrl_offboard_set_target(vx, vy, vz)` truyền thẳng không đổi dấu;
`ctrl_offboard.c` đặt `fwd = vx`, `right = vy`, `climb = -vz` — **dấu trừ duy nhất trong
toàn bộ đường dữ liệu**, đúng chỗ đổi từ "Z xuống dương" của NED sang "lên dương" của
vòng giữ độ cao.

Dấu `yaw_rate` suy từ code chứ không từ quy ước chung: trong `ctrl_poshold.c`,
`v_fwd = vn·cos(yaw) + ve·sin(yaw)`; đặt yaw = +90° cho `v_fwd = ve`, tức mũi hướng đông
→ yaw đo theo chiều kim đồng hồ từ bắc, chuẩn NED.

### 3.2 MAVROS ĐỔI DẤU giúp bạn — và đó là bẫy

Node phía Pi publish theo chuẩn ROS (**FLU/ENU**); MAVROS tự đổi sang FRD/NED trước khi
lên dây. Phép biến đổi đo thật được:

```
FRAME_BODY_NED (8):   (vx, vy, vz) → (vx, -vy, -vz);   yaw → -yaw
FRAME_LOCAL_NED (1):  (vx, vy, vz) → (vy,  vx, -vz);   yaw → π/2 - yaw
```

**Quy tắc bắt buộc phía Pi:** node điều khiển tính và publish theo **FLU/ENU** như mọi
node ROS bình thường, rồi để MAVROS đổi. **Tuyệt đối không tự tính theo NED rồi publish
thẳng** — làm vậy `vy` và `vz` sẽ đảo dấu trên dây, và lỗi này **không lộ ra khi test
trên bàn**.

### 3.3 `yaw` tuyệt đối trở thành vô nghĩa — và đó là may mắn

Vì FC **không đọc trường `yaw`** (mục 5.2), toàn bộ khác biệt biến đổi yaw giữa frame 1
và frame 8 của MAVROS trở thành vô hại. `yaw_rate` là đại lượng thuần hệ thân, không phụ
thuộc frame.

### 3.4 Quy trình kiểm dấu — đóng mục này bằng phép đo, không cần cánh quạt [ĐANG LÀM]

`ctrl_offboard_set_target()` nhận và lưu setpoint **bất kể máy bay ở trạng thái nào**, kể
cả `KHOA` và chưa arm. Lệnh CLI `offboard` trên console USART1 in mục tiêu đã nhận bằng
từ vựng của FC:

```
vtoi_mms   mục tiêu bay TỚI,        mm/s  (dương = tới)
vphai_mms  mục tiêu bay SANG PHẢI,  mm/s  (dương = phải)
vlen_mms   mục tiêu ĐI LÊN,         mm/s  (dương = lên)
yaw_mdps   mục tiêu quay,           mđộ/s (dương = phải)
```

Cách làm: tháo cánh quạt, cấp điện FC (không cần arm, không cần điện động cơ), mở console
USART1 @921600. Từ Pi phát lặp một setpoint, gõ `offboard` trên console.

| Pi ra lệnh (ROS, FLU) | Kỳ vọng trên console | Thấy ngược nghĩa là |
|---|---|---|
| bay tới 0,5 m/s | `vtoi_mms = 500` | sai dấu trục X |
| bay sang trái 0,5 m/s (FLU `y+`) | `vphai_mms = -500` | MAVROS **không** đảo dấu Y như giả định |
| đi lên 0,3 m/s (FLU `z+`) | `vlen_mms = 300` | sai dấu Z |
| quay trái 30 °/s (FLU yaw`+`) | `yaw_mdps = -30000` | sai dấu yaw |

> Cột "kỳ vọng" ở trên là **dự đoán** khi node publish đúng chuẩn ROS (FLU) và MAVROS áp
> phép biến đổi đo được ở 3.2. Riêng `yaw_rate` là suy diễn: phép đo 3.2 chỉ xác nhận
> `yaw → -yaw`, chưa xác nhận `yaw_rate`. Thấy dấu ngược thì trước khi đổ lỗi cho MAVROS,
> hãy kiểm node của mình đang publish FLU hay đã tự tính sang NED — đó là nguyên nhân
> thường gặp hơn.

Đồng thời đọc ba bộ đếm của console để tách ba kiểu hỏng:

| Bộ đếm | Tăng nghĩa là |
|---|---|
| `nhan` tăng, `loai` = 0 | định dạng đúng, chỉ còn chuyện dấu |
| `loai` tăng | sai `type_mask` hoặc `coordinate_frame` → **khung bị vứt cả gói** |
| `kep` tăng | định dạng đúng nhưng lệnh vượt giới hạn bao |

**Đây là việc đáng làm nhất hiện nay.** Làm xong thì mục 3.1–3.2 đóng bằng phép đo thay
vì suy luận qua tầng biến đổi của MAVROS.

> **Console USART1 nằm ở máy bàn test phía FC, không ở chỗ Pi** (câu hỏi 11.1 #7). Hai cách:
>
> **Ngay bây giờ — làm chung một phiên.** Pi phát setpoint, phía FC gõ `offboard` trên console
> và đọc lại số. Phía FC đọc được cả qua SWD, không cần chạm vào máy bay.
>
> **[THOẢ THUẬN — Pi đồng ý 09-13, mời FC hiện thực] Cho Pi tự làm được một mình — đi qua
> `NAMED_VALUE_FLOAT`, KHÔNG qua bản tin vị trí chuẩn.** *Pi đồng ý cả lập luận bỏ `POSITION_TARGET_LOCAL_NED`.
> Pi đã kiểm bằng FC giả: topic `/mavros/debug_value/named_value_float` có sẵn, tên 9 ký tự
> `OB_T_YAWR` và giá trị âm `-30.0` đi qua nguyên văn (`type = 3`). Không cần Pi sửa gì thêm.
> Đề nghị phát **cả khi chưa có setpoint nào** (giá trị 0) để Pi phân biệt "chưa nhận" với
> "FC chưa phát" — cùng lý do như `OB_*` ở 6.3.* FC phát 2 Hz mục tiêu đã nhận (sau kẹp dải), bằng **đúng từ vựng vật lý
> của console**:
>
> | `name` | Đơn vị | Dương nghĩa là |
> |---|---|---|
> | `OB_T_FWD` | m/s | bay **tới** |
> | `OB_T_RGT` | m/s | bay **sang phải** |
> | `OB_T_UP` | m/s | đi **lên** |
> | `OB_T_YAWR` | °/s | quay **phải** |
>
> Kèm ba bộ đếm `OB_RX_OK`, `OB_RX_REJ`, `OB_RX_CLP` thay cho `nhan`/`loai`/`kep`. Pi đọc ở
> `/mavros/debug_value/named_value_float` và `..._int`. Có mấy thứ này thì 3.4 không cần console.
>
> **Vì sao KHÔNG dùng `POSITION_TARGET_LOCAL_NED` (85) — bản 09-13 trước đã đề xuất nhầm.** Bản
> tin đó đi qua MAVROS nên bị đổi hệ quy chiếu **cả hai chiều**: Pi publish FLU → MAVROS đổi sang
> FRD lên dây → FC phát lại FRD → MAVROS đổi ngược về FLU. Phép đổi đi và phép đổi về triệt tiêu
> nhau, nên số Pi đọc lại **luôn khớp** số Pi gửi — kể cả khi MAVROS đổi sai dấu. Một phép thử
> không bao giờ trượt thì không kiểm được gì. `NAMED_VALUE_FLOAT` thì MAVROS chuyển nguyên văn,
> không biến đổi, nên Pi so được ý định vật lý của mình ("sang **trái** 0,5") với nhãn vật lý
> của FC (`OB_T_RGT = -0.5`) — đúng như phép thử bằng console.
>
> **Giới hạn chung của cả console lẫn cách này:** chúng kiểm **giao diện** (Pi → dây → FC đọc
> đúng trường), **không** kiểm vòng điều khiển có thật sự bay đúng hướng. Việc đó cần phép thử
> vật lý ở giai đoạn có điện động cơ (12.B).

---

## 4. FC → Pi: bảng phát

### 4.1 Đang phát — đã đo thật

Đo qua `pymavlink` trên `/dev/ttyAMA0` ngày 2026-09-12 và đo lại 09-13 (20 s + 30 s + 30 s,
**0 khung hỏng**, tần số không đổi).

| Message | ID | Tần số danh nghĩa | Đo được | Topic MAVROS sinh ra |
|---|---|---|---|---|
| `HEARTBEAT` | 0 | 1 Hz | 1.0 | `/mavros/state` |
| `SYS_STATUS` | 1 | 2 Hz | 2.1 | `/mavros/sys_status`, `/mavros/battery` |
| `ATTITUDE` | 30 | 50 Hz | 50.0 | `/mavros/imu/data` |
| `LOCAL_POSITION_NED` | 32 | 30 Hz | 30.3 | `/mavros/local_position/pose`, `.../velocity_local` |
| `GLOBAL_POSITION_INT` | 33 | 10 Hz | 10.0 | `/mavros/global_position/rel_alt`, `.../global` |
| `VFR_HUD` | 74 | 10 Hz | 10.0 | `/mavros/vfr_hud` |
| `HIGHRES_IMU` | 105 | 50 Hz | 50.0 | `/mavros/imu/data_raw`, `/mavros/imu/mag` |
| `BATTERY_STATUS` | 147 | 1 Hz | 1.0 | `/mavros/battery` |
| `EXTENDED_SYS_STATE` | 245 | 1 Hz | 1.0 | `/mavros/extended_state` |
| `COMMAND_ACK` | 77 | theo sự kiện | đã kiểm | kết quả `/mavros/cmd/*` |
| `AUTOPILOT_VERSION` | 148 | khi được hỏi | đã trả lời | hết cảnh báo `VER` |
| `DISTANCE_SENSOR` | 132 | 20 Hz | 20.0 | `/mavros/mtf01p` (`sensor_msgs/Range`) — **tên topic lấy từ khoá cấu hình**, mục 8.3 |
| `NAMED_VALUE_INT` | 252 | 2 Hz × 4 tên | 8.0 (mỗi tên 2.0) | `/mavros/debug_value/named_value_int` |

Tần số topic ROS đo trên MAVROS: `/mavros/imu/data` 49.99, `/mavros/imu/mag` 48.05,
`/mavros/local_position/pose` 30.30, `/mavros/global_position/rel_alt` 9.997,
`/mavros/sys_status` 2.00, `/mavros/battery` 1.00, `/mavros/extended_state` 1.00,
`/mavros/mtf01p` 20.2, `/mavros/debug_value/named_value_int` 8.0 (09-13).

**Đo 09-13, 40 s, sau khi FC nạp bản 1.1** (`flight_custom_version` = `8b9b35f7b2223c3a`):

- `DISTANCE_SENSOR`: `min_distance = 1`, `max_distance = 800`, `type = 0` (LASER),
  `id = 0`, `orientation = 25` (PITCH_270), `covariance = 25`, FOV = 0, quaternion = 0,
  `signal_quality = 100` ở 800/800 mẫu, `current_distance` 17–19 cm. Trên ROS:
  `min_range = 0.01`, `max_range = 8.0`, `range ≈ 0.18`, **`variance = 0.0`** — MAVROS
  **không** chuyển `covariance` sang `Range.variance`. Pi dùng σ = 0,05 m ở 9.6.
- `NAMED_VALUE_INT`: `OB_STATE = 1`, `OB_AUTH = 1`, `OB_EXIT = 1`, `FC_CTR_VER = 10100`,
  mỗi tên đúng 2,00 Hz, 80/80 mẫu. Tên 10 ký tự `FC_CTR_VER` đọc đúng cả qua `pymavlink`
  lẫn MAVROS.
- `STATUSTEXT`: không có — đúng, vì không có chuyển trạng thái nào trong lúc đo.

### 4.2 Quy tắc hiệu lực dữ liệu — phần Pi bắt buộc phải hiện thực

Đây là chỗ dễ làm hỏng EKF nhất. FC **không** đưa dữ liệu về 0 khi mất tin cậy, vì
*"vận tốc = 0 là một lời nói dối khác — nó bảo EKF máy bay đứng yên chắc chắn, trong khi
FC không biết"*. Thay vào đó FC **gắn cờ**, và Pi phải đọc cờ.

#### a) Cờ optical flow

Bit `MAV_SYS_STATUS_SENSOR_OPTICAL_FLOW` (`0x40`) trong
`SYS_STATUS.onboard_control_sensors_health` — **phía ROS trường này tên là `sensors_health`**
trong `mavros_msgs/SysStatus` trên `/mavros/sys_status`.

Bit này **nay phản ánh `est.position_valid`** (tức `ekf_velocity_is_valid()`), không còn
phản ánh "MTF01P còn nói chuyện" như bản firmware đầu. Bản vá đã kiểm chứng trên dây
2026-09-13 (`sensors_health = 0x1010F`, bit flow tắt đúng lúc).

**Bit tắt → bỏ hẳn mẫu, KHÔNG hạ trọng số.** Số đó không phải nhiễu, nó là một ước lượng
đã hỏng (tích phân gia tốc kế thuần tuý, trôi ~13 cm/s sau vài chục giây nằm im). Tăng
covariance chỉ làm EKF nuốt nó chậm hơn thay vì loại hẳn.

**Bỏ ĐÚNG trường — bỏ thừa là lỗi nặng:**

| Bản tin | Trường | Bit flow tắt thì |
|---|---|---|
| `LOCAL_POSITION_NED` | `vx`, `vy` | **BỎ** |
| `LOCAL_POSITION_NED` | `x`, `y` | **BỎ** (đứng yên ở 0, không trôi, nhưng vô nghĩa) |
| `LOCAL_POSITION_NED` | `z`, `vz` | **GIỮ** — từ EKF độ cao, không phụ thuộc flow |
| `GLOBAL_POSITION_INT` | `vx`, `vy` | **BỎ** |
| `GLOBAL_POSITION_INT` | `relative_alt`, `alt`, `vz` | **GIỮ** |
| `VFR_HUD` | `groundspeed` | bỏ (chỉ để hiển thị) |
| `VFR_HUD` | `alt`, `climb` | **GIỮ** |

> **Không bao giờ bỏ cả khung `GLOBAL_POSITION_INT`.** `optical_flow_node` lấy `relative_alt`
> để quy đổi pixel → mét; trên bàn flow gần như luôn tắt, nên bỏ cả khung sẽ khiến node
> **không bao giờ nhận được độ cao**.

#### b) Cờ trường của `HIGHRES_IMU`

`fields_updated` là bitmask nói trường nào có thật. **Pi phải đọc nó**, đừng giả định mọi
trường hợp lệ. Bit từ kế (6–8) theo thiết kế chỉ bật khi QMC5883P vừa `healthy` vừa
`calibrated`. **Bit này bật gần như liên tục là ĐÚNG** — từ kế đã hiệu chuẩn thật; lần tắt
thoáng qua là `healthy` chập chờn. Chi tiết mục 11.1 #3.

#### c) Bộ đếm lỗi cảm biến

`SYS_STATUS.errors_count1..4`, có sẵn trong `mavros_msgs/SysStatus`:

| Trường | Cảm biến | Tăng khi |
|---|---|---|
| `errors_count1` | IMU ICM20602 | mẫu toàn 0, tràn nhịp DRDY, lỗi SPI |
| `errors_count2` | máy thu CRSF (tay cầm) | khung ngắn, sai CRC, lỗi UART |
| `errors_count3` | baro BMP388 | lỗi I2C |
| `errors_count4` | MTF01P (flow + laser) | khung ngắn, sai CRC, lỗi UART |
| `errors_comm`, `drop_rate_comm` | — | luôn 0, chưa dùng |

Kiểu `uint16`, cộng dồn từ lúc khởi động, **không bao giờ tự xoá**, tràn vòng ở 65535.

> **Cảnh báo theo TỐC ĐỘ TĂNG, không theo giá trị.** Quy tắc: tăng quá N lần trong 10 s.
> Đo thực: CRSF 2 lỗi / 101 674 khung (0,002 %), MTF01P 1 / 441 482 (0,0002 %) — đó là
> nhiễu lúc cắm dây, không phải lỗi đang diễn ra.

### 4.3 Ghi chú từng bản tin

**`GLOBAL_POSITION_INT` — bo mạch KHÔNG có GPS.** Bản tin vẫn phát vì `optical_flow_node`
cần `relative_alt`. Quy ước "không biết" theo đúng đặc tả: `lat = lon = 0`, `hdg = 65535`.
Chỉ ba nhóm trường mang thông tin thật:

| Trường | Nguồn | Ghi chú |
|---|---|---|
| `relative_alt` | `est.altitude_m` × 1000 | EKF hợp nhất baro + laser MTF01P |
| `alt` | `baro.altitude_m` × 1000 | độ cao áp suất, **không phải MSL thật** |
| `vx`/`vy`/`vz` | `est.velocity_mps` × 100 | hệ NED, cm/s |

Phía Pi: `telemetry_aggregator_node` phải bỏ qua `NavSatFix` khi `status.status < 0`
(`NO_FIX`) để GCS không thấy toạ độ 0,0 ở độ cao ~37 m.

**`LOCAL_POSITION_NED` — độ tin cậy KHÔNG đồng đều giữa các trục.**

- `z`, `vz` — từ EKF độ cao, đáng tin.
- `x`, `y` — tích phân vận tốc optical flow, **chỉ** khi `ekf_velocity_is_valid()` đúng.
  Không khoá flow thì đứng yên ở 0 (đúng thiết kế, không phải lỗi). Dùng được để biết
  "đã rời chỗ cũ bao xa" trong vài chục giây gần đây. **Tuyệt đối không dùng làm gốc toạ
  độ cho bay theo lộ trình hay quay về điểm xuất phát.**
- `vx`, `vy` — theo quy tắc 4.2a.

**`HIGHRES_IMU` thay cho `SCALED_IMU`.** `SCALED_IMU` nhét gyro vào `int16` đơn vị mrad/s,
mà 2000 dps = 34900 mrad/s — **tràn kiểu ngay trong dải đo bình thường của ICM20602**.
`HIGHRES_IMU` dùng `float` nên không có bẫy đó và chở luôn từ kế lẫn khí áp trong cùng khung.

**`BATTERY_STATUS`.** Dự án đo điện áp tổng và dòng, **không đo từng cell**. FC tự nhận
dạng số cell lúc cắm pin; `voltages[]` điền `điện áp tổng / N` vào đúng N ô đầu, phần còn
lại `UINT16_MAX`. Chưa cắm pin thì `cell_count = 0` nên cả 10 ô đều 65535.

> **`battery_remaining = -1` là cố ý**, nghĩa "không biết": suy phần trăm từ điện áp lúc
> đang tải sai lệch lớn. **MAVROS quy đổi thành `percentage = -0.01`.** Phía Pi bắt buộc
> coi `percentage < 0` là "không biết" trước khi so ngưỡng — nếu không,
> `battery_pct = -1.0` sẽ kích `ESCALATE_EMERGENCY_LAND` ngay lập tức.

**`EXTENDED_SYS_STATE`.** Chưa arm → `ON_GROUND`. Đã arm → theo độ cao, ngưỡng **0,5 m**.
Không có `est.altitude_valid` → `UNDEFINED`.

**`GPS_RAW_INT` (24) — KHÔNG phát, có chủ ý.** Bo mạch không có GPS nên bản tin sẽ mang
`fix_type = 0` vĩnh viễn, và MAVROS cũng cho ra đúng "no fix" khi thiếu nó. Phát một bản
tin rỗng không đổi được gì. Bật lại khi lắp GPS thật.

### 4.4 Timestamp

| Bản tin | Trường | Nguồn |
|---|---|---|
| `ATTITUDE`, `LOCAL_POSITION_NED`, `GLOBAL_POSITION_INT` | `time_boot_ms` | `HAL_GetTick()` |
| `HIGHRES_IMU` | `time_usec` | **mili giây × 1000**, cố ý không dùng `micros()` |

`micros()` đọc TIM2 32 bit nên **tràn vòng sau 71 phút** — đủ để EKF trên Pi sắp sai thứ
tự phép đo.

**Đồng bộ thời gian: đã kiểm, KHÔNG phải vấn đề ở giai đoạn này.** Plugin `sys_time` đang
tắt nên không có `/mavros/time_reference` và không trao đổi `TIMESYNC`. Nhưng đo thực
`/mavros/imu/data`: nhịp 50.000 Hz, lệch chuẩn 0,43 ms, độ trễ header so với giờ Pi 2 ms
(min 1, max 3). Mọi mốc thời gian đều dùng giờ Pi lúc nhận gói nên **tự nhất quán**.

---

## 5. Pi → FC: giao diện điều khiển

### 5.1 Bảng nhận

| Message / lệnh | ID | FC làm gì | Trạng thái |
|---|---|---|---|
| `HEARTBEAT` | 0 | chỉ ghi lại thời điểm nhận. **FC KHÔNG làm gì khi mất heartbeat** — mục 11.1 #8 | [CHỐT] |
| `SET_POSITION_TARGET_LOCAL_NED` | 84 | setpoint vận tốc — mục 5.2 | [THOẢ THUẬN] — FC **đã hiện thực**, chưa đo đầu-cuối (11.1 #2). **Trước 09-13 FC loại mọi khung `0x07C7`** — mục 5.2 |
| `COMMAND_LONG` / `MAV_CMD_COMPONENT_ARM_DISARM` (400) | 76 | arm/disarm theo hợp đồng mục 6.2 | [CHỐT] |
| `COMMAND_LONG` / `MAV_CMD_REQUEST_AUTOPILOT_CAPABILITIES` (520) | 76 | trả `AUTOPILOT_VERSION` | [CHỐT] |
| `COMMAND_LONG` / `MAV_CMD_REQUEST_MESSAGE` (512) | 76 | trả lời khi `param1 = 148`; ID khác → `UNSUPPORTED` | [CHỐT] — Pi đo 09-13: `param1 = 148` → `ACCEPTED` + `AUTOPILOT_VERSION`; `param1 = 33` → `UNSUPPORTED`; qua `/mavros/cmd/command` (`1`/`191`) → `result = 0` |
| `SET_MODE` | 11 | **KHÔNG hiện thực** — đừng gửi | — |
| `LANDING_TARGET` | 149 | **chưa xử lý** | [ĐANG LÀM] |
| `COMMAND_LONG` / `MAV_CMD_NAV_TAKEOFF` (22) | 76 | ACK `UNSUPPORTED` | — |

> **`SET_MODE` không dùng được và cũng KHÔNG báo lỗi rõ.** Nó không dùng `COMMAND_ACK`;
> MAVROS xác nhận bằng cách chờ `custom_mode` trong `HEARTBEAT` đổi — mà FC không đổi.
> Gọi `/mavros/set_mode` sẽ treo tới timeout rồi báo thất bại. **Đừng gọi.**

### 5.2 `SET_POSITION_TARGET_LOCAL_NED` — đặc tả chốt

**FC chỉ đọc `vx`, `vy`, `vz`, `yaw_rate`. Bỏ qua vị trí, gia tốc, và `yaw` tuyệt đối.**

| Trường | FC dùng | Ánh xạ |
|---|---|---|
| `vx`, `vy` | có | mục tiêu vận tốc ngang → `ctrl_poshold` |
| `vz` | có | mục tiêu tốc độ lên/xuống → `ctrl_althold` (`vz = 0` tự chốt độ cao) |
| `yaw_rate` | có | tốc độ quay (rad/s) → `ctrl_rate` |
| `x`, `y`, `z` | **bỏ** | lý do ở mục 1 |
| `afx`, `afy`, `afz` | **bỏ** | không có vòng gia tốc |
| `yaw` | **bỏ** | FC **không có vòng góc yaw**; ra lệnh `yaw` tuyệt đối → **loại cả khung** |

**`coordinate_frame` = `MAV_FRAME_BODY_NED` (8).** `ctrl_poshold` vốn xoay vận tốc về hệ
thân để tính, và sai số AprilTag cũng sinh ra ở hệ camera — dùng hệ thân bỏ được một phép
xoay ở mỗi đầu. Chuyển sang `MAV_FRAME_LOCAL_NED` (1) chỉ sau khi từ kế QMC5883P đã hiệu
chuẩn và yaw chứng minh đáng tin (mục 11, mục 10.3).

**`type_mask` = `0b0000011111000111` = `0x07C7`.**

| Bit | Trường | Giá trị | Nghĩa |
|---|---|---|---|
| 0–2 | `x`, `y`, `z` | `111` | bỏ qua |
| 3–5 | `vx`, `vy`, `vz` | `000` | **DÙNG** |
| 6–8 | `afx`, `afy`, `afz` | `111` | bỏ qua |
| 9 | `force` | `1` | cờ "af mang lực thay vì gia tốc" — vô hại vì af đã bị bỏ. **FC không xét bit này** |
| 10 | `yaw` | `1` | **bỏ qua** — đặt `0` sẽ bị FC **loại cả khung** |
| 11 | `yaw_rate` | `0` | **DÙNG** |

Bit 11 được phép bật (= không ra lệnh yaw) — đó là trường hợp hợp lệ, không phải lỗi.
Mọi sai khác khác → khung bị loại, `loai` tăng, và sau `offboard_timeout_ms` FC hết hạn
setpoint rồi lùi về POSHOLD.

> **Lỗi FC đã sửa 2026-09-13.** Trước ngày này firmware loại khung khi bit 9 bật — tức loại
> **100 %** setpoint dùng đúng `0x07C7`. Kiểm dấu mục 3.4 chạy trước ngày đó sẽ chỉ thấy
> `loai` tăng, `nhan` = 0. **Chạy lại.** Đã kiểm logic mới với 7 mask: `0x07C7`, `0x05C7`,
> `0x0FC7` được nhận; ra lệnh `yaw` tuyệt đối, ra lệnh vị trí, không bỏ gia tốc, không có
> vận tốc — đều bị loại.

> FC **bắt buộc** tôn trọng `type_mask`. Đọc cả ô Pi cố tình để trống là lỗi kinh điển —
> máy bay sẽ lái theo số rác.

**Tần số:** Pi gửi **10–20 Hz**. Vòng vận tốc đóng trên FC ở tốc độ vòng lặp chính nên
không cần Pi chạy nhanh hơn.

**Hết hạn setpoint — phần quan trọng nhất.** Quá **500 ms** (`offboard_timeout_ms`) không
nhận được khung hợp lệ, FC rời OFFBOARD về **POSHOLD**: mục tiêu vận tốc lấy từ cần, cần ở
giữa = 0 → **phanh rồi treo**. Đây chính là thứ làm lệnh vận tốc an toàn hơn lệnh góc.

Đường lùi này **đảm bảo được** vì từ hợp đồng 1.1 FC **bắt buộc ch6 ở POSHOLD** mới cho vào
OFFBOARD (mục 6.3 điều kiện 7). Trước đó FC lùi về chế độ ch6 đang giữ — ch6 ở ANGLE thì
**không có phanh**, máy bay chỉ giữ thăng bằng rồi trôi theo quán tính. Mất optical flow thì
POSHOLD vẫn tụt tiếp về ANGLE: không còn gì đo vận tốc thì không phanh được.

### 5.3 MỘT publisher duy nhất — quy tắc kiến trúc bắt buộc

**Đúng một node phía Pi được publish `/mavros/setpoint_raw/local`.** Node đó là
**`position_controller_node`**.

Lý do không phải thẩm mỹ: hai node cùng publish, mỗi bên một timer 20 Hz, thì **timeout
500 ms ở mục 5.2 không bảo vệ được gì** — luôn có gói tới nên FC không bao giờ hết hạn,
nó chỉ nhận setpoint mâu thuẫn xen kẽ nhau. Đây là kiểu hỏng im lặng và khó chẩn đoán nhất.

Các node khác muốn tác động thì đi qua `/mission/setpoint`, không publish thẳng.
`fc_command_bridge_node` chỉ còn là cổng lệnh arm / mode, **không** publish setpoint.
*(Đã sửa 2026-09-12.)*

---

## 6. Máy trạng thái: chế độ bay, ARM, OFFBOARD

### 6.1 `custom_mode` và `base_mode`

Firmware khai `autopilot = MAV_AUTOPILOT_GENERIC`, nên MAVROS **không biết** tên chế độ và
`/mavros/state` trả `mode: "CMODE(n)"` thay vì tên. `custom_mode` chở thẳng `flight_mode_t`
của firmware.

| `custom_mode` | Tên firmware | Ý nghĩa |
|---|---|---|
| 0 | `FLIGHT_MODE_ACRO` | điều khiển tốc độ góc trực tiếp |
| 1 | `FLIGHT_MODE_ANGLE` | tự cân bằng theo góc nghiêng |
| 2 | `FLIGHT_MODE_ALTHOLD` | giữ độ cao (baro + laser) |
| 3 | `FLIGHT_MODE_POSHOLD` | giữ **vận tốc** bằng optical flow |
| 4 | `FLIGHT_MODE_OFFBOARD` | nhận lệnh vận tốc từ Pi — **đã phát thật** |

Không có LAND và RTL, và **đừng thêm** cho tới khi có GPS hoặc VIO: cả hai cần một nguồn
vị trí tuyệt đối mà bo mạch không có.

> **`custom_mode` trả lời câu "setpoint của Pi có đang được dùng không", KHÔNG trả lời câu
> "Pi còn quyền không".** Mất optical flow trong lúc OFFBOARD → FC tụt về ANGLE và
> `custom_mode` thành `1`, dù máy trạng thái OFFBOARD bên trong chưa thoát. Dùng `OB_AUTH`
> (mục 6.3) cho câu hỏi về quyền.

**`base_mode`.** Luôn bật `MAV_MODE_FLAG_CUSTOM_MODE_ENABLED` (1) và `MANUAL_INPUT_ENABLED`
(64); bật thêm `STABILIZE_ENABLED` (16) ở mọi chế độ trừ ACRO. Bit `SAFETY_ARMED` (128)
bám đúng `motor.armed` — **đây là nguồn duy nhất** cho trường `armed` của `/mavros/state`.
Giá trị 81 (`64|16|1`) là trạng thái chưa arm ở một chế độ có cân bằng.

Hệ quả cho Pi: nếu có ngày `SET_MODE` được hiện thực, phải truyền `custom_mode` dạng **số**
và `base_mode = 0`. Truyền chuỗi `"GUIDED"` / `"OFFBOARD"` không có tác dụng.

### 6.2 Hợp đồng ARM [CHỐT — thay thế hoàn toàn bản đợt 1]

> Bản tài liệu đợt 1 ghi *"ARM từ Pi luôn bị từ chối"*. **Điều đó không còn đúng.**

**Nguyên tắc không đổi: quyền phủ quyết luôn nằm ở người đứng cạnh máy bay, bằng phần
cứng.** Cái đã đổi là cách trao quyền, không phải nguyên tắc.

Công tắc **ch8** (tham số `offboard_switch_channel = 7`, đếm từ 0) nay **mặc định BẬT**.
Khi ch8 lên, công tắc ARM **ch5** đổi nghĩa: không còn là "arm ngay" mà thành
**"cho phép Pi arm"**.

**Trình tự chuẩn:**

| # | Ai | Việc |
|---|---|---|
| 1 | Người lái | cần ga về **giữa**; công tắc chế độ **ch6 ở POSHOLD** |
| 2 | Người lái | ch8 **lên** |
| 3 | Người lái | ch5 **lên** — FC **không tự arm**, đứng chờ lệnh từ Pi |
| 4 | Pi | phát `SET_POSITION_TARGET_LOCAL_NED` 10–20 Hz (**trước**, vận tốc 0 làm nhịp giữ chỗ) |
| 5 | Pi | gửi `MAV_CMD_COMPONENT_ARM_DISARM`, `param1 = 1` |
| 6 | FC | arm, và OFFBOARD **tự vào** khi đủ điều kiện mục 6.3 |

Sau đó Pi **tự do arm và disarm** không cần thêm thao tác nào từ tay cầm.

> **Vì sao ch6 = POSHOLD áp cho cả lệnh ARM, không chỉ lúc vào OFFBOARD.** Chế độ Pi đòi ga
> ở giữa. Arm xong, trước khi OFFBOARD kịp vào, máy bay chạy theo chế độ ch6. Ở ANGLE ga lấy
> thẳng từ cần: cần ở giữa ≈ **50 % lực đẩy ngay khoảnh khắc arm** — khi có điện động cơ
> là một cú bật nhảy. POSHOLD giao ga cho vòng giữ độ cao, cần ở giữa = giữ nguyên.
> Đã kiểm trên bàn 09-13: ch8 + ch5 lên, ch6 ở ANGLE → console báo cờ chặn `0x0800`.

**Mã trả về `COMMAND_ACK` — đọc trường `result`, không chỉ `success`:**

| `result` | Nghĩa | Pi phải làm gì |
|---|---|---|
| `ACCEPTED` (0) | đã làm, hoặc vốn đã ở trạng thái đó | tiếp tục |
| `TEMPORARILY_REJECTED` (1) | có quyền nhưng chưa đủ điều kiện (ga chưa giữa, **ch6 chưa ở POSHOLD**, cảm biến…) | **thử lại sau** là hợp lý |
| `DENIED` (2) | **Pi không có quyền**: ch8/ch5 chưa đúng, hoặc người lái đã giành lái | **ĐỪNG thử lại** — chờ người lái. **Xem cảnh báo ngay dưới** |

Phân biệt `DENIED` với `TEMPORARILY_REJECTED` là chủ ý: **một cái đòi người, cái kia đòi
thời gian.**

> **Lỗ hổng đã biết (FC phát hiện 09-13), chưa sửa:** `OB_AUTH = 1` **không** có nghĩa "gửi ARM
> là được". `OB_AUTH` chỉ đòi ch8 bật và không `KHOA`; còn ARM đòi thêm **ch5 đã lên sau ch8**.
> Nên khi ch8 lên mà ch5 còn xuống, Pi thấy `OB_AUTH = 1`, gửi ARM và nhận `DENIED`. Trên dây
> **không có gì báo lúc người lái gạt ch5 lên** — Pi chỉ còn cách thử lại mù, trái với dòng
> "ĐỪNG thử lại". Đề xuất sửa: kênh `OB_ARM_RDY` ở mục 6.3.

**Pi có quyền khi và chỉ khi:** ch8 bật, sóng RC đọc được, và OFFBOARD **không** ở `KHOA`.
Nên Pi mất quyền ở **mọi** trường hợp đưa về `KHOA` (bảng `OB_EXIT` mục 6.3), trong đó có:

- người lái **chạm bất kỳ cần nào** sau khi đã arm (roll, pitch, yaw, hoặc ga lệch khỏi giữa);
- mất sóng RC;
- **hết hạn setpoint** — tức Pi crash rồi restart;
- kẹp dải liên tục, mất góc, người lái gạt ch6 rời POSHOLD;
- và ch8 đang xuống.

Ngay khi mất quyền, **Pi không ra lệnh gì được nữa — kể cả DISARM.** Hệ quả cần biết: node
crash → hết hạn → `KHOA` → node restart xong **không disarm được**. Người lái vẫn luôn cắt
được bằng ch5. Ngoại lệ duy nhất: lệnh DISARM khi máy bay **vốn đang không arm** luôn trả
`ACCEPTED` (trả lời trung thực trạng thái, không làm gì).

**Lấy lại quyền CHỈ bằng cách người lái gạt ch8 xuống rồi lên lại.** Node ROS 2 crash rồi
tự restart **không** khôi phục quyền.

### 6.3 Vòng đời OFFBOARD

**Vào OFFBOARD: chỉ bằng công tắc RC + setpoint. Không có `SET_MODE`.** Bảy điều kiện phải
đúng **đồng thời**:

1. công tắc `offboard_switch_channel` đang bật, và sóng RC còn đọc được
2. đã arm (`motor.armed` và `mode == FC_MODE_ARMED`)
3. `est.attitude_valid`
4. **đã nhận được ít nhất một setpoint hợp lệ**
5. setpoint đó còn trong hạn `offboard_timeout_ms` (500 ms)
6. người lái không đang đẩy cần
7. **công tắc chế độ ch6 ở POSHOLD** *(thêm ở hợp đồng 1.1 — đường lùi phải phanh được, mục 5.2)*

> Điều kiện 4–5 là thứ định hình `mission_manager_node`: **Pi phải đang phát setpoint
> TRƯỚC khi người lái gạt công tắc.** Gạt công tắc lúc Pi còn im lặng thì không có gì xảy
> ra và **FC không báo lỗi** — có chủ ý, vì vào chế độ rồi hết hạn ngay sau đó là kiểu
> hỏng khó chẩn đoán hơn nhiều.

**OFFBOARD KHÔNG tự vào lại sau hỏng hóc.** Rơi ra vì hết hạn, mất sóng, đẩy cần, kẹp dải,
mất góc, hay ch6 rời POSHOLD thì trạng thái thành `KHOA`, và **chỉ** thoát bằng cách người
lái gạt ch8 xuống rồi lên lại. Node ROS 2 restart **không** làm máy bay chạy lại.

Hai lối ra **không** khoá, về `TAT`: người lái gạt ch8 xuống (`CONG_TAC`), và disarm
(`DISARM` — để Pi arm lại được).

Mặc định `offboard_switch_channel = 7` (= ch8, tham số đếm từ 0), đã đổi mặc định và `save`
trên máy bay ngày 09-13. Tắt hẳn đường ra lệnh từ Pi: `set offboard_switch_channel=-1` rồi
`save`.

#### Kênh báo trạng thái — `NAMED_VALUE_INT` (252) @ 2 Hz [CHỐT — Pi đo trên dây và qua MAVROS 09-13, mục 4.1]

Đây là **kênh máy đọc**, và là thứ `mission_manager_node` bị chặn bởi.

| `name` | Giá trị |
|---|---|
| `OB_STATE` | 0 = KHOA, 1 = TAT, 2 = DANG_CHAY |
| `OB_AUTH` | 1 = Pi còn quyền, 0 = đã mất |
| `OB_EXIT` | lý do rời gần nhất (bảng dưới) |

Phát **định kỳ** chứ không theo sự kiện, để node restart lúc nào cũng biết trạng thái trong
vòng 0,5 s. Phát **cả khi chưa arm, ch8 xuống, hay `offboard_switch_channel = -1`**, để Pi
phân biệt "FC chưa phát" với "FC báo không có quyền". Cùng nhóm phát còn có `FC_CTR_VER`
(mục 10.1).

Đo trên target 09-13 (giải mã bộ đệm TX UART8 qua SWD, kiểm CRC): `OB_STATE=0`,
`OB_AUTH=0`, `OB_EXIT=1`, `FC_CTR_VER=10100` — đúng trạng thái khoá lúc khởi động. Phía Pi: plugin `debug_value` **đã bật và kiểm chứng**, topic
`/mavros/debug_value/named_value_int` (`mavros_msgs/DebugValue`: `name`, `value_int`,
`type = 4`).

| `OB_EXIT` | Tên | Nghĩa | Pi tự phục hồi được? |
|---|---|---|---|
| 0 | — | chưa từng rời | — |
| 1 | `KHOI_DONG` | khoá sẵn từ lúc bật nguồn | không — chờ người lái gạt ch8 |
| 2 | `CONG_TAC` | người lái gạt ch8 xuống | không |
| 3 | `HET_HAN` | quá 500 ms không có setpoint hợp lệ | **không** — dù Pi đã sống lại |
| 4 | `DAY_CAN` | người lái chạm cần | không |
| 5 | `DISARM` | disarm khi đang OFFBOARD | **có** — Pi arm lại được |
| 6 | `KEP_DAI` | lệnh vượt giới hạn bao liên tục | không |
| 7 | `MAT_GOC` | bộ ước lượng mất góc | không |
| 8 | `MAT_SONG` | mất sóng RC | không |
| 9 | `CHE_DO` | người lái gạt ch6 rời POSHOLD khi đang OFFBOARD *(thêm ở 1.1)* | không |

#### Kênh người đọc — `STATUSTEXT` (253), bắn khi chuyển trạng thái

```
OFFBOARD: DANG_CHAY
OFFBOARD: KHOA (DAY_CAN)
OFFBOARD: TAT (CONG_TAC)
```

Có sẵn ở `/mavros/statustext/recv` (plugin `sys_status`), không cần bật thêm gì.
`severity`: `NOTICE` cho `DANG_CHAY` và `TAT`, `WARNING` cho mọi lần vào `KHOA` [THOẢ THUẬN —
FC xác nhận và đã hiện thực 09-13, chờ Pi đo]. Không bắn lúc khởi động (không có chuyển
trạng thái nào).

> **STATUSTEXT một mình KHÔNG đủ.** Nó là sự kiện, bắn một lần lúc chuyển trạng thái.
> Kịch bản phải xử lý chính là **node crash rồi restart** — node restart đã lỡ mất sự kiện
> và sẽ tưởng mình còn quyền. Vì thế mới cần kênh định kỳ.

#### Bảng suy diễn trạng thái cho `mission_manager_node`

| `OB_STATE` | `OB_AUTH` | `custom_mode` | Nghĩa | Pi làm gì |
|---|---|---|---|---|
| 2 | 1 | 4 | OFFBOARD chạy, setpoint đang được dùng | bay bình thường |
| 2 | 1 | 1 | OFFBOARD còn chạy nhưng FC đã tụt về ANGLE vì **mất flow** — **`vx`/`vy` không được dùng**, nhưng **`vz` và `yaw_rate` VẪN được dùng** | giữ nhịp phát (ngừng là hết hạn → `KHOA`); nhớ `vz`/`yaw_rate` vẫn có hiệu lực; báo cảnh báo. `OB_EXIT` **không** đổi. |
| 1 | 1 | bất kỳ, **chưa arm** | TAT — có quyền, chưa arm | phát setpoint 0; **gửi ARM khi người lái đã gạt ch5**. Hiện Pi **không biết** lúc đó (xem cảnh báo 6.2) — tạm thời thử ARM thưa (ví dụ 1 lần/2 s), `DENIED` thì chờ; khi có `OB_ARM_RDY` thì chỉ gửi lúc `OB_ARM_RDY = 1` |
| 1 | 1 | bất kỳ, **đã arm** | TAT — đã arm nhưng chưa vào OFFBOARD | phát setpoint; FC tự vào khi đủ 7 điều kiện. Không vào được thì thường do setpoint quá hạn, cần chưa ở giữa, hoặc ch6 chưa ở POSHOLD |
| 0 | bất kỳ | bất kỳ | KHOA | **không tự phục hồi**; báo người lái gạt ch8 xuống-lên |
| bất kỳ | 0 | bất kỳ | mất quyền | **không gửi lệnh nào, kể cả DISARM** |
| *không có gói* | — | — | FC chưa phát `NAMED_VALUE_INT` | coi như không biết; **không** suy quyền từ `RC_CHANNELS` hay `custom_mode` |

> *Sửa 09-13 theo câu hỏi 11.1 #7 của Pi:* dòng này bản cũ ghi "chờ người lái gạt công tắc".
> Sai — `OB_AUTH = 1` nghĩa là ch8 **đã** lên; theo 6.2 thì lúc đó người lái chờ ch5, còn Pi
> là bên gửi ARM.

**[THOẢ THUẬN — Pi đồng ý 09-13, mời FC hiện thực] Kênh sẵn sàng arm — thêm vào nhóm `NAMED_VALUE_INT` 2 Hz:**

| `name` | Giá trị |
|---|---|
| `OB_ARM_RDY` | 1 = FC **đang chờ Pi arm và mọi điều kiện đã thoả** — gửi ARM lúc này sẽ `ACCEPTED`; 0 = chưa |
| `OB_ARM_BLK` | bitmask lý do chưa sẵn sàng (`arm_block_flags` của FC) — để hiển thị, bảng bit đăng ký ở 9.3 khi chốt |

Đúng quy tắc R5: trạng thái hỏi lại được, node restart không lỡ gì. Pi không phải thử lại mù.

> *Phản hồi Pi 09-13:* đồng ý, và coi đây là **ưu tiên cao nhất phía FC** — `mission_manager_node`
> viết đường ARM dựa vào `OB_ARM_RDY`, không viết vòng thử lại mù. Hai đề nghị kèm theo:
> (1) `OB_ARM_RDY = 1` **chỉ** khi gửi ARM ngay lúc đó sẽ `ACCEPTED` — nếu còn trường hợp FC
> vẫn trả `TEMPORARILY_REJECTED` khi `OB_ARM_RDY = 1`, xin ghi rõ ở đây;
> (2) **đăng ký bảng bit `OB_ARM_BLK` vào 9.3 cùng lúc phát** (có cờ `0x0800` = ch6 chưa ở POSHOLD
> đã thấy ở 6.2), để GCS hiển thị được lý do. Pi coi bit lạ là "lý do chưa rõ", không crash (R1).
> Dòng cuối quan trọng: **đừng suy quyền từ vị trí công tắc.** ch8 đang lên nhưng người lái
> vừa chạm cần → KHOA, Pi mất quyền, trong khi `RC_CHANNELS` vẫn báo ch8 lên. Suy quyền từ
> công tắc là tự dựng lại máy trạng thái của FC ở phía Pi, và sẽ lệch.

---

## 7. `COMMAND_ACK` — hợp đồng bắt buộc

**Mọi `COMMAND_LONG` gửi tới đúng địa chỉ đều được trả `COMMAND_ACK`, kể cả lệnh không hỗ
trợ** (`MAV_RESULT_UNSUPPORTED`). Đường duy nhất thoát ra không ACK là khi `target_system`
không phải của FC — đúng đặc tả.

Đây không phải lịch sự mà là bắt buộc: MAVROS **chặn** service `/mavros/cmd/*` cho tới khi
có ACK. Thiếu ACK thì mọi lệnh treo tới timeout và **nhìn từ ROS sẽ giống "lệnh thất bại"
dù FC đã làm xong**.

`COMMAND_ACK` chứa đúng `command` vừa nhận (không để 0); `target_system`/`target_component`
lấy thẳng từ `sysid`/`compid` của khung gửi tới — **`1`/`191` với MAVROS** (mục 2), `255`/`190` với
script `pymavlink`. Lần nghiệm thu 09-12 bên dưới đi bằng `255`/`190`.

**Đã nghiệm thu trên bàn** (2026-09-12), cả hai dưới 0,01 s, `armed` giữ nguyên `False`:

| Gửi | Nhận | Kiểm điều gì |
|---|---|---|
| `MAV_CMD_REQUEST_AUTOPILOT_CAPABILITIES` (520) | `ACCEPTED` + `AUTOPILOT_VERSION` | đường ACK chiều thành công |
| `MAV_CMD_NAV_TAKEOFF` (22) | `UNSUPPORTED` | ACK cho lệnh không hỗ trợ |
| `MAV_CMD_COMPONENT_ARM_DISARM` (400), `param1 = 0` | `ACCEPTED` | disarm — vô hại khi đang không arm |

---

## 8. Ánh xạ ba tầng: MAVLink ↔ MAVROS ↔ node ROS

Bảng này là giao diện mà node phía Pi được phép dựa vào. Cột cuối là node chịu trách nhiệm.

### 8.1 Chiều FC → Pi

| Bản tin | Topic MAVROS | Kiểu | Node tiêu thụ |
|---|---|---|---|
| `HEARTBEAT` | `/mavros/state` | `mavros_msgs/State` | `mission_manager`, `failsafe_monitor`, `telemetry_aggregator` |
| `SYS_STATUS` | `/mavros/sys_status` | `mavros_msgs/SysStatus` | bit flow: node nào dùng vận tốc FC — **hiện chưa có** (11.3 P3); `failsafe_monitor` (`errors_count*`) |
| `ATTITUDE` | `/mavros/imu/data` | `sensor_msgs/Imu` | `ekf_filter_node` (`robot_localization`, imu0) |
| `HIGHRES_IMU` | `/mavros/imu/data_raw`, `/mavros/imu/mag` | `sensor_msgs/Imu`, `MagneticField` | chẩn đoán |
| `LOCAL_POSITION_NED` | `/mavros/local_position/pose`, `.../velocity_local` | `PoseStamped`, `TwistStamped` | *(chưa nối vào EKF)* |
| `GLOBAL_POSITION_INT` | `/mavros/global_position/rel_alt` | `std_msgs/Float64` | **`optical_flow_node`** — quy đổi pixel→mét |
| `GLOBAL_POSITION_INT` | `/mavros/global_position/global` | `sensor_msgs/NavSatFix` | `telemetry_aggregator` (bỏ khi `NO_FIX`) |
| `BATTERY_STATUS`, `SYS_STATUS` | `/mavros/battery` | `sensor_msgs/BatteryState` | `failsafe_monitor` (**xử lý `percentage < 0`**) |
| `EXTENDED_SYS_STATE` | `/mavros/extended_state` | `mavros_msgs/ExtendedState` | `mission_manager` |
| `VFR_HUD` | `/mavros/vfr_hud` | `mavros_msgs/VfrHud` | hiển thị |
| `NAMED_VALUE_INT` | `/mavros/debug_value/named_value_int` | `mavros_msgs/DebugValue` | **`mission_manager`** — `OB_STATE`/`OB_AUTH`/`OB_EXIT` |
| `STATUSTEXT` | `/mavros/statustext/recv` | `mavros_msgs/StatusText` | `mission_logger` |
| `DISTANCE_SENSOR` | `/mavros/mtf01p` | `sensor_msgs/Range` | *(chưa nối — hạ cánh chính xác)*. `variance` luôn 0, dùng σ ở 9.6 |

### 8.2 Chiều Pi → FC

| Topic MAVROS | Kiểu | Sinh bản tin | Node publisher |
|---|---|---|---|
| `/mavros/setpoint_raw/local` | `mavros_msgs/PositionTarget` | `SET_POSITION_TARGET_LOCAL_NED` | **`position_controller_node` — DUY NHẤT** |
| `/mavros/cmd/arming` (service) | `mavros_msgs/srv/CommandBool` | `COMMAND_LONG` 400 | `fc_command_bridge_node` |
| `/mavros/landing_target/pose` | `geometry_msgs/PoseStamped` | `LANDING_TARGET` | `landing_target_bridge_node` |

> **Cạm bẫy topic hạ cánh chính xác.** `/mavros/landing_target/raw` **KHÔNG tồn tại** —
> đã xác minh trực tiếp với `mavros_node`. Plugin `landing_target` thật sự cung cấp:
>
> | Topic | Kiểu | Chiều |
> |---|---|---|
> | `/mavros/landing_target/pose` | `geometry_msgs/PoseStamped` | Pi → FC |
> | `/mavros/landing_target/pose_in` | `geometry_msgs/PoseStamped` | FC → Pi |
> | `/mavros/landing_target/lt_marker` | `geometry_msgs/Vector3Stamped` | FC → Pi |
>
> *(`landing_target_bridge_node` đã sửa 2026-09-12. `position_controller_node` vẫn còn
> subscribe topic mồ côi đó — xem mục 11.)*

### 8.3 `plugin_allowlist` — cấu hình MAVROS

Đang bật: `sys_status`, `setpoint_position`, `setpoint_raw`, `command`, `imu`,
`local_position`, `global_position`, `landing_target`, `vfr_hud`, `debug_value`,
`distance_sensor` *(bật 09-13)*.

Ba điểm dễ sai đã gặp:

- **`'battery'` KHÔNG phải tên plugin hợp lệ.** `/mavros/battery` do `sys_status` sinh ra.
  Để nó trong allowlist khiến allowlist có 9 mục nhưng chỉ 8 plugin nạp — im lặng.
- **Launch MAVROS: đặt `namespace='mavros'`, KHÔNG đặt `name=`.** `name=` trong launch áp
  remap `__node` cho **mọi** node con trong process `mavros_node` (router, từng plugin) →
  trùng tên, plugin đè topic nhau và **crash**: `create_subscription() called for existing
  topic name rt/mavros/mavros/local with incompatible type`. Thêm namespace mà vẫn giữ
  `name=` thì **vẫn crash**. *(Đã sửa `estimation.launch.py` 2026-09-13.)*
- **`distance_sensor` cần khối `config` riêng, và topic KHÔNG nằm dưới `distance_sensor/`.**
  Không khai `id` thì plugin không phát gì (`DS: no mapping for sensor id`). Topic lấy tên
  từ khoá cấu hình, đặt thẳng dưới namespace: khoá `mtf01p` → **`/mavros/mtf01p`**. Cấu hình
  hiện tại: `id: 0`, `orientation: PITCH_270`, `send_tf: false` (chưa đo vị trí lắp laser).

Plugin đang tắt, bật khi FC phát bản tin tương ứng: `odometry`,
`px4flow`, `rc_io`, `vibration`, `altitude`, `sys_time`.

### 8.4 Tham số covariance IMU — đã đặt, có lý do

`mavros.yaml` node `/mavros/imu`:

| Tham số | MAVROS mặc định | Đã đặt | Vì sao |
|---|---|---|---|
| `orientation_stdev` | 1.0 (57,3°) | **0.01745** (1°) | mặc định bi quan ~1800× → EKF gần như **bỏ qua** góc từ FC, thứ đáng tin nhất |
| `linear_acceleration_stdev` | 0.0003 | **0.0185** | mặc định lạc quan 62× → EKF tin gần như tuyệt đối vào gia tốc thô, thứ nhiễu nhất |
| `angular_velocity_stdev` | 0.000349 | **0.00183** | lạc quan 5,2× |

> **Đừng dùng con số bộ lọc tự chấm điểm chính nó làm covariance tuyệt đối.** FC có phát
> `ekf_attitude_uncertainty_deg()` = **0,071°** trên bàn, và Pi đo nhiễu ngắn hạn 0,032° —
> **cả hai đều là bẫy**. Chúng không mô hình hoá lệch lắp đặt IMU, trôi bias gia tốc kế,
> hay sai số do rung khi có điện động cơ. Giá trị hợp lý tính cả trôi bias là **0,5–2°**.
>
> Tương tự, `0.0185 m/s²` là **sàn khi đứng im**; có rung động cơ sẽ cao hơn nhiều —
> **phải đo lại ở giai đoạn có điện động cơ**.

**Yaw: không dùng làm hướng tuyệt đối ở bất cứ giá trị nào** cho tới khi hướng từ kế được
**kiểm chứng**. Từ kế đã hiệu chuẩn (09-04) nhưng chưa ai đo độ đúng hướng, và chưa đo khi có
điện động cơ (mục 10.6a). Trôi yaw 0,00°/phút đo được là **lúc nằm im, nhiệt độ ổn định** — không suy ra được
cho lúc bay.

---

## 9. Sổ đăng ký — nền tảng để mở rộng

Mọi thứ hai bên cùng dùng đều được cấp số ở đây. **Thêm mục mới = sửa bảng này trước, viết
code sau.** Đây là chỗ duy nhất cấp phát, để hai bên không đụng số của nhau.

### 9.1 Bản tin MAVLink

| Dải | Dùng cho | Quy tắc |
|---|---|---|
| ID chuẩn `common.xml` | mọi thứ hiện có | **Ưu tiên tuyệt đối.** Có bản tin chuẩn thì dùng, đừng tự chế |
| ID `180–229` | dialect riêng của dự án, **nếu buộc phải có** | Phải ghi vào bảng 9.2 trước khi dùng |
| `DEBUG_VECT` / `NAMED_VALUE_*` | tên **chưa** đăng ký ở 9.3: chẩn đoán tạm, thử nghiệm. Tên **đã** đăng ký ở 9.3 (`OB_*`, `FC_CTR_VER`): **kênh báo trạng thái chính thức** | **Chỉ chiều FC → Pi, chỉ để BÁO trạng thái.** Không bao giờ dùng để RA LỆNH cho FC. Xem ghi chú dưới |

Bản tin chuẩn đã cấp:

| ID | Bản tin | Chiều | Trạng thái |
|---|---|---|---|
| 0 | `HEARTBEAT` | ↔ | [CHỐT] |
| 1 | `SYS_STATUS` | FC→Pi | [CHỐT] |
| 11 | `SET_MODE` | Pi→FC | **không hiện thực, không dùng** |
| 30 | `ATTITUDE` | FC→Pi | [CHỐT] |
| 32 | `LOCAL_POSITION_NED` | FC→Pi | [CHỐT] |
| 33 | `GLOBAL_POSITION_INT` | FC→Pi | [CHỐT] |
| 65 | `RC_CHANNELS` | FC→Pi | [THOẢ THUẬN] — Pi chốt cách điền 09-13 (11.2), mời FC viết code |
| 74 | `VFR_HUD` | FC→Pi | [CHỐT] |
| 76 | `COMMAND_LONG` | Pi→FC | [CHỐT] |
| 77 | `COMMAND_ACK` | FC→Pi | [CHỐT] |
| 84 | `SET_POSITION_TARGET_LOCAL_NED` | Pi→FC | [THOẢ THUẬN] — chờ kiểm dấu |
| 251 | `NAMED_VALUE_FLOAT` | FC→Pi | [THOẢ THUẬN — Pi đồng ý 09-13] — `OB_T_*`, mục tiêu đã nhận để Pi tự kiểm dấu 3.4. *(Bỏ đề xuất `POSITION_TARGET_LOCAL_NED` (85): phép đổi hệ hai chiều của MAVROS triệt tiêu nên không kiểm được dấu — mục 3.4)* |
| 105 | `HIGHRES_IMU` | FC→Pi | [CHỐT] |
| 106 | `OPTICAL_FLOW_RAD` | FC→Pi | [ĐỀ XUẤT] — cần đo hệ số quy đổi radian trước |
| 132 | `DISTANCE_SENSOR` | FC→Pi | [CHỐT] — Pi đã bật plugin và đo 09-13 (mục 4.1). `min_distance` vẫn là số tạm |
| 147 | `BATTERY_STATUS` | FC→Pi | [CHỐT] |
| 148 | `AUTOPILOT_VERSION` | FC→Pi | [CHỐT] — Pi đo 09-13: `flight_sw_version = 0x00010000`, hash có. **Thứ tự byte: FC chốt `uint64` little-endian từ hợp đồng 1.2** (9.5, 11.1 #9); cờ capability mục 9.5 |
| 149 | `LANDING_TARGET` | Pi→FC | [ĐANG LÀM] — FC chưa xử lý |
| 241 | `VIBRATION` | FC→Pi | [ĐỀ XUẤT] — giai đoạn có điện động cơ |
| 245 | `EXTENDED_SYS_STATE` | FC→Pi | [CHỐT] |
| 252 | `NAMED_VALUE_INT` | FC→Pi | [CHỐT] cho `OB_STATE`/`OB_AUTH`/`OB_EXIT`/`FC_CTR_VER` — Pi đo 09-13 (mục 4.1). Hành vi chuyển trạng thái còn ở 12.A4 |
| 253 | `STATUSTEXT` | FC→Pi | [THOẢ THUẬN] — FC đã hiện thực 09-13, chờ Pi đo |
| 331 | `ODOMETRY` | FC→Pi | [THOẢ THUẬN] — Pi chốt cách điền 09-13 sau khi kiểm MAVROS (11.2), mời FC viết code |

> *Sửa 09-13 theo mâu thuẫn (c) Pi nêu ở 11.1 #7.* Bản cũ ghi `NAMED_VALUE_*` "không dùng cho
> đường điều khiển", trong khi 6.3 dùng `OB_AUTH` để Pi quyết định arm/OFFBOARD. Hai ý không
> thực sự đá nhau, nhưng câu chữ thì có. Ranh giới đúng: `OB_*` **báo** trạng thái; quyết định
> an toàn **vẫn do FC tự thi hành** — Pi đọc sai `OB_AUTH` thì FC vẫn trả `DENIED` và vẫn bỏ
> qua setpoint khi không có quyền. Pi dùng `OB_*` để **tránh gửi lệnh vô ích**, không phải để
> cấp quyền cho chính mình.
>
> Vì sao không làm bản tin dialect riêng (180–229): phải sinh lại thư viện MAVLink **và** vá
> MAVROS ở phía Pi, trong khi `NAMED_VALUE_INT` chuẩn đã được plugin `debug_value` hỗ trợ sẵn.
> Cái giá phải trả: so khớp chuỗi tên, nên tên đã cấp **không bao giờ đổi** (9.3).

**Chưa cấp ID dialect riêng nào.** Kênh Pi ↔ GCS dùng dialect riêng nhưng **không đi qua
đường FC** (xem mục 10.5), nên không thuộc sổ này.

### 9.2 `custom_mode` — dải cấp phát

| Dải | Chủ | Dùng cho |
|---|---|---|
| **0–15** | FC | chế độ bay cốt lõi. 0–4 đã dùng (mục 6.1), **5–15 để dành** |
| **16–31** | FC | chế độ phái sinh do Pi kích hoạt (LAND tự động, RTL…) — **chưa dùng** |
| 32+ | — | chưa cấp |

Quy tắc: **không bao giờ đổi nghĩa một số đã cấp.** Chế độ bị phế bỏ thì để trống số cũ,
không tái sử dụng.

### 9.3 Tên `NAMED_VALUE_INT` / `NAMED_VALUE_FLOAT`

Giới hạn cứng của đặc tả: **`name` tối đa 10 ký tự**, không có ký tự kết thúc nếu đủ 10.
So khớp **nguyên văn, phân biệt hoa thường**.

| Tiền tố | Chủ | Nghĩa |
|---|---|---|
| `OB_*` | FC | trạng thái OFFBOARD |
| `FC_*` | FC | chẩn đoán FC khác |
| `PI_*` | Pi | dành cho chiều Pi → FC (chưa dùng) |

| Tên | Kiểu | Chủ | Nghĩa | Trạng thái |
|---|---|---|---|---|
| `OB_STATE` | INT | FC | 0 = KHOA, 1 = TAT, 2 = DANG_CHAY | [CHỐT] — đo 09-13 |
| `OB_AUTH` | INT | FC | 1 = Pi còn quyền, 0 = đã mất | [CHỐT] — đo 09-13 |
| `OB_EXIT` | INT | FC | lý do rời gần nhất, bảng 6.3 | [CHỐT] — đo 09-13 |
| `FC_CTR_VER` | INT | FC | phiên bản hợp đồng, mục 10.1 | [CHỐT] — đo 09-13, `10100` |
| `OB_ARM_RDY` | INT | FC | 1 = đang chờ Pi arm, mọi điều kiện thoả (6.3) | [THOẢ THUẬN] — Pi đồng ý 09-13 |
| `OB_ARM_BLK` | INT | FC | bitmask lý do chưa sẵn sàng arm (6.3) — **bảng bit phải đăng ký ở đây khi phát** | [THOẢ THUẬN] — Pi đồng ý 09-13 |
| `OB_RX_OK` | INT | FC | số setpoint hợp lệ đã nhận, cộng dồn (3.4) | [THOẢ THUẬN] — Pi đồng ý 09-13 |
| `OB_RX_REJ` | INT | FC | số setpoint bị loại vì sai `type_mask`/frame/NaN (3.4) | [THOẢ THUẬN] — Pi đồng ý 09-13 |
| `OB_RX_CLP` | INT | FC | số setpoint bị kẹp dải (3.4) | [THOẢ THUẬN] — Pi đồng ý 09-13 |
| `OB_T_FWD` | FLOAT | FC | mục tiêu bay tới đã nhận, m/s (3.4) | [THOẢ THUẬN] — Pi đồng ý 09-13 |
| `OB_T_RGT` | FLOAT | FC | mục tiêu bay sang phải, m/s (3.4) | [THOẢ THUẬN] — Pi đồng ý 09-13 |
| `OB_T_UP` | FLOAT | FC | mục tiêu đi lên, m/s (3.4) | [THOẢ THUẬN] — Pi đồng ý 09-13 |
| `OB_T_YAWR` | FLOAT | FC | mục tiêu quay phải, °/s (3.4) | [THOẢ THUẬN] — Pi đồng ý 09-13 |
| `FC_DIRTY` | INT | FC | 1 = firmware build từ cây có thay đổi chưa commit, 0 = sạch (9.5) | [THOẢ THUẬN] — Pi đồng ý 09-13 |

> Bản 1.0 đề xuất tên `CONTRACT` — **vi phạm quy tắc tiền tố ở trên** (tên do FC phát phải là
> `FC_*` hoặc `OB_*`), và `FC_CONTRACT` dài 11 ký tự, vượt giới hạn 10. Đổi thành `FC_CTR_VER`
> (đúng 10 ký tự, **không** có byte kết thúc). Tên chưa từng phát lên dây nên đổi không phá gì.

**Tên đã cấp thì không đổi.** Phía Pi so khớp chuỗi nguyên văn nên đổi tên = hỏng im lặng.

### 9.4 Mã `OB_EXIT`

Xem bảng mục 6.3. Dải **0–31 thuộc FC**; mã mới thêm vào cuối, **không chèn giữa**.
Pi phải coi mã lạ là "không rõ lý do, không tự phục hồi" chứ không được crash.

### 9.5 Cờ `capabilities` trong `AUTOPILOT_VERSION`

| Cờ | Giá trị | Khai chưa | Ghi chú |
|---|---|---|---|
| `MAVLINK2` | `0x2000` | **có** | |
| `SET_POSITION_TARGET_LOCAL_NED` | `0x0080` | **chưa — cố ý** | đã hiện thực, nhưng theo quy tắc vàng bên dưới và 10.3 bước 7 chỉ khai **sau khi kiểm dấu 3.4 đạt**; lúc đó thành `0x2080` |
| MISSION_*, FTP, PARAM_FLOAT… | — | **cố ý KHÔNG** | |

> **Quy tắc vàng: chỉ khai cờ cho thứ đã hiện thực VÀ đã nghiệm thu.** Khai thừa khiến
> MAVROS bật plugin mà firmware chưa hỗ trợ rồi chờ phản hồi không bao giờ tới — hỏng theo
> kiểu chờ mãi, khó chẩn đoán hơn hỏng ngay.

Hai trường còn lại, [THOẢ THUẬN — FC đã điền 09-13, chờ Pi đo]:

| Trường | Giá trị | Cách điền |
|---|---|---|
| `flight_sw_version` | `0x00010000` | MAJOR.MINOR.PATCH.TYPE = 0.1.0 `DEV`, mã hoá chuẩn MAVLink |
| `flight_custom_version` | 8 byte **nhị phân** | 16 chữ số hex đầu của SHA commit `HEAD`, sinh **mỗi lần build** |

> **Thứ tự byte — FC chốt 09-13 (11.1 #9).** 16 chữ số hex đầu của SHA được coi là **một số
> `uint64`**, rồi ghi lên dây theo **little-endian** (byte thấp nhất trước). Hash
> `8b9b35f7b2223c3a` lên dây thành `3a 3c 22 b2 f7 35 9b 8b`, và MAVROS in **đúng chiều**:
> `VER: Flight software: 00010000 (8b9b35f7b2223c3a)`. Đọc mảng thô bằng `pymavlink` thì phải
> ghép lại như `uint64` little-endian: `struct.unpack('<Q', bytes(msg.flight_custom_version))`.
>
> | Firmware phát `FC_CTR_VER` | Thứ tự byte của hash |
> |---|---|
> | `10100` (hợp đồng 1.1, bản `8b9b35f7…` đang chạy) | theo **chuỗi** — MAVROS in ngược |
> | `10200` trở đi | **`uint64` little-endian** — MAVROS in đúng |

> **Giới hạn cần biết:** build từ cây có thay đổi chưa commit thì hash là của `HEAD`, **không
> đại diện** cho bản build. Bản đang chạy 09-13 là trường hợp này (`8b9b35f7b2223c3a` +
> thay đổi chưa commit). Console `version` in rõ `(CO THAY DOI CHUA COMMIT)`, nhưng **trên dây
> chưa có cờ**. [THOẢ THUẬN — Pi đồng ý 09-13, đã đăng ký ở 9.3] thêm `NAMED_VALUE_INT` `FC_DIRTY` (0/1) vào nhóm 2 Hz. Cho tới
> khi có: **bản firmware dùng để bay phải build từ commit sạch.**

### 9.6 Tham số FC mà Pi cần biết

| Tham số | Giá trị | Ảnh hưởng tới Pi |
|---|---|---|
| `offboard_switch_channel` | mặc định **`7`** (= ch8, đếm từ 0) | `-1` thì OFFBOARD không bao giờ vào được |
| `rc_mode_channel` | `5` (= ch6) | ch6 phải ở nấc POSHOLD (giá trị thô ≥ 1401) thì Pi mới arm và vào OFFBOARD được; `-1` thì không bao giờ được |
| `offboard_timeout_ms` | 500 | quyết định tần số tối thiểu Pi phải phát setpoint |
| Timeout heartbeat Pi | 3000 ms — **hằng số có trong code nhưng KHÔNG được dùng** | mất heartbeat không gây ra gì (11.1 #8). Pi vẫn nên phát ≥ 1 Hz (MAVROS tự phát) |
| `FLOW_RANGE_MAX_MM` | 8000 (8,0 m) | `DISTANCE_SENSOR.max_distance` |
| `EST_RANGE_MAX_M` | 6,0 | ngưỡng Pi tự áp khi dùng cho hạ cánh |
| Nghiêng tối đa cho laser | 25° | quá thì EKF bỏ mẫu |
| σ laser | 0,05 m | `covariance = 25` (cm²) |

---

## 10. Quy tắc mở rộng

Mục này là phần **dành cho tương lai**. Nó tồn tại vì lịch sử dự án cho thấy chi phí lớn
nhất không nằm ở việc thêm một bản tin, mà ở việc hai bên hiểu khác nhau về bản tin đó.

### 10.1 Phiên bản hoá hợp đồng

Hợp đồng mang số **MAJOR.MINOR**:

| Đổi gì | Tăng gì |
|---|---|
| Đổi nghĩa/dấu/đơn vị một trường đang dùng; bỏ một bản tin; đổi `type_mask`; đổi mã `custom_mode` | **MAJOR** |
| Thêm bản tin mới; thêm `NAMED_VALUE_INT` mới; thêm mã `OB_EXIT` mới; khai thêm cờ capability | **MINOR** |
| Sửa lỗi chính tả, thêm giải thích, thêm số đo | không tăng |

**Đổi MAJOR bắt buộc hai bên nạp bản mới cùng lúc.** Đổi MINOR thì bên cũ vẫn chạy được.

Hai bên biết nhau đang ở phiên bản nào bằng ba mốc, theo thứ tự tin cậy giảm dần:

1. **`flight_custom_version`** (8 byte git hash) trong `AUTOPILOT_VERSION` — chính xác
   tuyệt đối, biết đúng bản build nào **nếu build từ commit sạch** (mục 9.5). [THOẢ THUẬN]
2. **`NAMED_VALUE_INT` `FC_CTR_VER`** = `MAJOR × 10000 + MINOR × 100`. Hợp đồng 1.1 →
   `10100`. Phát 2 Hz cùng nhóm `OB_*`. [THOẢ THUẬN — FC đã phát 09-13; tên đổi từ `CONTRACT`,
   mục 9.3]
3. `flight_sw_version` — thô, chỉ cho biết đại khái.

**Quy tắc phía Pi:** `FC_CTR_VER` có MAJOR khác MAJOR mình biết → **từ chối vào OFFBOARD**,
báo `STATUSTEXT`/log rõ ràng, chỉ đọc telemetry. Thà không bay còn hơn bay theo hợp đồng
sai.

### 10.2 Quy tắc tương thích — năm điều bất di bất dịch

**R1. Bản tin lạ, tên `NAMED_VALUE_*` lạ, mã enum lạ → BỎ QUA im lặng, không crash, không
coi là lỗi.** Đây là điều kiện để một bên nạp bản mới trước bên kia.

**R2. Trường mới chỉ được THÊM, không bao giờ đổi nghĩa trường cũ.** Muốn đổi nghĩa thì
thêm trường mới và phế bỏ trường cũ theo mục 10.4 — đừng tái sử dụng.

**R3. Dữ liệu không tin cậy thì GẮN CỜ, đừng thay bằng giá trị "an toàn".** Nguyên tắc đã
áp ở mục 4.2: `0` là một lời nói dối khác. Đi kèm: **mọi trường có thể mất hiệu lực phải
có một cờ đi cùng**, và cờ đó phải nằm ở bản tin **định kỳ**, không phải sự kiện.

**R4. Mọi lệnh đều phải có đường trả lời.** `COMMAND_LONG` → `COMMAND_ACK` (mục 7). Bản
tin không có cơ chế ACK (`SET_MODE`, `SET_POSITION_TARGET_*`) **không được dùng cho hành
động một lần không thể lặp**. Hệ quả trực tiếp: đường vào OFFBOARD dùng công tắc RC chứ
không dùng `SET_MODE`.

**R5. Trạng thái phải hỏi lại được.** Mọi thứ Pi cần biết để quyết định phải có ở một bản
tin **định kỳ**, vì node có thể restart bất cứ lúc nào và sẽ lỡ mọi sự kiện trước đó. Đây
là lý do `NAMED_VALUE_INT` được ưu tiên trên `STATUSTEXT`.

### 10.3 Quy trình thêm một bản tin mới — 8 bước

Áp cho mọi bản tin trong danh sách [THOẢ THUẬN]/[ĐỀ XUẤT] ở mục 9.1.

1. **Chọn bản tin chuẩn**, không tự chế. Nếu không có bản tin chuẩn nào hợp thì ghi rõ vì
   sao trước khi nghĩ tới dialect riêng.
2. **Ghi vào sổ 9.1** với trạng thái [ĐỀ XUẤT] + bên nào làm + bên nào bị chặn.
3. **Chốt cách điền từng trường** trong tài liệu này, **trước khi viết code**. Gồm: đơn vị,
   hệ quy chiếu, giá trị "không biết", và **trường nào mang cờ hiệu lực**.
4. **FC hiện thực và đo trên console** trước khi phát lên dây.
5. **Pi bật plugin MAVROS tương ứng và xác minh topic tồn tại** — bật trước, để lúc FC phát
   là thấy ngay. Plugin sai tên thì im lặng (bài học `'battery'`).
6. **Đo tần số và nội dung trên dây bằng `pymavlink`**, đối chiếu bảng 4.1. Cập nhật bảng.
7. **Khai cờ capability tương ứng** (mục 9.5) — chỉ sau khi bước 6 xong.
8. **Đổi trạng thái sang [CHỐT].** Bước này **không** tăng số hợp đồng. MINOR đã tăng từ lúc
   firmware **bắt đầu phát** thứ mới (bước 4), đồng thời ở hai chỗ: `MAV_CONTRACT_MINOR` trong
   firmware và bảng lịch sử ở đây — để `FC_CTR_VER` trên dây luôn khớp tài liệu (11.1 #10).

> Bước 3 là bước hay bị bỏ nhất, và là bước đắt nhất khi bỏ. Ba lần sửa lại `type_mask`,
> bảng `custom_mode`, và hợp đồng ARM đều vì bước này làm sau code.

### 10.4 Phế bỏ một mục

1. Đánh dấu **[PHẾ BỎ từ ngày…]** trong tài liệu này, nêu thứ thay thế. **Vẫn phát/vẫn
   nhận.**
2. Bên tiêu thụ chuyển sang cái mới, xác minh bằng phép đo.
3. Sau ít nhất một chu kỳ nghiệm thu, ngừng phát. Tăng **MAJOR**.
4. **Số/tên đã cấp vĩnh viễn không tái sử dụng** — để trống trong sổ 9.

### 10.5 Ranh giới rõ: cái gì KHÔNG thuộc hợp đồng này

| Việc | Đi đường nào | Vì sao không qua FC |
|---|---|---|
| Pi ↔ GCS (nhiệm vụ, telemetry, lệnh khẩn) | **4G/LTE gắn thẳng Pi**, dialect riêng | không phải sửa firmware FC để forward; đường FC vốn dành cho điều khiển |
| Khung nhị phân riêng tới ESP32 | USART3 của FC | không trộn với UART8 |
| Console CLI, hiệu chuẩn, `set`/`save` | USART1 của FC | |
| Ảnh camera, AprilTag thô, EKF của Pi | nội bộ ROS 2 | FC không cần biết |

**Không mở rộng đường MAVLink FC↔Pi để chở dữ liệu nhiệm vụ.** Đường đó là đường điều
khiển; nhồi thêm sẽ làm mờ ranh giới trách nhiệm ở mục 1 và làm băng thông điều khiển phụ
thuộc vào tải nhiệm vụ.

### 10.6 Ba hướng mở rộng đã lường trước

Ghi ở đây để khi làm không phải thiết kế lại từ đầu.

**a) Chuyển `MAV_FRAME_BODY_NED` (8) → `MAV_FRAME_LOCAL_NED` (1).**
Điều kiện mở cổng ghi ở bản 1.0: từ kế QMC5883P hiệu chuẩn xong, và bit từ kế trong
`fields_updated` bật **đúng theo cổng `calibrated`**. **Điều kiện đó nay đã thoả một cách
hình thức** — nhưng không đủ: `calibrated` chỉ có nghĩa "offset/scale khác ma trận đơn vị",
không chứng minh hướng đúng.

[THOẢ THUẬN — Pi đồng ý 09-13; không có việc trước giai đoạn có điện động cơ] điều kiện thật: (1) đo sai lệch hướng so với một la bàn tham chiếu ở ít nhất
8 hướng; (2) **đo lại khi có điện động cơ** — dòng động cơ làm lệch từ trường; (3) xác nhận
module, dây nguồn, cách bắt bo không đổi từ lần hiệu chuẩn 09-04. Việc phải làm: đổi
`coordinate_frame`, chạy lại toàn bộ quy trình kiểm dấu 3.4 (**phép biến đổi yaw và trục
của MAVROS khác nhau giữa hai frame**), tăng MAJOR.

**b) Thêm GPS / VIO → mở đường lệnh VỊ TRÍ.**
Chỉ khi có nguồn vị trí tuyệt đối. Việc phải làm: FC thêm vòng P vị trí; đổi `type_mask`
(bit 0–2 về `000`) → **tăng MAJOR**; cấp `custom_mode` mới cho LAND/RTL trong dải 16–31;
bật `GPS_RAW_INT` (24). **Đừng làm từng phần** — nửa vời là kiểu bay để bù cho phần trôi
không có thật, đã nói ở mục 1.

**c) Chuyển `LOCAL_POSITION_NED` → `ODOMETRY` (331).**
`ODOMETRY` mang covariance nên **giải quyết dứt điểm** cả vấn đề covariance đoán mò (8.4)
lẫn vấn đề cờ hiệu lực (4.2a) — vận tốc không hợp lệ chỉ cần covariance rất lớn. Cả hai
EKF của FC đã giữ ma trận hiệp phương sai nội bộ; `ekf_velocity_uncertainty_mps()` đã được
khai. Việc phải làm: FC điền covariance, Pi bật plugin `odometry`, chạy song song một chu
kỳ rồi mới phế bỏ `LOCAL_POSITION_NED` theo mục 10.4.

> Nhắc lại cảnh báo 8.4 khi làm (c): đó là **bộ lọc tự chấm điểm chính nó**. Dùng làm trọng
> số tương đối thì tốt; coi là sai số tuyệt đối thì không.

---

## 11. Bảng trạng thái — mục chưa chốt

Sổ theo dõi chi tiết vẫn là `viec_can_lam_mavlink.md`. Bảng này chỉ liệt kê thứ **chạm vào
hợp đồng**, để không ai viết code dựa vào chỗ chưa xong.

### 11.1 Đang chặn — không viết code dựa vào

| # | Việc | Chủ | Chặn gì | Trạng thái |
|---|---|---|---|---|
| ~~1~~ | ~~Phát `NAMED_VALUE_INT` `OB_STATE`/`OB_AUTH`/`OB_EXIT` @2 Hz~~ | FC → Pi đo | — | **XONG 09-13** — Pi đo trên dây và qua MAVROS (4.1). Hành vi chuyển trạng thái còn ở 12.A4 |
| **2** | Chạy quy trình kiểm dấu (mục 3.4) | **Pi** | mọi lệnh điều khiển; đóng mục 3.1–3.2 bằng phép đo | [ĐANG LÀM] — **việc đáng làm nhất hiện nay**. **Chạy lại nếu đã chạy trước 09-13** (lỗi bit 9, mục 5.2) |
| **3** | `fields_updated` vẫn bật cờ từ kế | **FC** | tin cậy `HIGHRES_IMU.mag`; cổng mở `FRAME_LOCAL_NED` | **ĐÃ TRẢ LỜI 09-13** — đúng thiết kế |
| ~~4~~ | ~~Launch MAVROS crash~~ | Pi | — | **XONG 2026-09-13** — xem mục 8.3 |
| **5** | Đường arm đi qua cổng nào? | **FC** | nghiệm thu 12.B | **ĐÃ TRẢ LỜI 09-13** — chỉ UART8 |
| **6** | Pi dùng `1`/`191` trùng `sysid` với FC — FC có lọc bỏ không? | **FC** | `mavros.yaml`, heartbeat, mọi lệnh | **XONG 09-13** — FC không lọc; Pi đo: `/mavros/cmd/command` 520 và 512 qua `1`/`191` đều `result = 0` |
| **7** | Ba mâu thuẫn + hai điểm nhỏ Pi báo | **FC** | — | **ĐÃ TRẢ LỜI 09-13** — xem chi tiết |
| **8** | Timeout heartbeat Pi 3000 ms không được dùng | **FC** → hai bên chốt | hiểu đúng lưới an toàn | [ĐỀ XUẤT] — xem chi tiết |
| **9** | Thứ tự byte `flight_custom_version` | **FC** | đọc đúng bản build từ log MAVROS | **ĐÃ TRẢ LỜI 09-13** — đổi sang `uint64` little-endian từ 1.2 |
| **10** | Chuyển [CHỐT] có phải tăng MINOR không (10.3 bước 8) | **hai bên** | số `FC_CTR_VER` | **FC TRẢ LỜI 09-13** — không; MINOR tăng khi firmware phát thứ mới. Đã sửa 10.3 bước 8, chờ Pi phản đối nếu có |

**Chi tiết #3 — trả lời của FC:** phép đo của Pi **đúng**, câu trả lời đợt 1 của FC **sai**.
`0x1BFF` (bit từ kế bật) là hành vi đúng thiết kế.

- Từ kế QMC5883P **đã hiệu chuẩn thật** ngày 2026-09-04, trên máy bay lắp hoàn chỉnh (pin, dây
  nguồn, động cơ đúng chỗ, cánh tháo), bằng phép khớp 6 tham số. Bộ số được ghi làm giá trị
  mặc định trong `fc_config.h`: offset `0.0404 / 0.0763 / -0.1861` G, scale
  `0.9863 / 0.9885 / 1.0257`.
- Firmware định nghĩa `calibrated` = "offset/scale khác ma trận đơn vị" → `true` → bit bật.
- Đợt 1 FC trả lời "chưa hiệu chuẩn" vì **dòng log khởi động in "CHUA HIEU CHUAN" vô điều
  kiện**, không đọc cờ. Lỗi log đó đã sửa 09-13; nay log in theo cờ thật.
- 1,2 % mẫu tắt bit và 1/61 mẫu `health = 0x1010B` là **`mag.healthy` chập chờn** — việc riêng,
  đáng tìm hiểu, không làm sai hợp đồng.

Hệ quả: cổng mở `FRAME_LOCAL_NED` ghi ở mục 10.6a **đã thoả hình thức nhưng không đủ** — xem
đề xuất điều kiện thật ở đó.

**Chi tiết #5 — trả lời của FC:** đường arm **chỉ đi qua UART8**, không có cổng nào khác.
"Chưa kiểm được đường arm từ Pi" là vì **bàn test phía FC** chỉ vào được bo mạch qua ST-Link
(SWD) và console USART1 — không có đường nào đẩy MAVLink vào UART8 từ phía đó. Pi là bên duy
nhất gửi được, nên nghiệm thu 12.B đường ARM **phải do Pi chạy**.

**Chi tiết #6 — câu hỏi của Pi:** mục 2 bản 1.0 ghi Pi là `255/190`, nhưng MAVROS thật chạy `1/191` (mặc
định, `mavros.yaml` không đặt). `1/191` là cách chuẩn cho máy tính đồng hành — cùng
`sysid` với autopilot, khác `compid`. **Câu hỏi cho FC:** firmware có lọc gói theo `sysid`
nguồn không (ví dụ bỏ gói có `sysid` trùng mình, hoặc chỉ tính heartbeat Pi khi `sysid =
255`)? Nếu có thì timeout heartbeat 3000 ms (5.1) sẽ không bao giờ được thoả và lệnh từ
MAVROS bị vứt. Nghiệm thu ACK ở mục 7 dùng `pymavlink` 255/190 nên **chưa phủ trường hợp
này**. Pi có thể đổi sang 255/190 bằng `system_id`/`component_id` trong `mavros.yaml` nếu FC
yêu cầu — nhưng chờ FC chốt, không tự đổi.

**Chi tiết #6 — trả lời của FC:** `1`/`191` **chạy được. Giữ nguyên `mavros.yaml`.**

Đọc lại toàn bộ đường nhận trong `mav_link.c`: FC **không kiểm `sysid`/`compid` nguồn** ở bất kỳ
bản tin nào.

| Bản tin | FC kiểm gì |
|---|---|
| `HEARTBEAT` | không kiểm gì — nhận từ bất kỳ ai |
| `COMMAND_LONG` | chỉ `target_system` (đích) ∈ {0, 1} |
| `SET_POSITION_TARGET_LOCAL_NED` | chỉ `target_system` (đích) ∈ {0, 1} |
| `COMMAND_ACK` gửi trả | `target_system`/`target_component` = đúng nguồn khung vừa nhận |

`1`/`191` là cấu hình chuẩn cho máy tính đi kèm (`MAV_COMP_ID_ONBOARD_COMPUTER`), khuyên giữ.

**Chưa đo trên dây:** lúc FC kiểm 09-13, bộ đếm cho thấy FC **chưa nhận khung MAVLink nào** kể
từ lần nạp gần nhất — Pi không chạy MAVROS lúc đó. Pi xác nhận bằng cách gọi
`/mavros/cmd/command` với lệnh 520: có ACK về đúng service là đường `1`/`191` thông.

> Nếu sau này FC **thêm** lọc theo nguồn, đó là thay đổi **MAJOR** (mục 10.1) — Pi đang dựa vào
> việc FC không lọc.

**Chi tiết #7 — câu hỏi của Pi: ba chỗ giao ước tự mâu thuẫn, Pi không tự chọn:**

| # | Chỗ A | Chỗ B | Cần FC chốt |
|---|---|---|---|
| 7a | 6.2: `offboard_switch_channel` **"nay mặc định BẬT"** (= 7) | 6.3 và 9.6: **"mặc định `-1`, tắt hẳn"**, phải `set = 7` rồi `save` | mặc định thật là gì |
| 7b | 5.2 (sau bảng `type_mask`): hết hạn setpoint → **"lùi về POSHOLD"** | 5.2 đoạn "Hết hạn setpoint" và 12.B: **"lùi về ANGLE"** | lùi về chế độ nào |
| 7c | 9.1: `NAMED_VALUE_*` là **"chẩn đoán tạm — không dùng cho đường điều khiển sản phẩm"** | 6.3, 10.1: `OB_AUTH`/`CONTRACT` qua `NAMED_VALUE_INT` là **kênh quyết định** của `mission_manager_node` (arm, từ chối OFFBOARD) | chấp nhận ngoại lệ cho `OB_*`/`CONTRACT` và ghi vào 9.1, hay chuyển sang bản tin khác |

**Chi tiết #7 — trả lời của FC cho năm điểm Pi báo:**

| Điểm | Trả lời |
|---|---|
| (a) mặc định `offboard_switch_channel` | **Đã sửa ở 1.1** — Pi đang đọc bản dựng từ 1.0. Đúng là `7` (mục 6.3, 9.6) |
| (b) hết hạn setpoint lùi về đâu | **Đã sửa ở 1.1**, và nay FC còn **bắt buộc ch6 ở POSHOLD** nên đường lùi luôn là POSHOLD (5.2) |
| (c) `NAMED_VALUE_*` "chỉ chẩn đoán" vs `OB_AUTH` | **Đúng, là mâu thuẫn câu chữ.** Đã viết lại quy tắc 9.1: tên đăng ký là kênh báo trạng thái chính thức, chỉ báo, không ra lệnh |
| Console USART1 cho 3.4 | **Pi không có** — console ở máy bàn test FC. Tạm thời làm chung phiên; đề xuất `NAMED_VALUE_FLOAT` `OB_T_*` + bộ đếm `OB_RX_*` để Pi tự làm (mục 3.4) |
| Dòng TAT trong bảng 6.3 | **Pi đúng**, và lỗi còn sâu hơn: `OB_AUTH = 1` mà ARM vẫn có thể `DENIED`. Đã sửa bảng, đề xuất `OB_ARM_RDY` (6.2, 6.3) |

**Chi tiết #8 — phát hiện của FC 09-13:** `mav_link_ok()` (so thời điểm heartbeat cuối với
3000 ms) **không được gọi ở bất cứ đâu** trong firmware. Mất heartbeat của Pi hiện **không gây
ra hành động nào**. Lưới an toàn **duy nhất** khi Pi chết là **timeout setpoint 500 ms** (5.2).

Hệ quả cho Pi: node **publish setpoint** chết thì FC phát hiện trong 500 ms. Nhưng nếu node
setpoint vẫn sống mà phần còn lại của Pi treo (ví dụ `mission_manager` kẹt, setpoint vẫn phát
đều lệnh cũ), **FC không biết** — việc giám sát đó thuộc về Pi.

[THOẢ THUẬN — Pi đồng ý 09-13] **Không thêm** hành động khi mất heartbeat: timeout setpoint đã chặt hơn cho
đường điều khiển, còn khi chưa OFFBOARD thì người lái đang cầm máy bay. Chỉ cần sửa lại câu chữ
ở 5.1 và 9.6 (đã sửa) để không ai tưởng heartbeat là một cổng an toàn.

> *Phản hồi Pi 09-13:* đồng ý. Pi nhận phần việc giám sát nội bộ: ghi thành nợ **P6** ở 11.3
> (watchdog để `position_controller_node` ngừng phát khi `mission_manager_node` treo — ngừng phát
> thì FC tự hết hạn sau 500 ms, đúng đường an toàn đã có).

**Chi tiết #9 — câu hỏi của Pi 09-13:** FC ghi 8 byte hash theo thứ tự chuỗi
(`8b 9b 35 f7 b2 22 3c 3a`). MAVROS đọc thành `uint64` little-endian và in
`3a3c22b2f7359b8b` — ai tra hash từ log MAVROS sẽ không tìm thấy commit. Theo hiểu biết của
Pi (**chưa kiểm trên PX4 thật**), PX4 ghi hash dạng `uint64` little-endian, nên MAVROS và
QGroundControl in đúng chiều. **Câu hỏi cho FC:** đảo thứ tự byte cho khớp quy ước đó, hay giữ
nguyên và ghi vào đây "đọc log MAVROS thì đảo ngược"? Pi không phụ thuộc chiều nào, chỉ cần
hai bên ghi một chiều.

**Chi tiết #9 — trả lời của FC:** **đổi sang `uint64` little-endian**, áp dụng từ firmware hợp
đồng **1.2** (đợt A–E). Cách ghi cụ thể ở 9.5.

- **Lý do:** để công cụ chuẩn đọc đúng — người tra hash thường đọc từ log MAVROS hoặc GCS, không
  đọc mảng byte thô. Lý do này đứng vững nhờ **hành vi MAVROS Pi đã đo**, không cần dựa vào
  nhận định về PX4 (Pi tự ghi là chưa kiểm; FC cũng chưa kiểm PX4).
- **Vì sao không phải MAJOR** dù đổi cách ghi một trường đã [CHỐT]: trường mới chốt trong ngày
  và Pi xác nhận **không bên nào dựa vào chiều cũ**. Đây là ngoại lệ có ghi lại, không phải
  tiền lệ — sau hôm nay, đổi cách ghi trường đã chốt vẫn là MAJOR theo 10.1.
- **Không nhầm được giữa hai bản:** cùng khung 2 Hz đã có `FC_CTR_VER` — `10100` là thứ tự
  chuỗi, `10200` trở đi là `uint64` little-endian (bảng ở 9.5).

**Chi tiết #10 — câu hỏi của Pi 09-13:** 10.3 bước 8 ghi "đổi sang [CHỐT] **và tăng MINOR**".
Ngày 09-13 Pi đã chuyển `DISTANCE_SENSOR`, `AUTOPILOT_VERSION`, `REQUEST_MESSAGE` và 4 tên
`NAMED_VALUE_INT` sang [CHỐT] theo phép đo, nhưng **chưa tăng số** — tăng lên 1.2 thì
`FC_CTR_VER` trên dây (`10100`) lệch tài liệu cho tới khi FC nạp lại. Cần chốt: bước 8 áp cho
bản tin FC **đã** tính vào 1.1 (thì không tăng), hay mỗi lần [CHỐT] đều tăng?

**Chi tiết #10 — trả lời của FC:** **không tăng khi chuyển [CHỐT].** Pi làm đúng khi giữ 1.1.

Số hợp đồng theo dõi **cái gì đang đi trên dây**, không theo trạng thái nghiệm thu. Chốt một thứ
FC đã phát từ 1.1 không làm dây thay đổi gì, nên tăng số lúc đó chỉ sinh ra đúng cái lệch Pi
nêu: tài liệu 1.2 trong khi `FC_CTR_VER = 10100`.

Quy tắc đề xuất — đã sửa vào 10.3 bước 8:

| Sự kiện | Số hợp đồng |
|---|---|
| Firmware **bắt đầu phát** bản tin / tên / mã / cờ mới | tăng MINOR, **cùng lúc** ở `MAV_CONTRACT_MINOR` và bảng lịch sử |
| Pi đo xong, chuyển [CHỐT] | không tăng |
| Sửa câu chữ, thêm số đo, trả lời câu hỏi | không tăng |

Áp vào đợt kế tiếp: A–E (`OB_ARM_RDY`, `OB_T_*`, `FC_DIRTY`, `ODOMETRY`, `RC_CHANNELS`, thứ tự byte
hash) là thứ mới → **hợp đồng 1.2**, firmware đó phát `FC_CTR_VER = 10200`.

Nếu Pi phản đối cách hiểu này, ghi ngay dưới đây; FC chưa nạp 1.2 cho tới khi hai bên thống nhất.

### 11.2 Đã thoả thuận, chờ hiện thực — thứ tự đã chốt hai bên

| # | Việc | Chủ | Mở khoá gì |
|---|---|---|---|
| 1 | Trạng thái OFFBOARD lên dây | FC | `mission_manager_node` — **XONG, Pi nghiệm thu 09-13** |
| 2 | `AUTOPILOT_VERSION` đầy đủ + `REQUEST_MESSAGE` (512) | FC | phiên bản hoá hợp đồng (10.1) — **XONG, Pi nghiệm thu 09-13**, trừ cờ capability (9.5) và thứ tự byte hash (11.1 #9) |
| 3 | `DISTANCE_SENSOR` (132) | FC | hạ cánh chính xác — **XONG, Pi nghiệm thu 09-13**; còn `min_distance` thật |
| 4 | `ODOMETRY` (331) | FC | đưa vận tốc FC vào EKF (10.6c) — **Pi chốt cách điền 09-13**, mời FC viết code |
| 5 | `RC_CHANNELS` (65) | FC | **chỉ hiển thị** — không suy quyền từ nó — **Pi chốt cách điền 09-13**, mời FC viết code |

**Thứ tự Pi đề nghị cho đợt FC tiếp theo (09-13)** — xếp theo thứ đang chặn code phía Pi:

| # | Việc | Mở khoá gì bên Pi |
|---|---|---|
| A | `OB_ARM_RDY` + `OB_ARM_BLK` kèm bảng bit ở 9.3 (6.3) | đường ARM của `mission_manager_node` — không có thì phải thử lại mù |
| B | `OB_T_*` + `OB_RX_*` (3.4) | Pi tự kiểm dấu không cần console — **việc 11.1 #2, đáng làm nhất** |
| C | `FC_DIRTY` (9.5) + trả lời 11.1 #9, #10 | biết chắc bản build đang chạy |
| D | `ODOMETRY` (331) | đưa vận tốc FC vào EKF |
| E | `RC_CHANNELS` (65) | hiển thị |

`OPTICAL_FLOW_RAD` và `VIBRATION` để sau.

Cách điền `DISTANCE_SENSOR` đã thống nhất: `type = MAV_DISTANCE_SENSOR_LASER`,
`orientation = PITCH_270`, `max_distance = 800` cm, `covariance = 25` (cm², σ = 5 cm),
`signal_quality = 1` khi ngoài tầm/không hợp lệ (đặc tả định nghĩa rõ `1 = invalid signal`),
`range_quality` quy về 2–100 % khi hợp lệ. **Không dùng mẹo `current_distance = max_distance + 1`** —
đó là quy ước riêng của vài firmware, đặc tả MAVLink không định nghĩa. Tần số **20 Hz**,
`id = 0`, FOV = 0 (không biết), quaternion = 0 (chỉ dùng khi hướng `CUSTOM`). `max_distance`
lấy từ tham số `flow_range_max_mm` lúc chạy, nên luôn khớp ngưỡng firmware đang loại mẫu.

**`min_distance` CHƯA XÁC ĐỊNH** — firmware không đặt cận dưới, chưa ai đo. Khi phát bản
tin trước lúc đo xong, phải **ghi rõ trong tài liệu con số đang điền là tạm**, để Pi không
dùng nó làm ngưỡng. Đo trên bàn: ổn định 0,19–0,23 m, `range_quality = 255`.

**FC đang điền `min_distance = 1` cm — SỐ TẠM.** Chọn thấp để MAVROS không vứt mẫu gần mặt đất
đang đọc được. **Pi không dùng số này làm ngưỡng.** Đo trên target 09-13: `current_distance =
17` cm, `signal_quality = 100`, các trường còn lại đúng như trên. **Pi đo lại trên dây và qua
MAVROS 09-13: khớp toàn bộ** (mục 4.1). Riêng `covariance = 25` **không** tới được ROS —
`Range.variance` luôn 0.

**Pi kiểm plugin `odometry` của MAVROS 09-13** — theo yêu cầu bên dưới. Cách đo: dựng FC giả
bằng `pymavlink` qua UDP (**không** chạm FC thật), phát `ODOMETRY` giá trị đã biết vào
`mavros_node` Jazzy, đọc `/mavros/odometry/in`:

| Phát (hệ MAVLink) | Nhận trên ROS | Đúng? |
|---|---|---|
| `x, y, z` = N 1, E 2, D −3 | `position` = (2, 1, 3) ENU | đúng |
| `q` đơn vị (mũi hướng bắc) | `q` = yaw 90° ENU | đúng |
| `vx, vy, vz` = tới 1, phải 0,5, xuống 0,2 (thân FRD) | `twist.linear` = (1, −0,5, −0,2) FLU | đúng |
| `yawspeed` = +0,1 (quay phải) | `twist.angular.z` = −0,1 | đúng |
| `pose_covariance` đường chéo (1, 4, 9, 16, 25, 36) | (4, 1, 9, 25, 16, 36) — hoán x↔y, roll↔pitch | đúng |
| `velocity_covariance` đường chéo (100, 400, **1e6**, …) | (100, 400, **1e6**, …) — `1e6` đi qua nguyên | đúng |
| Header ROS | `frame_id = odom`, `child_frame_id = base_link` | — |

**Tổ hợp FC đề xuất (`frame_id = 1`, `child_frame_id = 12`) cho kết quả đúng.** Nhưng có hai
điểm FC phải biết trước khi viết code:

1. **MAVROS KHÔNG đọc `frame_id` / `child_frame_id`.** Phát bốn tổ hợp `(1,12)`, `(18,12)`,
   `(1,8)`, và cả tổ hợp vô lý `(20,12)` (LOCAL_FLU) — **ROS nhận y hệt nhau**, không cảnh báo.
   MAVROS luôn giả định vị trí NED và vận tốc thân FRD. Tức FC **bắt buộc** điền vận tốc hệ
   thân FRD như đề xuất; lỡ điền vận tốc NED thì MAVROS đổi sai **im lặng**, và hai trường
   frame không cứu được.
2. **`quality` không tới được ROS** — `nav_msgs/Odometry` không có chỗ chứa. Bên Pi chỉ thấy
   covariance. Tức khi dùng `ODOMETRY`, cờ hiệu lực vận tốc **chỉ còn là `1e6`**. Việc này đụng
   quy tắc 4.2a ("bỏ hẳn mẫu, KHÔNG hạ trọng số") — **Pi chưa chốt**, xem 11.3 P3.

#### Cách điền `ODOMETRY` (331) [THOẢ THUẬN — Pi chốt 09-13, mời FC viết code]

> *Phản hồi Pi 09-13:* **đồng ý toàn bộ bảng dưới**, sau khi kiểm MAVROS bằng FC giả (kết quả
> ngay trên). Ba điều FC cần giữ khi viết code:
> 1. `vx, vy, vz` **phải** là hệ thân FRD — MAVROS không đọc `child_frame_id` nên không có lưới
>    nào bắt lỗi điền nhầm NED. Kiểm dấu bằng tay trên bàn (dịch drone tới trước → ROS
>    `twist.linear.x > 0`) trước khi chuyển [CHỐT].
> 2. Vẫn điền `frame_id = 1`, `child_frame_id = 12` đúng đặc tả dù MAVROS bỏ qua — để công cụ
>    khác (log `pymavlink`, GCS) đọc đúng.
> 3. `quality` không tới được ROS, nên **covariance `1e6` là cờ hiệu lực duy nhất phía Pi** — xin
>    đảm bảo `1e6` được đặt **cùng khung** với lúc bit flow ở `SYS_STATUS` tắt, không trễ.
>
> Cách Pi dùng `1e6` (bỏ hẳn mẫu hay đưa vào EKF) là việc nội bộ Pi — 11.3 P3, **không chặn FC**.

Theo 10.3 bước 3: chốt **trước**. Hệ quy chiếu là chỗ dễ sai nhất, nên **Pi xác minh MAVROS
plugin `odometry` chấp nhận đúng tổ hợp frame dưới đây bằng phép đo** trước khi FC viết code.

| Trường | Đề xuất |
|---|---|
| Tần số | 30 Hz, chạy **song song** `LOCAL_POSITION_NED` một chu kỳ rồi mới phế bỏ (10.4) |
| `frame_id` | `MAV_FRAME_LOCAL_NED` (1) — cho `x, y, z` và `q` |
| `child_frame_id` | `MAV_FRAME_BODY_FRD` (12) — cho `vx, vy, vz` và tốc độ góc |
| `x, y, z` | `est.position_m` (NED) |
| `q` | `est.attitude_q` |
| `vx, vy, vz` | vận tốc **hệ thân FRD** (xoay từ NED bằng yaw) |
| `rollspeed…yawspeed` | `est.rate_dps` đổi rad/s |
| `pose_covariance` x, y | `(uncertainty_mps × t_từ_lúc_khoá)²`; **`1e6` khi `position_valid` sai** |
| `pose_covariance` z | từ hiệp phương sai EKF độ cao |
| `pose_covariance` roll/pitch | **cố định `(1°)²`** theo mục 8.4 — không dùng số bộ lọc tự chấm |
| `pose_covariance` yaw | **`1e6`** cho tới khi hướng từ kế được kiểm chứng (10.6a) |
| `velocity_covariance` vx, vy | `ekf_velocity_uncertainty_mps()²`; **`1e6` khi bit flow tắt** — thay thế quy tắc 4.2a |
| `velocity_covariance` vz | từ EKF độ cao |
| `reset_counter` | tăng mỗi lần tích phân vị trí được đặt lại |
| `estimator_type` | `MAV_ESTIMATOR_TYPE_UNKNOWN` (0) |
| `quality` | 0 khi vận tốc không hợp lệ, 100 khi hợp lệ |

#### Cách điền `RC_CHANNELS` (65) [THOẢ THUẬN — Pi chốt 09-13, mời FC viết code]

> *Phản hồi Pi 09-13:* **đồng ý toàn bộ bảng dưới.** Đã kiểm plugin `rc_io` bằng FC giả: `chan*_raw`
> tới `/mavros/rc/in` nguyên đơn vị µs, đúng `chancount = 16` kênh (kênh 17–18 bị bỏ), `rssi`
> đi qua nguyên giá trị thô 0–255 (200 và 255 đều thấy). Pi sẽ bật `rc_io` khi FC phát. **Lưu ý
> an toàn:** plugin `rc_io` có topic `/mavros/rc/override` sinh `RC_CHANNELS_OVERRIDE` — FC
> **không** được xử lý bản tin đó (không có trong 5.1), và phía Pi không publish vào đó.

**Chỉ để hiển thị.** Không suy quyền của Pi từ bản tin này (mục 6.3).

| Trường | Đề xuất |
|---|---|
| Tần số | 5 Hz |
| `chancount` | 16 |
| `chan1_raw…chan16_raw` | µs, quy đổi CRSF chuẩn: `µs = (raw − 992) × 5/8 + 1500` (172 → 988, 1811 → 2012) |
| `chan17_raw, chan18_raw` | `UINT16_MAX` (không dùng) |
| `rssi` | `link_quality` % quy về 0–254; **255 khi mất sóng** (đặc tả: 255 = không biết) |

### 11.3 Nợ phía Pi, chạm hợp đồng

| # | Việc | Trạng thái |
|---|---|---|
| P1 | `position_controller_node:55` vẫn subscribe `/mavros/landing_target/raw` (topic **không tồn tại**, mồ côi sau khi sửa `landing_target_bridge_node`) | **chưa sửa** — phải chốt kiến trúc đóng vòng hạ cánh (Pi vòng vận tốc **vs** FC xử lý `LANDING_TARGET`) |
| P2 | `mission_manager_node` theo hợp đồng ARM mới + `OB_AUTH` | chờ #1 |
| P3 | Bộ lọc vận tốc theo bit flow (mục 4.2a) | chưa cần — EKF hiện lấy vận tốc từ camera Pi, **chưa dùng vận tốc FC**. **Chưa chốt:** khi chuyển sang `ODOMETRY`, Pi chỉ thấy covariance `1e6` (MAVROS bỏ `quality`, 11.2) — chấp nhận `1e6` thay cho "bỏ hẳn mẫu", hay Pi tự loại mẫu có covariance ≥ ngưỡng trước khi vào EKF |
| P4 | Cảnh báo `errors_count*` theo tốc độ tăng | chưa viết |
| P5 | Đo lại `linear_acceleration_stdev` khi có điện động cơ | giai đoạn C |
| P6 | Watchdog nội bộ: `position_controller_node` ngừng phát setpoint khi `mission_manager_node` treo (FC không dùng heartbeat Pi — 11.1 #8) | chưa viết |
| P7 | Bật `rc_io` khi FC phát `RC_CHANNELS`; không bao giờ publish `/mavros/rc/override` | chờ FC |

### 11.4 Chưa ai đo — cả hai bên nên biết

- **Sai số góc TUYỆT ĐỐI** của bộ ước lượng — cần bàn xoay hoặc mặt phẳng chuẩn.
- **Trôi yaw khi có điện động cơ** — con số 0,00°/phút chỉ đúng lúc nằm im, nhiệt độ ổn định.
- **`min_distance` thật của MTF01P.**
- **Rung khi có điện động cơ** ảnh hưởng covariance IMU thế nào.
- `BATTERY_STATUS` với pin thật — chưa gắn pin nên chưa xác minh được cách điền `voltages[]`.

---

## 12. Nghiệm thu

Chia theo **điều kiện thực hiện**, vì drone đang ở bàn test — **chưa cấp điện tầng công
suất động cơ**.

### 12.A — Bàn test: FC chỉ cần cấp điện, drone đứng im

#### A1. FC phát đủ bản tin

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

- [x] 9 bản tin đúng tần số bảng 4.1, 0 khung hỏng
- [x] Hết cảnh báo `VER: ... switched to default capabilities`
- [ ] `relative_alt` đổi **đúng chiều** khi nhấc drone bằng tay
- [ ] Dấu `vx`/`vy` đúng khi dịch drone ngang bằng tay trên sàn có vân
- [ ] Bit `MAV_SYS_STATUS_SENSOR_OPTICAL_FLOW` **hạ** khi che cảm biến flow
- [x] `fields_updated` = `0x1BFF` (bit từ kế bật) — **đúng thiết kế**, mục 11.1 #3

#### A2. Topic ROS có dữ liệu

```bash
ros2 run mavros mavros_node --ros-args -r __ns:=/mavros \
  --params-file install/drone_bringup/share/drone_bringup/config/mavros.yaml
ros2 topic hz /mavros/imu/data_raw
ros2 topic echo /mavros/state --once
```

- [x] `/mavros/state` báo `connected: true`
- [x] Tần số topic khớp mục 4.1
- [ ] `mode` hiện `CMODE(n)` khớp bảng 6.1 khi gạt công tắc chế độ RC (**đổi chế độ không
      cần arm**)
- [x] `ros2 launch` với node MAVROS của `estimation.launch.py` chạy được, 0 lỗi (09-13).
      Chưa chạy cả `estimation.launch.py` (kéo theo perception).

#### A3. Hợp đồng lệnh

- [x] `NAV_TAKEOFF (22)` → `UNSUPPORTED`, dưới 0,01 s
- [x] `DISARM (400, p1=0)` → `ACCEPTED`, dưới 0,01 s, `armed` giữ `False`
- [ ] **Quy trình kiểm dấu mục 3.4** — bốn trục + ba bộ đếm

#### A4. Trạng thái OFFBOARD (sau khi FC phát `NAMED_VALUE_INT`)

- [ ] Ba giá trị phát **2 Hz kể cả khi chưa arm, ch8 xuống, `offboard_switch_channel = -1`**
      — Pi cần phân biệt "FC chưa phát" với "FC báo không có quyền" [THOẢ THUẬN — FC đã hiện
      thực; đã thấy trên target lúc khoá]. *Pi đo 09-13: 2,00 Hz mỗi tên khi chưa arm, ở trạng
      thái TAT (`OB_STATE=1, OB_AUTH=1, OB_EXIT=1`). Chưa đo lúc ch8 xuống và lúc `= -1`.*
- [x] `FC_CTR_VER = 10100` — Pi đo 09-13, cả `pymavlink` lẫn MAVROS
- [ ] Gạt ch8 xuống rồi lên → `STATUSTEXT` `OFFBOARD: TAT (...)` severity `NOTICE`
- [ ] `OB_AUTH` hạ xuống 0 **trong vòng 0,5 s** sau khi người lái chạm cần
- [x] Node restart giữa chừng vẫn biết đúng trạng thái trong 0,5 s — *Pi đo 09-13, 10 lần tạo node
      mới trong khi MAVROS chạy:* **sau khi DDS nối xong, đủ 3 tên trong ≤ 0,43 s** (đạt, đúng chu
      kỳ 2 Hz). Thời gian DDS nối node mới thêm 0,11–1,10 s — **nằm phía Pi, không phải FC**; node
      Pi phải coi "chưa có gói" là "không biết" trong khoảng đó (bảng 6.3 dòng cuối)

### 12.B — Cần cấp điện tầng công suất động cơ

**Không thực hiện, không đề xuất, cho tới khi có điện động cơ.**

- [ ] Arm bằng công tắc RC → `/mavros/state` đổi `armed: true`
- [ ] Hợp đồng ACK đường ARM: ch8/ch5 chưa lên → `DENIED (2)`; có quyền nhưng ga chưa giữa
      → `TEMPORARILY_REJECTED (1)`; đủ điều kiện → `ACCEPTED (0)`; sau khi người lái chạm
      cần → cả ARM lẫn DISARM đều `DENIED`
- [ ] `EXTENDED_SYS_STATE` chuyển `ON_GROUND` ↔ `IN_AIR` qua ngưỡng 0,5 m
- [ ] Vào OFFBOARD theo đúng trình tự mục 6.2; rơi ra → `KHOA`, **không tự phục hồi**
- [ ] Cắt setpoint 500 ms → FC phanh, treo, lùi **POSHOLD**, `OB_EXIT = 3`
- [ ] Đang OFFBOARD, gạt ch6 rời POSHOLD → `KHOA`, `OB_EXIT = 9`
- [ ] Pi gửi ARM khi ch6 chưa ở POSHOLD → `TEMPORARILY_REJECTED (1)`
- [ ] Tune PID trong `control.yaml` (**toàn bộ gain hiện là `0.0`, cố ý**) — **tune trong
      Gazebo trước, không bao giờ tune lần đầu trên drone thật**
- [ ] Hạ cánh chính xác theo AprilTag (phụ thuộc 11.3 P1 và `LANDING_TARGET` phía FC)

---

## Phụ lục A — Tra cứu nhanh

| Câu hỏi | Trả lời | Mục |
|---|---|---|
| Pi gửi gì xuống? | vận tốc, `SET_POSITION_TARGET_LOCAL_NED`, 10–20 Hz | 1, 5.2 |
| `type_mask`? | **`0x07C7`** | 5.2 |
| `coordinate_frame`? | **8** (`BODY_NED`) | 5.2 |
| Node nào publish setpoint? | **chỉ** `position_controller_node` | 5.3 |
| `vy` dương là gì? | sang **phải** trên dây; node ROS publish FLU, MAVROS tự đổi | 3.1–3.2 |
| Pi arm được không? | **được**: ga giữa, ch6 POSHOLD, rồi ch8 lên, rồi ch5 lên | 6.2 |
| Gửi `SET_MODE`? | **không** — chưa hiện thực, không báo lỗi | 5.1 |
| Vào OFFBOARD? | công tắc RC + ch6 ở POSHOLD, **và** Pi phải phát setpoint **trước** | 6.3 |
| Pi còn quyền không? | **`OB_AUTH`**, không phải `custom_mode`, không phải `RC_CHANNELS` | 6.3 |
| Bit flow tắt thì bỏ gì? | `vx,vy,x,y` + `groundspeed`. **GIỮ** `z,vz,relative_alt,alt,climb` | 4.2a |
| `battery percentage < 0`? | "không biết" — **không** so ngưỡng | 4.3 |
| Thêm bản tin mới? | 8 bước mục 10.3, ghi sổ 9.1 trước | 10.3 |

Lệnh đo nhanh: xem mục 12.A1.

## Phụ lục B — Liên kết

- Đặc tả bản tin chuẩn: <https://mavlink.io/en/messages/common.html>
- Ánh xạ plugin ↔ topic MAVROS: <https://wiki.ros.org/mavros>
- Cấu hình phía Pi: `src/drone_bringup/config/mavros.yaml`
- Sổ theo dõi việc: `viec_can_lam_mavlink.md`
- Kiến trúc node ROS 2: `thiet_ke_kien_truc_node_ros2.md`, `ke_hoach_thiet_ke_node.md`
- Lớp bản tin phía FC (repo riêng): `Mavlink/mav_link.c`, `Mavlink/mav_port.c`,
  `State/fc_state.h`, `Control/ctrl_offboard.c`, `Control/ctrl_poshold.c`,
  `Control/arming.c`, `Estimator/estimator.c`

---

## Lịch sử phiên bản

| Phiên bản | Ngày | Thay đổi |
|---|---|---|
| 1.1 *(FC trả lời #9 #10, không tăng số)* | 2026-09-13 | **FC trả lời 11.1 #9:** thứ tự byte `flight_custom_version` đổi sang `uint64` little-endian từ firmware 1.2 để MAVROS/GCS in đúng hash; phân biệt bằng `FC_CTR_VER` (9.5). **FC trả lời 11.1 #10:** chuyển [CHỐT] không tăng số; MINOR tăng khi firmware bắt đầu phát thứ mới, đồng thời ở firmware và tài liệu; sửa 10.3 bước 8. Đợt A–E sẽ là 1.2 (`FC_CTR_VER = 10200`). |
| 1.1 *(Pi nghiệm thu, không tăng số)* | 2026-09-13 | **Phía Pi đo bản FC 1.1** trên dây (`pymavlink` 40 s) và qua MAVROS. [CHỐT]: `DISTANCE_SENSOR` 20 Hz, `AUTOPILOT_VERSION` + `REQUEST_MESSAGE` 512, 4 tên `NAMED_VALUE_INT` @2 Hz, `FC_CTR_VER = 10100`; 11.1 #1, #6 xong. Băng thông đo 9,8 %. Bật plugin `distance_sensor` (topic `/mavros/mtf01p`, 8.3); `Range.variance` luôn 0. Kiểm plugin `odometry` bằng FC giả qua UDP: đổi hệ và covariance đúng, nhưng MAVROS **bỏ qua `frame_id`/`child_frame_id`** và không chuyển `quality` (11.2). Câu hỏi mới: thứ tự byte git hash (11.1 #9), có tăng MINOR khi chuyển [CHỐT] không (11.1 #10); P3 chưa chốt. **Phản hồi đề xuất FC → [THOẢ THUẬN]:** `OB_ARM_RDY`/`OB_ARM_BLK`, `OB_T_*`/`OB_RX_*` (kiểm `named_value_float` bằng FC giả), `FC_DIRTY` (đăng ký 9.3), cách điền `ODOMETRY` và `RC_CHANNELS` (kiểm `rc_io` bằng FC giả), không hành động khi mất heartbeat (#8), điều kiện từ kế (10.6a). Nghiệm thu 12.A4 "node restart": ≤ 0,43 s sau khi DDS nối. Thêm nợ P6, P7; thứ tự đề nghị đợt FC tiếp theo (11.2). Sửa lỗi hợp nhất: 3 dòng bảng suy diễn 6.3 bị tách khỏi bảng. |
| 1.1 *(hợp nhất hai bản, không tăng số)* | 2026-09-13 | **Hợp nhất ba chiều** bản FC (repo firmware) với bản Pi (repo ROS), gốc chung 1.0. Từ bản Pi giữ: `sysid`/`compid` Pi = `1`/`191` kèm cách đọc, dòng heartbeat Pi, người tiêu thụ `SYS_STATUS`/`ATTITUDE` (8.1), quy tắc launch MAVROS đã sửa (8.3), 11.1 #4 xong, câu hỏi #6 #7, 12.A2 `ros2 launch` chạy được. 3 xung đột phân giải bằng tay (mục 2, mục 7, bảng 11.1). Sửa câu "heartbeat Pi đáp ứng timeout 3000 ms của FC" cho khớp 11.1 #8. **Chốt bản gốc duy nhất ở repo ROS** (0.2). |
| 1.1 *(bổ sung, không tăng số)* | 2026-09-13 | Trả lời phản hồi Pi: `sysid`/`compid` Pi = `1`/`191`, FC không lọc nguồn (11.1 #6); ba mâu thuẫn + hai điểm nhỏ (11.1 #7) — (a), (b) đã sửa từ 1.1, viết lại quy tắc `NAMED_VALUE_*` (9.1), sửa dòng TAT (6.3). Phát hiện: timeout heartbeat không được dùng (11.1 #8); `OB_AUTH = 1` mà ARM vẫn `DENIED` (6.2). Đề xuất: `OB_ARM_RDY`/`OB_ARM_BLK`, `OB_T_*` + `OB_RX_*` để Pi tự kiểm dấu (bỏ đề xuất `POSITION_TARGET_LOCAL_NED` vì không kiểm được dấu), một bản gốc duy nhất (0.2). |
| **1.1** | 2026-09-13 | **MINOR.** Thêm điều kiện vào OFFBOARD và ARM từ Pi: ch6 phải ở POSHOLD (5.2, 6.2, 6.3); thêm mã `OB_EXIT = 9 CHE_DO`; đổi tên `CONTRACT` → `FC_CTR_VER`. FC hiện thực việc 11.2 #1–#3 (`NAMED_VALUE_INT`, `STATUSTEXT`, `AUTOPILOT_VERSION` + 512, `DISTANCE_SENSOR`). Sửa theo phép đo: lỗi FC loại mask `0x07C7` (5.2); trả lời 11.1 #3, #5; mặc định `offboard_switch_channel = 7`; phạm vi mất quyền (6.2); lối ra `TAT` vs `KHOA` (6.3); `vz`/`yaw_rate` khi mất flow. Đề xuất cách điền `ODOMETRY`, `RC_CHANNELS` (11.2), điều kiện thật cho cổng từ kế (10.6a), cờ `FC_DIRTY` (9.5). |
| **1.0** | 2026-09-13 | Bản hợp nhất đầu tiên. Gộp 5 tài liệu nguồn; giải quyết 3 mâu thuẫn (`type_mask`, hợp đồng ARM, `custom_mode = 4`); thêm sổ đăng ký (mục 9) và quy tắc mở rộng (mục 10) vốn chưa tài liệu nào có. |
