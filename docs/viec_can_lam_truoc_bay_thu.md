# Việc cần làm trước khi bay thử gắp hàng thật

Cập nhật 27/10/2026. Tài liệu sống: làm xong mục nào thì đánh dấu `[x]` và ghi ngày + kết quả đo.

## 1. Hiện trạng (27/10/2026)

### Phần cứng
| Hạng mục | Trạng thái |
|---|---|
| Khung | Version2.0, tay đòn giàn phẳng 8 mm, X2212 980KV + cánh 8x4.5 |
| Cơ cấu kẹp | MG90 giữa + 2 thanh CauNoi + 2 hàm GapHang — **đã lắp xong** |
| Xung kẹp (đo 26/10) | **500 µs = mở hết cỡ, 2500 µs = kẹp vào** |
| Camera | OV9281 trên servo nghiêng; bảng góc đo 22/10 chỉ trong 1100–2050 µs |
| Khối lượng | Ước ~1,5 kg (từ ga treo cũ) — **chưa cân lại** sau khi lắp kẹp |

### FC (FCH743_V1.0)
| Hạng mục | Trạng thái |
|---|---|
| Firmware trên bo | **`git 8c9698d`** (nạp 26/10, kiểm lại 27/10 qua `version`) |
| Tham số | 176 tham số, `table_crc 0xDD307FBD`, **save seq 7** — 27/10 đọc lại khớp 100 % bản sao lưu 26/10 |
| Servo | `servo_grip_open_us=500`, `close_us=2500`, `cam_min/max=500/2500`, kênh RC CH11/CH12 |
| Bay tay | ALTHOLD (07/10), POSHOLD (09/10), vòng vận tốc kp 16 ki 3 (10/10) đã bay thật |
| `althold_hover_thr` | 0,43 (bay 09/10 đo ~0,49, chưa có kẹp + hộp) |

### Pi 5
| Hạng mục | Trạng thái |
|---|---|
| Code | `~/ros2_ws` **`main` `7733b0c`** (pull + build 27/10), `drone-startup.service` khởi động lại 07:52 27/10 — 41 node, MAVROS `connected` |
| `gripper_controller_node` | `simulate: true` — không gửi lệnh kẹp xuống FC (giữ tới bậc 6) |
| Đường lệnh Pi → FC | **Đã kiểm 26/10**: `MAV_CMD_DO_GRIPPER` qua MAVROS → ACCEPTED, kẹp mở / đóng đúng; ACK khứ hồi 5 ms |
| Tải | 27/10: load ~6,5, 66–68 °C, `throttled=0x0`, đĩa trống 13 GB; EKF thỉnh thoảng "Failed to meet update rate" (~1 lần/30 s) |
| EKF Pi trên bàn | `healthy: false` (phương sai vị trí 195 m², `anchored: false`) — bình thường khi chưa thấy tag home / chưa có fix GPS; phải `healthy` trước OFFBOARD |
| Bảo mật | `/etc/sudoers.d/90-pc-nopasswd` vẫn còn (tạm từ 25/09) |

### Phần mềm (`main` = `kep-servo-moi` = `7733b0c`, đã đẩy GitHub 27/10)
- Camera nghiêng bằng servo, gắp / thả khi treo, không hạ cánh ở bãi thả.
- Bãi có hướng: thấy tag to là vào thẳng điểm hộp; gắp hạ tìm tag hộp; thả hạ neo tag to.
- Đo tag tại đúng lúc chụp ảnh (TF nội suy), tâm hộp trong khung thế giới.
- Lúc gắp giữ vị trí bằng EKF tại tâm hộp, PID `grip` riêng (kp 1,5 — **chỉ chỉnh trong Gazebo**).
- Chỉ kẹp khi vị trí dự đoán lúc hàm khép xong (vị trí + vận tốc × 0,4 s) lệch ≤ 3 cm.
- Không kẹp mù / không thả mù: lệch quá ngưỡng thì bay lên làm lại (tối đa 2 lần) rồi hạ cánh.

### Mô phỏng (Gazebo trên ubt, 25/10)
Lặng gió, gió 4 m/s hướng 120° và 200° (giật ±35 % tốc độ, ±25° hướng) đều **gắp – thả – về home**.
Gió 200° cần 3 lần thử mới gắp được. FC giả **không có trễ vận tốc** nên kết quả lạc quan hơn thật.

## 2. Việc chặn bay — phải xong trước

### 2.1 Phần mềm và triển khai
- [x] Gộp `kep-servo-moi` vào `main`, đẩy GitHub `7733b0c` (27/10).
- [x] Pi: pull + `colcon build` (9 gói) + pytest **371 đạt** (mission 158, control 43, sim 38, estimation 49,
      perception 31, comms 31, safety 21) — 27/10; khởi động lại dịch vụ, đủ node.
- [ ] Đặt `gripper_controller_node.simulate: false` trong `config/mission.yaml` — **chỉ cần từ bậc 6**
      (bậc 1–5 kẹp do núm RC CH11 điều khiển trực tiếp trên FC).
- [x] Firmware FC: nạp `8c9698d` (có UART RX DMA tự chữa `17367b5`) 26/10, sao lưu tham số trong
      `FCH743_V1.0/App/Config/param_backup/2026-10-26_*`; cấu hình giữ nguyên (27/10 đọc lại khớp).

### 2.2 Đo và hiệu chuẩn trên bàn (tháo cánh)
- [ ] Cân drone đủ đồ (kẹp + pin) và khi mang hộp 120 g.
- [x] Đo hành trình kẹp: user kiểm 26/10 — đúng như hiện tại (500 mở hết cỡ, 2500 kẹp vào).
- [ ] Hàm kẹp mở hết cỡ có chạm đất khi drone đứng trên chân không? Hộp 120 mm có lọt giữa hai hàm mở không?
- [x] Bảng góc camera: user chốt 26/10 GIỮ bảng đo 22/10 (1100–2050 µs, `mission.yaml`); FC `servo_cam_min/max_us` 500/2500 chứa trọn bảng.
- [x] TF camera: từ 22/10 `camera_tilt_node` phát `base_link → camera_link` theo góc servo (trục servo + ống kính từ URDF);
      đối chiếu URDF 25/10: trục servo khớp, hướng ống kính vuông góc trục servo (không lệch ngang). `main` cũ còn TF tĩnh 20° — hết khi gộp.
- [ ] Chạy FSM trên bàn (tháo cánh, có quyền Pi): chuỗi mở kẹp → nghiêng camera → kẹp → camera +20° đúng thứ tự.
- [ ] Kiểm laser ToF đọc nắp hộp khi hộp dưới drone (bù `box_height` của `range_vertical_node`).

### 2.3 An toàn
- [ ] Nhớ quy trình mở quyền: sau mỗi lần FC bật nguồn phải gạt **ch8 xuống – lên** (`OB_EXIT 1 KHOI_DONG`) — chỉ cần từ bậc 4 (OFFBOARD).
- [ ] Người lái tập lấy lại quyền: gạt ch8 xuống / chạm cần → FC về POSHOLD.
- [ ] Bãi thử trong nhà có lưới hoặc dây buộc; khu vực không người; pin đầy, có pin dự phòng.
- [ ] Kiểm `offboard_max_alt_m 5`, `offboard_max_vel_mps 2` phù hợp phòng thử.

## 3. Bay thử theo bậc — chỉ lên bậc sau khi bậc trước đạt

| Bậc | Nội dung | Tiêu chí đạt |
|---|---|---|
| 1 | Bay tay POSHOLD **có cơ cấu kẹp**, không hộp | treo ổn ±10 cm, không dao động pitch; ghi ga treo |
| 2 | Bay tay POSHOLD **có hộp 120 g kẹp sẵn** | như bậc 1; ga treo < 0,6; log rung không tăng rõ |
| 3 | Cập nhật `althold_hover_thr` theo bậc 1–2 | `save`, bay lại ALTHOLD giữ cao ±5 cm |
| 4 | OFFBOARD: cất cánh – treo – hạ theo tag home | như các lần bay 09–11/10 |
| 5 | OFFBOARD treo trên hộp ~0,5 m, **chưa kẹp**, bật PID `grip` | không dao động; lệch EKF ≤ 5 cm; nếu dao động hạ `grip.*.kp` về 0,7 |
| 6 | Một lần gắp ở độ cao thấp, người lái sẵn sàng ch8 | kẹp được, nhấc hộp, giữ cao khi mang tải |
| 7 | Thả tại bãi B (không hạ cánh) | hộp chạm đất trước khi nhả, cách tâm điểm hộp ≤ 5 cm |
| 8 | Nhiệm vụ trọn vẹn home → A → B → home | hoàn thành, log đầy đủ |

Mỗi lần bay: ghi rosbag + log QSPI của FC, rút gọn kết quả vào `nhat_ky_lam_viec.md`.

## 3a. Chuẩn bị ngày bay bậc 1–3 (bay tay, Pi chỉ ghi log)

Pi và FC đã sẵn (27/10): code mới chạy, MAVROS nối FC, rosbag tự ghi vào `~/drone_logs/` (bỏ ảnh camera),
tham số FC đúng bản đã bay 09–10/10 + servo. Ở bậc 1–3 Pi **không** điều khiển: giữ **CH8 (OFFBOARD) ở dưới** suốt chuyến.

Kênh RC theo tham số FC (đếm từ 1): CH5 arm, CH6 chế độ (ALTHOLD ≥ 582 µs, POSHOLD ≥ 1401 µs), CH7 log QSPI,
CH8 OFFBOARD, CH11 kẹp, CH12 camera.

Trước khi cất cánh:
1. Gắn cánh đúng chiều, siết đai ốc; pin đầy, có pin dự phòng; dây pin không chạm cánh / cơ cấu kẹp.
2. Bật RC trước, cắm pin sau; để drone đứng yên trên mặt phẳng tới khi FC hiệu chuẩn IMU xong.
3. Kiểm trên GCS: hết `mat RC`, điện áp pin đúng; Pi `ros2 topic echo /mavros/state` → `connected: true`.
4. Núm CH11 (kẹp): vặn về **kẹp vào** (2500 µs) để hàm không chạm đất / không quét trước camera.
5. CH7 (log QSPI) bật; CH6 ở ALTHOLD/POSHOLD; CH8 ở dưới.
6. Đứng cách drone ≥ 3 m, khu vực có lưới / không người.

Bậc 1: arm → ALTHOLD lên ~1 m → POSHOLD treo 30–60 s → ghi ga treo (GCS / log). Hạ ngay nếu rít / rung tăng,
pitch lắc hay độ cao vọt: về ALTHOLD rồi hạ ga hạ cánh; chỉ tắt arm (CH5) trên không khi mất kiểm soát.

Sau bay: rút log QSPI qua USB CDC, chép bag mới nhất trong `~/drone_logs/` về PC; ghi kết quả vào `nhat_ky_lam_viec.md`.

## 4. Rủi ro đã biết — theo dõi trong mọi lần bay

- **Dao động pitch ở ga cao** (nguyên nhân rơi 23/09: ga 0,85 → accel sai → EKF độ cao phân kỳ). Mang tải làm ga tăng.
- **Cộng hưởng khung pitch ~245 Hz** (đo 29/09); tay đòn giàn 8 mm yếu.
- **Không có cảm biến xác nhận kẹp**: kẹp hụt thì FSM vẫn bay đi thả. Theo dõi bằng mắt; cân nhắc thêm đo dòng servo hoặc công tắc hành trình.
- **Optical flow có thể về 0 Hz** khi chất lượng ảnh thấp (q = 0,3, hoãn từ 08/10) — POSHOLD và EKF ngang phụ thuộc.
- **PID `grip` kp 1,5 chưa thử với FC thật** (trễ vận tốc 1–1,5 s).
- **Pi 5 tải nặng**: theo dõi nhiệt độ và tần số apriltag lúc bay; quá 80 °C sẽ hạ xung.

## 5. Nợ không chặn bay trong nhà

- [ ] Hiệu chuẩn la bàn IST8310 — 27/10 đã nạp FC `4044b72` (`MAG_I2C_USE_GPS_MAG 1`, cấu hình giữ nguyên seq 7,
      sao lưu `param_backup/2026-10-27_truoc_nap_ist8310_toan_bo.txt`); IST8310 chạy (~30 Hz qua MAVROS) nhưng
      bộ số `mag_*` đang là của QMC → |B| ~0,88 G, **yaw FC sai — KHÔNG bay tới khi hiệu chuẩn xong**.
      27/10 09:43 hiệu chuẩn ngoài trời bằng rosbag trên Pi (không cắm dây): |B| 0,496 G ± 0,73 %, chúc +32,4°,
      trục MỚI `MAP 1,0,2 / SIGN -1,+1,+1` (module GPS quay 90° so với 22/09); đã `save` seq 8 + reboot, |B| 0,50 G.
      Kiểm hướng ngoài trời (từ bag): B +2..+5°, Đ +3,5°, N +9,5°, T +11,7°; trong nhà 10:08 lệch +22° so với điện thoại
      (yaw FC bám đúng la bàn) → **còn phải kiểm hướng ngoài trời bằng hướng đi GPS** trước khi bay theo GPS.
- [ ] GPS: fix ngoài trời + đo gốc bãi mới (Pi 5 chưa có gốc: `geo_origin_valid: false`).

### 5a. Quy trình ngoài trời: la bàn IST8310 rồi gốc GPS

Mang: laptop + cáp USB vào FC (COM23) **và bộ chuyển USB-UART1 (COM4, 921600)** — luồng `mode 26` của
`mag_cal9.py` chỉ ra UART1, không ra USB. Pin lắp, cánh tháo. Bãi trống, cách xe / cột thép / lưới thép ≥ 5 m.
Pi cần mạng (hotspot) để chạy `gps_survey.py` và nhận bản đồ từ GCS.

La bàn (làm trước — yaw FC là hướng khung bản đồ):
1. `python tools/mag_cal9.py --port COM4 --seconds 90` (thư mục FCH743_V1.0), xoay chậm cả drone mọi hướng,
   cả lật ngửa / dựng đứng, tới khi phủ 26/26; dừng yên 2–3 s ở vài tư thế.
2. Đạt khi: |B| sau hiệu chuẩn ± ≤ 3 %, trục ra `MAP 0,1,2 / SIGN -1,-1,+1` (như 22/09), chúc từ dương ~25–33°.
3. Gửi các dòng `set mag_...` + `save` qua console, rút pin + USB cho FC khởi động lại.
4. Kiểm hướng: đặt mũi drone theo la bàn điện thoại Bắc / Đông / Nam / Tây, đọc yaw FC (GCS hoặc
   `/mavros/imu/data`) — đạt khi lệch ≤ 5° cả 4 hướng.

Gốc GPS (bãi mới):
5. Chờ fix: `fix_type ≥ 3`, ≥ 10 vệ tinh, `h_acc ≤ 1,5 m` (lần 22/09: 13–20 sv, ~1 m sau vài phút).
6. Đặt drone sao cho **module GPS nằm ngay trên tâm tag home**, đứng yên, trên Pi:
   `source ~/ros2_ws/install/setup.bash && python3 ~/gps_survey.py --seconds 180`
   (script trong repo `scripts/gps_survey.py`) → in lat / lon / alt trung bình + độ lệch chuẩn (mong < 1 m).
7. GCS, trang thiết kế khu vực: nhập gốc = số vừa đo, `yaw_offset_deg` (north_yaw) = **−1,8°** — khung bản đồ
   lấy hướng từ yaw FC (Bắc từ, `est_mag_declination_deg 0`) nên north_yaw = độ lệch từ tại chỗ (WMM, như 22/09).
   Bố trí tag ngoài bãi phải theo đúng hướng Bắc từ như trong thiết kế.
8. Gửi bản đồ xuống Pi; kiểm `gps_odom_node` log gốc mới, `/gps/pose_odom` gần (0, 0) khi drone trên tag home,
   cầm drone đi tới tag khác: vị trí GPS lệch vị trí tag ≤ ~1,5 m.
- [x] Mô phỏng hạ khẩn cấp sau khi gắp (không thấy lại tag to / tag 2): LỖI MÔ PHỎNG, sửa 27/10 (`56b1197`) —
      hộp động gắn lệch 7 cm che ống kính. Gió 200° sau sửa: lượt nào gắp được đều thả và về home;
      2/4 lượt từ chối kẹp cả 3 lần (lệch 7–8 cm) rồi hạ cánh — giới hạn giữ vị trí trong gió giật 4 m/s.
- [ ] Gỡ `/etc/sudoers.d/90-pc-nopasswd` trên Pi.
- [ ] Đo đặc tính vòng vận tốc FC thật (trễ + hằng số thời gian) → đưa vào FC giả Gazebo; sau đó cân nhắc
      bộ quan sát nhiễu gió (ESO) và Smith predictor cho vòng vị trí.
