# Kế hoạch: camera nghiêng bằng servo + gắp hàng khi treo

2026-10-22: user đã duyệt và trả lời mục 7; **đã triển khai** (mục 8). Thay phần tiếp cận cổng G (mục 2b) và trình tự hạ cánh bằng
tag nhỏ trong `docs/ke_hoach_huong_bay_hai_tag.md`.

## 0. Yêu cầu của user (2026-10-22)

1. Cất cánh tại home như cũ: camera **0°** (nhìn thẳng xuống).
2. Bay tới đích: quay mũi về đích như cũ, camera **20° về mũi**.
3. Tới gần đích (hoặc thấy tag đích từ xa và đã đủ gần): camera về **0°**. **Bỏ cổng G** (`PAD_ALIGN`).
4. Camera 0° thấy tag to → căn tâm, **quay mũi về đúng hướng bãi**, hạ **thẳng đứng** với tốc độ như cũ.
5. Tag nhỏ **dán trên nóc hộp hàng**, đặt **lệch cố định về phía "dưới"** tag to (khai trước), không còn
   cùng mặt phẳng/độ cao với tag to.
6. Lấy hàng: hạ tới khoảng **0,5 m** vẫn bám tag to, đồng thời tìm tag nhỏ. Thấy tag nhỏ → camera
   **20° về đuôi**, bám tag nhỏ, hạ tới **ToF 0,20 m** (ToF chiếu xuống đất, không chiếu vào hộp),
   cơ cấu gắp ở tâm drone → gắp.
7. Gắp xong → camera 0° → bay lên → quay mũi về nơi thả → camera 20° về mũi → bay tiếp.
8. Thả hàng: **treo rồi thả**, không chạm đất.
9. Sửa **cả mô phỏng Gazebo** (camera có servo).

## 1. Số liệu servo camera (đo 2026-10-22, tag 36h11 ID 19)

- Chiều: **tăng µs → camera ngóc về mũi**.
- Phi tuyến, dùng bảng tra (góc về mũi, + là về mũi):

| Góc | −15°  | 0°   | +10° | +20° | +30° | +40° |
| --- | ----- | ---- | ---- | ---- | ---- | ---- |
| µs  | ~1310 | 1514 | 1642 | 1762 | 1871 | 1970 |

- Lặp lại khi tiến tới **từ cùng một phía**: ≤ 0,5°. Rơ khi đổi phía: **~1,9°** → Pi luôn tiến tới đích
  từ phía µs thấp (vượt xuống 100 µs rồi lên), giống lúc hiệu chỉnh.
- 0° tuyệt đối còn sai ±1,4° (hai chỗ đặt tag lệch mặt phẳng) — đo lại trên mặt đã kiểm thăng bằng.
- **Chưa đo phía đuôi quá −16°**: −20° về đuôi ngoại suy ≈ **1260 µs** — phải đo trước khi dùng (M2).

## 2. Hình học (camera 640×400, fy 298,8, cy 180,2; lắp 9 cm trước tâm)

Nửa FOV dọc: 31° về một phía ảnh, 36° về phía kia. Camera 0°, độ cao _h_: mặt đất thấy được khoảng
**±0,6·h** quanh điểm dưới camera (0,5 m → ±0,3 m; 2,5 m → ±1,5 m).

**Chuyển camera 20° → 0° khi tới gần đích:** ở độ cao bay _h_, camera 0° chỉ còn thấy tag to khi tâm
drone cách tag ≲ 0,5·h ngang. Đề xuất chuyển khi **khoảng cách ngang ≤ 0,5·h** (2,5 m → 1,25 m), hoặc
sớm hơn nếu đã thấy tag đích và dự báo tag vẫn trong khung sau khi về 0°.

**Lúc gắp (ToF 0,20 m):** camera cao khoảng 0,20 m so với đất; nóc hộp cao _H_; tag nhỏ ở ngay dưới
tâm drone = 9 cm sau camera, sâu _d_ = 0,20 − _H_. Camera nghiêng 20° về đuôi:

| Cao hộp _H_ | _d_   | Tag 10 cm nằm trong góc | Trong khung (−11° … +56° quanh đứng)?     |
| ----------- | ----- | ----------------------- | ----------------------------------------- |
| 5 cm        | 15 cm | 15° … 43°               | **có, thoải mái**                         |
| 8 cm        | 12 cm | 19° … 49°               | có, sát mép                               |
| 10 cm       | 10 cm | 22° … 55°               | **sát/ra mép** — cần tag nhỏ hơn (6–8 cm) |

→ Cần biết **cao hộp** để chốt cỡ tag nhỏ (M1).

## 3. Trình tự một chặng

```
TAKEOFF (cam 0°, neo tag home)
 → ENROUTE: quay mũi tại chỗ về đích, cam 20° mũi, bay tới
 → [ngang ≤ 0,5·h hoặc thấy tag đích đủ gần] cam 0° (chờ servo ổn định)
 → MARKER_SEARCH → căn tâm trên tag to
 → HEADING_ALIGN (MỚI): treo trên tâm, quay mũi = hướng bãi
 → PRECISION_LAND: hạ thẳng đứng, tốc độ cũ, bám tag to
     ├─ DROPOFF: tới ToF 0,20 m → thả → lên
     ├─ PICKUP: tới ToF ~0,5 m, giữ độ cao, bám tag to + tìm tag nhỏ
     │    → BOX_ALIGN (MỚI): thấy tag nhỏ → cam 20° đuôi → bám tag nhỏ (tâm drone trên tâm tag nhỏ)
     │      → hạ tới ToF 0,20 m → gắp (chờ xác nhận) → cam 0° → lên
     └─ Điểm cuối (home, ACTION_NONE): hạ chạm đất + disarm như cũ (cam 0°, chỉ tag to)
 → TAKEOFF/leo → ENROUTE chặng sau
```

Ghi chú:

- Camera 0° nhìn thẳng xuống nên tag to 25 cm thấy được tới ~0,25 m → **home hạ cánh chỉ bằng tag
  to**, không cần tag nhỏ ở home.
- Mất tag giữa chừng: giữ đúng các quy tắc hiện có (không hạ mù trên ngưỡng, có giới hạn thử lại).
- Lúc servo đang quay + thời gian ổn định (~0,6 s): **bỏ qua pose tag và optical flow** (ảnh nhòe,
  TF chưa đúng góc thật).

## 4. Gói việc

| #   | Việc                                                                                                                                                                                                 | Ghi chú                                                             |
| --- | ---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- | ------------------------------------------------------------------- |
| A   | **TF camera động**: node phát `base_link → camera_link` theo góc servo đã lệnh (bảng tra), kèm cờ "đang quay"                                                                                        | Thay static TF ở `estimation.launch.py:72` (đang ghi cứng 20°)      |
| B   | **optical_flow_node** tra TF mỗi khung (đang cache một lần) + bỏ khung khi servo quay                                                                                                                | Rủi ro lớn nhất: flow sai lúc đổi góc                               |
| C   | **Lệnh góc camera từ FSM**: `Action.camera_tilt_deg` → bridge `~/servo` (lệnh 183, servo 2), đổi độ → µs + chống rơ                                                                                  | Code bridge `~/servo` đang có, chưa commit                          |
| D   | **FSM**: bỏ `PAD_ALIGN`/cổng G khỏi luồng; thêm `HEADING_ALIGN`, `BOX_ALIGN`; ENROUTE đặt cam 20°; chuyển cam 0° theo khoảng cách; PICKUP/DROPOFF treo ở ToF 0,20 m                                  | Cập nhật `test_mission_fsm.py`                                      |
| E   | **Tag nhỏ trên hộp**: `tags.yaml` đổi `pad_small_tag_forward_m` → độ lệch về phía **dưới** + thêm `pad_small_tag_height_m` (cao hộp); `marker_pose_republisher` + `landing_target_bridge` dùng z hộp | Tag nhỏ không còn dùng neo EKF (hộp xê dịch được?) — xem câu hỏi Q3 |
| F   | **Giao ước GCS**: 2 trạng thái mới → `MissionState.msg` + enum `DRONE_MISSION_STATE` + GCS                                                                                                           | `PAD_ALIGN` giữ giá trị nhưng không còn dùng                        |
| G   | **Mô phỏng**: khớp servo camera trong Gazebo, mô hình hộp có tag nhỏ, `sim_tag_node` dùng TF động                                                                                                    | Chạy trọn nhiệm vụ home → A (gắp) → B (thả) → home                  |
| H   | **Firmware FC**: không đổi (lệnh 183 đã có)                                                                                                                                                          | Chỉ cần đặt `servo_cam_min/max_us` hợp lý (đang 500/2500)           |

## 5. Cần user cung cấp / đo

| #   | Việc                                                       | Dùng cho                          |
| --- | ---------------------------------------------------------- | --------------------------------- |
| M1  | **Chiều cao hộp** và **cỡ tag nhỏ** (vẫn 10 cm?)           | Hình học mục 2, cấu hình apriltag |
| M2  | Đo servo phía đuôi tới −25° (tag đặt sau drone)            | µs cho "20° về đuôi"              |
| M3  | **Độ lệch tâm hộp so với tâm tag to** (m, về phía "dưới")  | `tags.yaml`                       |
| M4  | Vị trí ToF so với tâm drone, vị trí cơ cấu gắp (đúng tâm?) | Căn tâm khi gắp                   |

## 6. Câu hỏi còn mở

- **Q1** Ngưỡng chuyển camera 20° → 0°: dùng đề xuất "ngang ≤ 0,5·h" được không?
- **Q2** Lúc lấy hàng hạ tới "khoảng 0,5 m" rồi tìm tag nhỏ: không thấy trong bao lâu thì coi là thất
  bại (đề xuất 10 s, dùng chung `max_retries`)?
- **Q3** Hộp đặt tay mỗi lần nên có thể lệch vài cm: căn tâm theo **pose tag nhỏ đo được** (không dùng
  vị trí khai báo), và tag nhỏ **không** dùng để neo EKF — đồng ý?
- **Q4** ID tag nhỏ vẫn = ID tag to + 10?

## 7. User cung cấp thông tin thêm.

- **M1** hộp 12x12x12cm tag 10cm
- **M2** lấy thông số từ các phép đã đo
- **M3** tag nằm trên hộp sẽ cách tâm tag to khoảng 30cm về phía dưới. khi drone đang bám tag to thì hộp sẽ nằm ngay khoảng phía dưới bụng drone
- **M1** vị trí tof lệch cách tâm một khoảng 10cm về phía đuôi và nằm trên trục roll
- **Q1** Ngưỡng chuyển camera 20° → 0°: dùng đề xuất "ngang ≤ 0,5·h" được không?. đồng ý
- **Q2** Lúc lấy hàng hạ tới "khoảng 0,5 m" rồi tìm tag nhỏ: không thấy trong bao lâu thì coi là thất
  bại (đề xuất 10 s, dùng chung `max_retries`)?. đồng ý
- **Q3** Hộp đặt tay mỗi lần nên có thể lệch vài cm: căn tâm theo **pose tag nhỏ đo được** (không dùng
  vị trí khai báo), và tag nhỏ **không** dùng để neo EKF — đồng ý?. đồng ý nhưng sau khi dùng tag nhỏ để căn tâm và gắp hàng xong vẫn cần phải sử dụng tag lớn để neo lại ekf và bay đến nơi thả.
- **Q4** ID tag nhỏ vẫn = ID tag to + 10?. đồng ý

## 8. Trạng thái triển khai (2026-10-22)

| Gói | Trạng thái |
| --- | --- |
| A | `camera_tilt_node` + `camera_tilt.py` (bảng tra trong `mission.yaml`, chống rơ, `/camera/tilt` NaN khi đang quay); thay static TF 20° trong `estimation.launch.py` |
| B | `optical_flow_node` tra TF mỗi khung, bỏ khung khi servo quay |
| C | `Action.camera_tilt_deg` → `/mission/camera_tilt` |
| D | FSM: bỏ `PAD_ALIGN`/cổng G (xoá `approach_guidance.py` + 2 file test); dùng `ALIGN_HEADING` (11), `FINAL_APPROACH` (13) có sẵn trong giao ước 0.8; gắp/thả khi treo ToF 0,20 m; gắp xong phải thấy lại tag to |
| E | `tags.yaml`: tag nhỏ −0,30 m, cao 0,12 m; tag nhỏ không neo EKF; bridge bám thẳng tag hộp |
| F | Không đổi enum. **Còn lại:** ý nghĩa mới của 11/13 chưa ghi vào `GIAO_UOC_GCS_PI.md` / dialect |
| G | Gazebo chế độ tag tổng hợp (`camera:=false`): servo + hộp; chạy trọn `sim_pick_drop` trên ubt. **Còn lại:** chế độ camera render (`camera:=true`) chưa có servo — camera Gazebo vẫn cố định |

**Cập nhật 2026-10-22 (đo tag trên hộp thật, user chốt):** gắp khi nắp hộp **ngang chân drone** → `GRIP_HOVER_M` 0,20 → **0,27 m** (chân 0,155 − mặt ToF 0,009 + hộp 0,12; thả cũng ở mức này). Bỏ hạ mù: xuống theo tag hộp ở −20° tới ToF `box_cam_switch_m` = **0,32 m**, dừng, nghiêng camera **−36° (1100 µs)** rồi bám tag tiếp tới ToF 0,27 m. Đo thật: tag cách ống kính 9,3 cm đọc được 1150..1050 µs; 1100 µs ở giữa. Đo trên tag hộp (chưa kiểm thăng bằng): 1244 µs = −24°, 1100 µs = −36° → bảng phía đuôi còn lệch ~4°.
Chưa đo: servo phía đuôi −20° (1244 µs là ngoại suy).
