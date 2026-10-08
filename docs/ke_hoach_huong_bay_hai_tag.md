# Kế hoạch: bay quay mũi về đích và hạ cánh bằng hai tag

*Lập 2026-10-08. Trạng thái: **ĐỀ XUẤT** — chưa viết code. Người làm: Pi + GCS (Claude), FC (user).*

## 0. Tóm tắt

Drone cất cánh tại home, neo vị trí bằng tag, **quay mũi về điểm đến** rồi bay tới với camera
(nghiêng 20° về trước) nhìn thẳng hướng đi. Tới nơi, drone căn tâm trên **tag to**, **quay mũi
theo hướng bãi đáp** để **tag nhỏ** (đặt phía "trên" của tag to) nằm trước camera, rồi hạ cánh
bằng tag nhỏ khi tag to đã ra khỏi khung hình. Lấy/thả hàng xong thì lặp lại cho chặng tiếp theo.

### Quyết định đã chốt (user, 2026-10-08)

| # | Quyết định | Hệ quả |
|---|---|---|
| Q1 | Hướng tag đi qua **giao ước GCS** — nâng lên **0.8** | Sửa dialect, CRC, GCS backend + UI, Pi |
| Q2 | Bay **ngoài trời, 3–5 m** | Chặng xa dẫn bằng GPS; hạ xuống độ cao tiếp cận mới bắt tag |
| Q3 | Tag to cỡ **20–25 cm** | Đọc được tới ~3–3,7 m → độ cao tiếp cận ~2,5 m |
| Q4 | ID tag nhỏ = **ID tag to + 10** | home 0→10, a 1→11, b 2→12 |
| Q5 | **Pi tự suy ra** tag nhỏ từ tag to (một mẫu bãi chung) | GCS chỉ khai tag to; CRC chỉ phủ tag to |
| Q6 | **Thêm trạng thái FSM mới** | Sửa `MissionState.msg`, enum `DRONE_MISSION_STATE` (giao ước), GCS |
| — | Hướng Đông/Tây/Nam/Bắc **không** là ràng buộc | Hướng bay = phương vị tới đích tính trên bản đồ tag; mỗi bãi có hướng tuỳ ý |
| — | Sửa yaw FC: **user làm** (2026-10-09) | Tiền đề cho mọi bước có quay mũi |

## 1. Hình học và mẫu bãi đáp

Số đo: camera **90 mm trước tâm**, nghiêng **20°** về trước; `fx 299,9  fy 298,8  cy 180,2`
(640×400). Từ đó:

- Ảnh nhìn từ **16,3° sau** tới **51,1° trước** phương thẳng đứng. Dải mặt đất nhìn thấy theo trục
  trước–sau, tính từ tâm drone, ở độ cao camera *h*: **[0,09 − 0,293·h ; 0,09 + 1,239·h]** m.
- Tag đọc được khi cạnh ≥ ~20 px (`decimate: 2` — xem rủi ro R5): cự ly ≈ 300·cạnh/20.

| Thông số | Giá trị đề xuất | Lý do |
|---|---|---|
| Tag to | **25 cm** (36h11) | Đọc tới ~3,7 m; mất khỏi khung khi *h* < **0,73 m** (20 cm: 3,0 m / 0,65 m) |
| Tag nhỏ | **10 cm**, cùng hướng tag to | Đọc được từ *h* ≈ **1,1 m** trở xuống; còn trong khung tới *h* ≈ 0,15 m |
| Tâm tag nhỏ | **0,22 m** về phía "trên" tag to, tính từ tâm bãi | ≥ 0,206 m để không đè viền trắng tag to (25 cm); vừa dải nhìn thấy ở *h* 0,15–1,1 m |
| Dải chuyển giao | *h* **0,73 → 1,1 m** | Cả hai tag cùng thấy được |
| Hạ mù | dưới *h* ≈ **0,15 m** | Khoảng cuối ngắn, giữ vị trí bằng EKF/flow |
| Độ cao tiếp cận | **2,5 m** | Tag 25 cm còn ~30 px — bắt chắc |
| Độ cao bay xa | 3–5 m (theo kế hoạch GCS) | Dẫn bằng GPS (hacc ~1 m, đo 09-22) |

```
            phía "trên" của bãi (hướng bãi, yaw_pad)
                         ▲
                   ┌──────────┐
                   │ tag nhỏ  │  10 cm, ID = ID_to + 10, tâm cách tâm bãi 0,22 m
                   └──────────┘
          ┌──────────────────────────┐
          │                          │
          │        tag to 25 cm      │  ← drone đậu ở đây, mũi hướng ▲
          │        (tâm bãi)         │
          │                          │
          └──────────────────────────┘
```

**Mọi con số ở bảng trên là tạm** cho tới khi đo M1–M3 (mục 5).

## 2. Quy trình bay mới

Ba trạng thái mới (Q6), nối vào bảng `TRANSITIONS` hiện có:

| Trạng thái | Giá trị | Làm gì | Xong khi |
|---|---|---|---|
| `ALIGN_HEADING` | 11 | Treo tại chỗ (đang neo trên tag), quay mũi về **phương vị tới điểm kế tiếp** | \|lệch yaw\| ≤ 5° giữ 1 s |
| `PAD_ALIGN` | 12 | Đã căn tâm trên tag to ở độ cao tiếp cận, quay mũi theo **yaw_pad** | \|lệch yaw\| ≤ 5° và tag to vẫn thấy |
| `FINAL_APPROACH` | 13 | Hạ từ dải chuyển giao xuống, căn tâm theo **tag nhỏ** (quy về tâm bãi), hạ mù đoạn cuối | chạm đất → `ACTUATE_GRIPPER` |

Chuỗi một chặng (home → A):

```
TAKEOFF (neo bằng tag home) → ALIGN_HEADING (mũi về A) → ENROUTE (bay cao 3–5 m, mũi luôn về A;
gần A thì hạ xuống 2,5 m) → MARKER_SEARCH (bắt tag to A) → PRECISION_LAND (căn tâm ở 2,5 m rồi hạ
xuống ~1,2 m) → PAD_ALIGN (quay theo yaw_pad A) → PRECISION_LAND (hạ tiếp tới dải chuyển giao)
→ FINAL_APPROACH (tag nhỏ, hạ mù < 0,15 m) → ACTUATE_GRIPPER → TAKEOFF → ALIGN_HEADING (mũi về B) → …
```

Các quy tắc giữ nguyên tinh thần FSM hiện tại: mất tag **trên** ngưỡng hạ mù vẫn là một lần thất
bại (về `MARKER_SEARCH`), không hạ mù sớm; mỗi lần thử có giới hạn.

## 3. Gói việc

Thứ tự đề xuất: **WP1 → WP2 → WP3** (bản đồ + giao ước) song song **WP5** (điều khiển yaw, cần
WP0) → **WP4** (EKF yaw) → **WP6** → **WP7** → **WP8** → **WP9**.

### WP0 — Yaw trên FC *(user, 2026-10-09)*

- PID yaw (kp 0,0006 / ki 0,0002 quá yếu — log 10-07), kiểm độ nghiêng motor/tay đòn (cặp chéo
  lệch ~122 DShot).
- **Nghiệm thu:** treo giữ yaw ±3°; bám lệnh `yaw_rate` 30 °/s, sai lệch ≤ 20 %. Pi chưa từng
  gửi `yaw_rate` ≠ 0 — cần thử đường lệnh này ở bước WP5.

### WP1 — Giao ước GCS ↔ Pi 0.8 *(làm tài liệu TRƯỚC code — quy trình mục 6.4)*

1. `DRONE_TAGMAP_ITEM`: thêm **sau `<extensions/>`** trường `int16_t yaw_cdeg` (hướng "trên" của
   tag trong hệ bản đồ, cdeg, chiều theo quy ước ENU của bản đồ) và `uint8_t yaw_valid`
   (R3b: yaw = 0 là hướng thật, nên cần cờ riêng).
2. `tagmap_crc`: bản ghi tag thêm `yaw_cdeg` **chỉ khi cả bản đồ có yaw** (giống cách nối bản ghi
   gốc ở 0.7) → bản đồ cũ giữ nguyên CRC. Cập nhật docstring `tagmap.py` (lý do cũ "yaw vô nghĩa"
   không còn đúng khi yaw đi từ tag).
3. `DRONE_MISSION_STATE`: thêm `ALIGN_HEADING=11`, `PAD_ALIGN=12`, `FINAL_APPROACH=13` (MINOR —
   GCS cũ bỏ qua giá trị lạ theo R1).
4. Tham số mẫu bãi (Q5) **không** đi qua dây: nằm trong `tags.yaml` của Pi. GCS chỉ cần biết để
   vẽ — ghi vào tài liệu.
5. Bảng lịch sử 0.8, sổ 8.1; `contract_ver` 800.

**Nghiệm thu:** XML mới sinh lại dialect hai bên; `tools/gcs_sim.py` nạp bản đồ có/không có yaw,
CRC khớp ở cả hai bên.

### WP2 — Bản đồ tag có hướng trên Pi

- `tags.yaml`: thêm `known_tags_yaw_deg: [...]` song song `known_tags` (giữ định dạng 4 phần tử
  cũ để không vỡ các bộ đọc khác), và khối mẫu bãi:
  `pad_small_tag: {id_offset: 10, size: 0.10, forward_m: 0.22}`.
- Hàm thuần mới (pytest): đọc tag + yaw → **suy vị trí/hướng tag nhỏ** = tâm tag to + xoay
  (0,22 m) theo yaw_pad.
- Sửa các bộ đọc: `estimation_math.parse_known_tags`, `drone_comms.tagmap` (đọc, CRC, ghi
  override kèm yaw), `landing_target_bridge_node`, `telemetry_aggregator_node`.
- `apriltag.yaml`: khai thêm ID 10, 11, 12 (frames, **size 0,10**); đổi size tag to theo bản in
  thật (đo bằng thước — M1).
- `gcs_link_node`: nhận `yaw_cdeg/yaw_valid`, ghi override.

**Nghiệm thu:** pytest cho bộ đọc/CRC/suy tag nhỏ; Pi khởi động với `tags.yaml` mới, CRC phát lên
khớp CRC GCS tính.

### WP3 — GCS backend + giao diện

- `link_mav/tagmap.py`, `tagmap_client.py`, dialect sinh lại, `mission/uploader.py`,
  `sim/fake_pi.py`, kiểm thử e2e.
- Trang thiết kế khu vực: nhập/xoay **hướng bãi**, vẽ mũi tên "trên" và vị trí tag nhỏ theo mẫu.
- Hiển thị 3 trạng thái mới.

**Nghiệm thu:** test backend xanh; nạp bản đồ có yaw tới Pi thật, CRC khớp.

### WP4 — EKF lấy hướng từ tag

1. **Đo trước (M4):** treo/cầm drone trên tag ngoài trời, so yaw IMU (từ kế đã hiệu chuẩn 09-22)
   với yaw suy từ tag. Quyết định theo số đo:
   - lệch ổn định ≤ 3° → giữ yaw IMU tuyệt đối, **thêm** yaw từ tag với covariance vừa phải;
   - lệch lớn/thay đổi → yaw IMU chuyển `differential`, yaw tuyệt đối chỉ từ tag (và từ kế khi
     chưa thấy tag).
2. `marker_pose_republisher_node`: tính yaw thân máy = yaw_tag_bản_đồ − yaw_tag_trong_thân, phát
   kèm orientation + covariance yaw (hiện chỉ phát vị trí, hướng lấy từ IMU).
3. `ekf.yaml` `pose0_config`: bật yaw.

**Nghiệm thu:** cầm drone xoay trên tag: EKF yaw bám yaw tag ±2°; sau khi rời tag 10 m, sai lệch
quay lại tag < 3°.

### WP5 — Điều khiển yaw trên Pi *(cần WP0)*

- `/mission/setpoint` (PoseStamped) mang **yaw mong muốn** trong `orientation` (hiện bỏ trống).
- `position_controller_node`: vòng P yaw → `yaw_rate`, giới hạn **30 °/s**, chỉ chạy khi nguồn
  cruise/landing còn mới; lệnh vận tốc mission (velocity source) giữ `yaw_rate` của mission.
  Sai số ngang đã xoay theo yaw hiện tại (dòng 233–238) — không đổi.
- pytest: bọc góc ±180°, giới hạn tốc độ, không quay khi thiếu nguồn.

**Nghiệm thu:** để bàn, tắt cánh — `yaw_rate` gửi xuống đúng dấu/độ lớn; sau đó treo 1 m trên tag
home quay 90° và quay về.

### WP6 — Đích hạ cánh từ hai tag

- `landing_target_bridge_node`: nhận **cả** tag to và tag nhỏ của bãi đang nhắm (ID, ID+10); đổi
  pose tag nào đang thấy thành **tâm bãi** trong `base_link` (tag nhỏ: lùi 0,22 m theo hướng tag).
  Ưu tiên tag to khi cả hai cùng thấy và *h* > 1 m, tag nhỏ khi thấp hơn.
- Phát kèm hướng bãi (yaw_pad) trong thân máy cho `PAD_ALIGN`.
- `mission_manager_node`: xác thực ID chấp nhận cả cặp.

**Nghiệm thu:** pytest chuyển đổi tag nhỏ → tâm bãi ở các yaw; thử tay: dịch drone trên bãi in
thật, tâm bãi báo ra khớp thước ±2 cm khi chỉ thấy tag nhỏ.

### WP7 — FSM

- 3 trạng thái mới + `TRANSITIONS` + `MissionState.msg` + `mission_manager_node` (phát setpoint
  yaw).
- `ENROUTE`: yaw = phương vị tới đích khi còn > 1,5 m (đứng gần thì giữ yaw để không quay vòng);
  hạ xuống độ cao tiếp cận khi còn trong bán kính tiếp cận (tham số, đề xuất 5 m).
- `PRECISION_LAND` hai pha quanh `PAD_ALIGN`; `PRECISION_BLIND_BELOW_M` thay bằng ngưỡng hạ mù
  của `FINAL_APPROACH` (~0,15 m) và **tính từ hình học** thay vì hằng số.
- `MARKER_SEARCH`: thêm **quét bằng quay mũi** (camera nhìn trước nên quay 360° phủ vòng quanh)
  trước khi bay mẫu tìm.
- pytest cho từng chuyển trạng thái mới (bộ test FSM hiện có làm khung).

### WP8 — Mô phỏng

- `sim_tag_node`: tag có hướng, tag nhỏ theo mẫu, che khuất theo FOV thật; chạy trọn nhiệm vụ
  home → A → B → home trong Gazebo/sim, kiểm chuỗi trạng thái và điểm đáp.

### WP9 — Thử thực địa, theo bậc

1. Để bàn: yaw từ tag, chuyển giao tag to ↔ nhỏ khi nâng/hạ bằng tay trên bãi in thật.
2. Treo thấp (1 m) trên home: `ALIGN_HEADING` 90°/180°.
3. Một chặng ngắn home → A cách 3 m, độ cao 2,5 m.
4. Nhiệm vụ đủ: home → A (lấy) → B (thả) → home, 10 m mỗi chặng, 3–5 m.

## 4. Phụ thuộc

```
WP0 (FC) ─────────────► WP5 ──┐
WP1 ─► WP2 ─► WP4 ────────────┼─► WP7 ─► WP8 ─► WP9
   └─► WP3                    │
WP2 ─► WP6 ───────────────────┘
```

WP1–WP4 và WP6 **không cần** WP0 — làm được ngay trong lúc chờ sửa yaw FC.

## 5. Cần user đo / chuẩn bị

| # | Việc | Dùng cho |
|---|---|---|
| M1 | In tag to 25 cm và tag nhỏ 10 cm, **đo cạnh ô đen ngoài cùng** bằng thước | `apriltag.yaml` size |
| M2 | **Độ cao camera khi drone đã đáp** (từ đất tới ống kính) | Ngưỡng hạ mù, vị trí tag nhỏ |
| M3 | Ảnh chụp từ drone đã đáp trên bãi: càng/khung có che tag nhỏ không; ảnh có nét ở 15–30 cm không | Chốt `forward_m` |
| M4 | Ngoài trời: so yaw IMU với yaw từ tag (Pi ghi log, user cầm drone xoay trên tag) | Chiến lược WP4 |
| M5 | Cự ly bắt tag 25 cm thật ngoài trời (nắng, `decimate 2`) | Độ cao tiếp cận |
| M6 | Quy ước hướng "trên" của tag in: mũi tên vẽ lên tờ in | Tránh lệch 90°/180° |

## 6. Rủi ro

| # | Rủi ro | Giảm thiểu |
|---|---|---|
| R1 | Yaw FC vẫn yếu → quay chậm/lắc, tag nhỏ không vào khung | WP0 trước; phương án 4 tag nhỏ quanh tag to (quay tối đa 45°) giữ làm dự phòng |
| R2 | Lệch 90°/180° do hiểu sai hướng "trên" của tag | M6 + test xoay tay ở WP9.1 |
| R3 | Yaw IMU và yaw tag giằng nhau trong EKF | Đo M4 trước khi bật (WP4.1) |
| R4 | Gió ngoài trời đẩy lệch khi quay mũi ở độ cao thấp | Quay ở 1,2 m (`PAD_ALIGN`) khi tag to còn neo vị trí; giới hạn 30 °/s |
| R5 | `decimate: 2` giảm một nửa độ phân giải dò tag → cự ly thực ngắn hơn số tính | Đo M5; nếu thiếu, giảm `decimate` khi thấp (đổi tham số động) |
| R6 | Optical flow chưa thử chuyển động thật, tần số ~1 Hz (`quality_level`) | Thử dịch quãng đã biết trước WP9.3; xem lại `quality_level` |
| R7 | Đổi giao ước làm lệch CRC hai bên | Bản đồ không yaw giữ CRC cũ; thử bằng `gcs_sim.py` trước Pi thật |

## 7. Đã trả lời (user, 2026-10-08)

1. **Độ cao bay xa do GCS chọn theo từng kế hoạch** (không cố định trong `mission.yaml`). WP1 phải
   xác định trường mang độ cao này trong bản tin kế hoạch.
2. **`PAD_ALIGN` hết thời hạn mà chưa quay đúng hướng bãi → hạ tiếp bằng tag to** (không tính là
   thất bại). Hệ quả: dưới ~0,73 m tag to ra khỏi khung, nếu tag nhỏ không vào khung thì xử lý như
   mất tag hiện nay (trên ngưỡng hạ mù → `MARKER_SEARCH`). WP7 ghi rõ chuyển trạng thái này.
