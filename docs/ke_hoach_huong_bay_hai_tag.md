# Kế hoạch: bay quay mũi về đích và hạ cánh bằng hai tag

*Lập 2026-10-08. Người làm: Pi + GCS (Claude), FC (user).*

**Tiến độ (2026-10-08):** WP1 ✅ (giao ước 0.8, CRC_EXTRA không đổi) · WP2 ✅ (Pi: bản đồ có hướng,
tag nhỏ tự suy; sửa lỗi `tag_frames` lệch `apriltag.yaml` từ 09-17) · WP3 ✅ (GCS: ô "Đã đo hướng",
migration cột `yaw_valid`; thử trên dây với Pi miền ROS riêng: CRC khớp). Còn: WP0 (user), WP4–WP9.

**Tiến độ (2026-10-08, tối):**
- WP6 ✅ (code + Gazebo). `landing_target_bridge_node` chuyển tag to → tag nhỏ, đầu ra vẫn là tâm bãi.
  Chưa làm: phát `yaw_pad`; thử tay trên bãi in.
- WP5 🟡 chỉ mô phỏng. `/mission/yaw` → vòng P yaw 30 °/s trong `position_controller_node`.
  Cờ `mission.yaml yaw_control` để **false** trên drone thật cho tới khi xong WP0.
- WP7 🟡. `PAD_ALIGN` (12) dùng `approach_guidance`; quá 60 s thì hạ bằng tag to; MARKER_SEARCH giữ
  tại điểm tới nơi.
  Gazebo `sim_ba_bai.yaml`: ba bãi quay W/N/S, mũi khớp hướng bãi 0,0°, lệch tâm 0,2 cm.
  Chưa làm: ALIGN_HEADING/FINAL_APPROACH (11/13), quét bằng quay mũi, ngưỡng hạ mù tính từ hình học.

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
| Tâm tag nhỏ | **0,21 m** (0,22 tới 10-09) về phía "trên" tag to, tính từ tâm bãi | ≥ 0,206 m để không đè viền trắng tag to (25 cm); vừa dải nhìn thấy ở *h* 0,15–1,1 m |
| Dải chuyển giao | *h* **0,73 → 1,1 m** | Cả hai tag cùng thấy được |
| Hạ mù | dưới *h* ≈ **0,15 m** | Khoảng cuối ngắn, giữ vị trí bằng EKF/flow |
| Độ cao tiếp cận | **2,5 m** | Tag 25 cm còn ~30 px — bắt chắc |
| Độ cao bay xa | 3–5 m (theo kế hoạch GCS) | Dẫn bằng GPS (hacc ~1 m, đo 09-22) |

```
            phía "trên" của bãi (hướng bãi, yaw_pad)
                         ▲
                   ┌──────────┐
                   │ tag nhỏ  │  10 cm, ID = ID_to + 10, tâm cách tâm bãi 0,21 m
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
| `PAD_ALIGN` | 12 | **Tiếp cận thẳng hàng trục bãi** (mục 2b): qua cổng G sau bãi, mũi = **yaw_pad**, hạ dốc | tới trên tâm bãi ~1,1 m, \|lệch yaw\| ≤ 5° |
| `FINAL_APPROACH` | 13 | Hạ từ dải chuyển giao xuống, căn tâm theo **tag nhỏ** (quy về tâm bãi), hạ mù đoạn cuối | chạm đất → `ACTUATE_GRIPPER` |

Chuỗi một chặng (home → A):

```
TAKEOFF (neo bằng tag home) → ALIGN_HEADING (mũi về cổng G của A) → ENROUTE (bay cao theo alt_m,
mũi luôn về G; thấy tag A → neo, tính lại G) → PAD_ALIGN (vào thẳng trục bãi qua G, hạ dốc tới
~1,1 m, mũi = yaw_pad — mục 2b) → FINAL_APPROACH (tag nhỏ, hạ mù < 0,15 m) → ACTUATE_GRIPPER →
TAKEOFF → ALIGN_HEADING (mũi về cổng G của B) → …
(Không thấy tag khi tới G → MARKER_SEARCH như cũ.)
```

Các quy tắc giữ nguyên tinh thần FSM hiện tại: mất tag **trên** ngưỡng hạ mù vẫn là một lần thất
bại (về `MARKER_SEARCH`), không hạ mù sớm; mỗi lần thử có giới hạn.

## 2b. Tiếp cận mượt — bổ sung 2026-10-08 (THAY cách "treo rồi quay" ở `PAD_ALIGN`)

**Yêu cầu user:** trong lúc bay, camera liên tục tìm tag đích; thấy đúng tag thì dùng vị trí đọc từ
tag cập nhật quỹ đạo liên tục, để **tới nơi đã đúng hướng bãi** — không quay mũi khi đã ở trên bãi
(quay lúc đó làm camera quay theo, mất tag).

**Ràng buộc hình học** (camera nghiêng 20°, lệch 90 mm, tag to 25 cm đọc tới ~3,0 m ở 25 px):
camera chỉ thấy tới 16° **sau** phương thẳng đứng, ngang ±47°. Muốn tới nơi đúng hướng bãi mà vẫn
thấy tag suốt đường thì phải **đi vào từ phía SAU bãi, dọc trục bãi** (như máy bay vào đường băng).

```
                    ▲ hướng bãi (yaw_pad, phía "TRÊN")
               [tag nhỏ]
               [ TAG TO ]  ← tới đây ở ~1,1 m, mũi đã đúng hướng → FINAL_APPROACH
                   ▲
                   │  bay thẳng + hạ dốc ~31°, mũi = yaw_pad, tag luôn ở phía trước
                   │
                 (G) cổng tiếp cận: 1,5 m SAU tâm bãi, độ cao 2,0 m
                 ╱
   ENROUTE ─────╯  (mũi luôn hướng tới đích đang nhắm: G rồi tâm bãi)
```

**Luật dẫn (chạy liên tục, không có điểm dừng quay):**
1. `ENROUTE`: mũi = phương vị tới **cổng G** (không phải tới tâm bãi); độ cao bay xa theo `alt_m`.
   Camera nhìn trước nên thấy tag đích sớm nhất có thể; thấy đúng ID → EKF neo theo tag (vị trí +
   hướng, WP4) → G và trục bãi được tính lại theo số đo tag mới nhất ở MỖI chu kỳ.
2. Cách G ~1,5 m: **trộn dần** hướng mũi từ "phương vị tới G" sang `yaw_pad` theo khoảng cách, và
   bám **đường thẳng trục bãi** (sai lệch ngang so với trục → vận tốc ngang sửa dần, như ILS).
3. Qua G: mũi = `yaw_pad`, đi dọc trục về tâm bãi, hạ dốc tới ~1,1 m (dải chuyển giao 0,73–1,1 m).
4. Ràng buộc mọi lúc: |hướng mũi − phương vị tới tag| < 35° (trong FOV ±47° có biên); vượt thì
   giảm tốc ngang, ưu tiên quay mũi giữ tag; tốc độ quay ≤ 30 °/s.
5. Tới khoảng cách ngang < `acceptance` trên tâm bãi → `FINAL_APPROACH` (tag nhỏ).

**Trường hợp drone tới từ PHÍA TRƯỚC bãi** (đích nằm ngược hướng bãi): G ở sau bãi nên drone phải
vòng qua — bay vòng cung bán kính ≥ 1,5 m quanh tâm bãi tới G, **mũi luôn nhìn tâm bãi** (tag trong
khung suốt vòng; tới G mũi đã đúng `yaw_pad` — mô phỏng cho thấy để mũi theo đường đi thì tới G
không kịp quay, mất tag khi vào hành lang). Gợi ý
khi đặt bãi: cho mũi tên TRÊN chỉ **theo hướng drone thường bay tới** thì luôn vào thẳng, không vòng.

**Hệ quả lên các trạng thái 0.8:** `PAD_ALIGN` (12) đổi nghĩa thành **"tiếp cận thẳng hàng trục bãi"**
(bước 2–3, bao cả đoạn vòng), không còn là treo tại chỗ rồi quay. Quy tắc user đã chốt giữ nguyên:
hết thời hạn mà chưa thẳng hàng → hạ tiếp bằng tag to. `ALIGN_HEADING` (11) vẫn dùng sau cất cánh.

**Thân nghiêng (bổ sung 10-08):** camera gắn cứng nên tăng tốc/hãm làm vùng nhìn đổi (ngóc mũi 5° →
tag to mất từ 1,08 m thay vì 0,74 m). Đã làm: (a) `landing_target_bridge_node` phát đích trong hệ
**thân phẳng** `base_level` (bỏ roll/pitch) — trước đó nghiêng 5–10° ở 1 m tạo lệch ngang giả 9–17 cm;
(b) mô phỏng có quán tính + nghiêng theo gia tốc + chiếu 4 góc tag vào ảnh thật
(`test_approach_visibility`) — bắt được cổng 2,0 m làm mất tag khi bắt đầu tiến (chúc mũi ~7°) → cổng
dời về **1,5 m**. Chưa làm: giới hạn gia tốc gần bãi; nới dải chuyển giao (tag nhỏ 12 cm) — chờ user.

**Cần có trước:** WP0 (yaw FC), WP4 (yaw từ tag — để trục bãi đo được chính xác), WP5 (điều khiển
yaw), WP6 (đích hạ cánh từ hai tag). Luật dẫn viết thành hàm thuần (pytest) trong WP7.

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
2. `tagmap_crc`: nối bản ghi `(tag_id, yaw_cdeg)` **cho từng tag có hướng**, sau bản ghi gốc →
   bản đồ không hướng giữ nguyên CRC (đã viết vào giao ước 8.6, vectơ `0xD3A58731`). Cập nhật
   docstring `tagmap.py` (lý do cũ "yaw vô nghĩa" không còn đúng khi yaw đi từ tag). Thêm mã
   `ERR_RESERVED_TAG` (dải 10–19) và `ERR_YAW`.
3. `DRONE_MISSION_STATE`: thêm `ALIGN_HEADING=11`, `PAD_ALIGN=12`, `FINAL_APPROACH=13` (MINOR —
   GCS cũ bỏ qua giá trị lạ theo R1).
4. Tham số mẫu bãi (Q5) **không** đi qua dây: nằm trong `tags.yaml` của Pi. GCS chỉ cần biết để
   vẽ — ghi vào tài liệu.
5. Bảng lịch sử 0.8, sổ 8.1; `contract_ver` 800.

**Nghiệm thu:** XML mới sinh lại dialect hai bên; `tools/gcs_sim.py` nạp bản đồ có/không có yaw,
CRC khớp ở cả hai bên.

### WP2 — Bản đồ tag có hướng trên Pi

- `tags.yaml`: thêm `known_tags_heading: [id, độ, ...]` (hướng từ trục N bản đồ, chiều kim đồng
  hồ — cùng quy ước trên dây; chỉ tag có hướng), giữ nguyên `known_tags` 4 phần tử để không vỡ các
  bộ đọc khác, và mẫu bãi `pad_small_tag_id_offset: 10`, `pad_small_tag_size_m: 0.10`,
  `pad_small_tag_forward_m: 0.21`.
- Hàm thuần mới (pytest): đọc tag + yaw → **suy vị trí/hướng tag nhỏ** = tâm tag to + xoay
  (0,21 m) theo yaw_pad.
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
  pose tag nào đang thấy thành **tâm bãi** trong `base_link` (tag nhỏ: lùi 0,21 m theo hướng tag).
  Ưu tiên tag to khi cả hai cùng thấy và *h* > 1 m, tag nhỏ khi thấp hơn.
- Phát kèm hướng bãi (yaw_pad) trong thân máy cho `PAD_ALIGN`.
- `mission_manager_node`: xác thực ID chấp nhận cả cặp.

**Nghiệm thu:** pytest chuyển đổi tag nhỏ → tâm bãi ở các yaw; thử tay: dịch drone trên bãi in
thật, tâm bãi báo ra khớp thước ±2 cm khi chỉ thấy tag nhỏ.

### WP7 — FSM

- 3 trạng thái mới + `TRANSITIONS` + `MissionState.msg` + `mission_manager_node` (phát setpoint
  yaw).
- `ENROUTE` → `PAD_ALIGN` theo luật dẫn mục 2b (hàm thuần, pytest): đích trung gian là cổng G
  (1,5 m sau tâm bãi, 2,0 m cao), tính lại mỗi chu kỳ từ vị trí + hướng tag; trộn hướng mũi theo
  khoảng cách; bám trục bãi; ràng buộc |mũi − phương vị tag| < 35°; vòng cung khi tới từ phía trước.
- `PRECISION_LAND` (cũ) giữ cho bãi KHÔNG có hướng (không có tag nhỏ); `PRECISION_BLIND_BELOW_M`
  thay bằng ngưỡng hạ mù của `FINAL_APPROACH` (~0,15 m) và **tính từ hình học** thay vì hằng số.
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
| R4 | Gió ngoài trời đẩy lệch khi đang tiếp cận | Bám trục bãi bằng vị trí tag (cập nhật liên tục); không quay mũi khi đã trên bãi; giới hạn 30 °/s |
| R5 | `decimate: 2` giảm một nửa độ phân giải dò tag → cự ly thực ngắn hơn số tính | Đo M5; nếu thiếu, giảm `decimate` khi thấp (đổi tham số động) |
| R6 | Optical flow chưa thử chuyển động thật, tần số ~1 Hz (`quality_level`) | Thử dịch quãng đã biết trước WP9.3; xem lại `quality_level` |
| R7 | Đổi giao ước làm lệch CRC hai bên | Bản đồ không yaw giữ CRC cũ; thử bằng `gcs_sim.py` trước Pi thật |

## 7. Đã trả lời (user, 2026-10-08)

1. **Độ cao bay xa do GCS chọn theo từng kế hoạch** (không cố định trong `mission.yaml`). WP1 phải
   xác định trường mang độ cao này trong bản tin kế hoạch.
2. **`PAD_ALIGN` hết thời hạn mà chưa quay đúng hướng bãi → hạ tiếp bằng tag to** (không tính là
   thất bại). Hệ quả: dưới ~0,73 m tag to ra khỏi khung, nếu tag nhỏ không vào khung thì xử lý như
   mất tag hiện nay (trên ngưỡng hạ mù → `MARKER_SEARCH`). WP7 ghi rõ chuyển trạng thái này.
