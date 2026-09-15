# Giao ước GCS ↔ Pi — kênh nhiệm vụ và telemetry

**Hợp đồng liên thông giữa trạm mặt đất (GCS) và ROS 2 trên Raspberry Pi 4, qua 4G/LTE.**

| | |
|---|---|
| Phiên bản hợp đồng | **0.3** |
| Ngày | 2026-09-16 |
| Trạng thái | **Bản thảo 0.3 — cả 23 đề xuất của hai bên đã phân giải, không còn mục mở (mục 11.4).** Chưa mục nào [CHỐT] — điều kiện là chạy thật trên dây (10.A) |
| Phạm vi | Mọi thứ đi qua đường 4G/LTE giữa Pi và GCS. Kiến trúc nội bộ mỗi bên nằm ngoài phạm vi |
| Tài liệu song sinh | `GIAO_UOC_FC_ROS2.md` (đường FC ↔ Pi) — **hai hợp đồng độc lập, xem mục 1.2** |

---

## 0. Cách dùng tài liệu này

### 0.1 Vì sao có bản này, và vì sao có ngay bây giờ

`gcs_link_node` đã tồn tại trong repo từ Phiên 1 nhưng **cả bốn hàm chức năng đều là thân
rỗng**: không mở socket, không giải mã, không watchdog. Nguyên nhân gốc không phải thiếu thời
gian viết code — mà là **chưa ai định nghĩa khung gói trên dây**. Nợ này treo từ mục 1.8 #8 của
nhật ký, qua 10 phiên, vì mỗi lần định làm lại vướng đúng câu hỏi "gói tin trông như thế nào".

Bản này trả lời câu hỏi đó trước khi viết dòng code nào — đúng **bước 3 trong quy trình 8 bước**
của hợp đồng FC (`GIAO_UOC_FC_ROS2.md` mục 10.3), bước mà tài liệu đó ghi là *"hay bị bỏ nhất,
và là bước đắt nhất khi bỏ"*. Lịch sử dự án đã trả giá ba lần cho việc bỏ bước này (`type_mask`,
bảng `custom_mode`, hợp đồng ARM).

### 0.2 Thứ tự ưu tiên khi mâu thuẫn

1. **Phép đo trên dây** (`tcpdump` / `pymavlink`) — luôn thắng mọi tài liệu, kể cả bản này.
2. **File dialect** `docs/mavlink/drone_gcs.xml` — bản đặc tả máy đọc được, là nguồn duy nhất
   sinh mã cho **cả hai** bên.
3. **Tài liệu này** — giải thích ngữ nghĩa, ràng buộc thời gian, quy tắc mà XML không diễn đạt được.

Ai đo được một chỗ sai thì **sửa ngay vào đây**, đừng viết tài liệu phản hồi mới. Bài học đã rút
từ đường FC: mỗi tài liệu phản hồi mới lại sinh thêm một nguồn sự thật cạnh tranh.

> **BẢN GỐC DUY NHẤT:** `drone-ros2-jazzy` → `docs/GIAO_UOC_GCS_PI.md`.
> Repo GCS giữ **bản sao chỉ đọc**, đồng bộ từ bản gốc. Mọi sửa đổi — kể cả của phía GCS — làm ở
> bản gốc. Mỗi lần sửa ghi **một dòng vào bảng lịch sử cuối tài liệu**, kể cả sửa không tăng số.

### 0.3 Quy ước trạng thái

Giống hợp đồng FC, để hai tài liệu đọc lẫn được:

| Nhãn | Nghĩa | Được viết code dựa vào chưa |
|---|---|---|
| **[CHỐT]** | Hai bên đã thống nhất **và** đã chạy thật trên dây | Được |
| **[THOẢ THUẬN]** | Hai bên đã thống nhất, chưa hiện thực xong ở ít nhất một bên | Viết được, chưa test được |
| **[ĐANG LÀM]** | Đã giao việc cho một bên, đang chờ | Không |
| **[ĐỀ XUẤT]** | Một bên đề nghị, bên kia chưa trả lời | Không |

Nhãn được ghi ở từng mục; chỗ nào không ghi thì hiểu là nhãn của mục cha.

**Ở bản 0.2, hầu hết mục là [THOẢ THUẬN]** — Pi đề xuất, GCS đã phản hồi, hai bên thống nhất.
**Chưa mục nào [CHỐT]**, vì chưa có byte nào chạy trên dây; đó là điều kiện duy nhất để lên [CHỐT]
và cũng là điều kiện để hợp đồng lên **1.0** (mục 11.1). P18 do Pi nêu **đã được GCS trả lời**
(mục 11.2). Còn **năm mục mở P19–P23** do GCS nêu khi duyệt bản 0.2, chờ Pi (mục 11.3).

### 0.4 Quy ước đặt tên

- **GCS** = trạm mặt đất (phần mềm điều hành, chưa có trong repo này).
- **Pi** = ROS 2 Jazzy trên Raspberry Pi 4. Trên kênh này Pi là **một thành phần của phương tiện**,
  không phải một phương tiện riêng (mục 2.2).
- **FC** = firmware STM32H743. **Không tham gia kênh này** (mục 1.2).
- "Trên dây" = byte MAVLink thật giữa Pi và GCS. Hợp đồng nằm ở đây, **không** nằm ở ngữ nghĩa
  của message ROS.

---

## 1. Ranh giới trách nhiệm

### 1.1 Ai quyết định gì

| Việc | Bên quyết | Ghi chú |
|---|---|---|
| Nội dung kế hoạch nhiệm vụ (đi đâu, gắp gì) | **GCS** | |
| **Kế hoạch có hợp lệ hay không** | **Pi** | `mission_manager_node` là bên duy nhất được từ chối. GCS không tự kiểm hộ |
| Thời điểm cất cánh | **GCS** ra lệnh, **Pi** thi hành nếu điều kiện cho phép | Pi vẫn có quyền từ chối (chưa nạp kế hoạch, FC không trao quyền) |
| Leo thang failsafe | **Pi** | GCS **không** điều khiển failsafe. GCS chỉ được báo sự cố của chính nó (mất liên kết) — và việc đó Pi tự phát hiện |
| Điều khiển bay theo cần (manual) | **không thuộc kênh này** | Người lái dùng RC trực tiếp tới FC. Xem mục 9.3 |

**Nguyên tắc chi phối:** kênh này là **kênh nhiệm vụ**, không phải kênh điều khiển bay. Mất kênh
này thì drone phải tự hoàn thành hoặc tự về — không bao giờ phụ thuộc GCS để bay an toàn. Đây là
lý do failsafe mất GCS leo thang sang RTH chứ không sang "chờ lệnh".

### 1.2 Hai hợp đồng độc lập — Pi KHÔNG chuyển tiếp

```
GCS  <--- 4G/LTE, UDP, hợp đồng NÀY --->  Pi  <--- UART, GIAO_UOC_FC_ROS2 --->  FC
                                           |
                                    KHÔNG forward
```

**Pi tuyệt đối không chuyển tiếp bản tin giữa hai kênh.** Lý do đã ghi ở hợp đồng FC mục 10.5:
đường FC↔Pi là đường điều khiển, nhồi thêm dữ liệu nhiệm vụ sẽ làm băng thông điều khiển phụ
thuộc vào tải nhiệm vụ, và làm mờ ranh giới trách nhiệm.

Hệ quả cụ thể, dễ làm sai:

- Lệnh `MAV_CMD_NAV_RETURN_TO_LAUNCH` từ GCS **không** được chuyển thành `COMMAND_LONG` xuống FC.
  Nó trở thành một yêu cầu đổi trạng thái của `mission_manager_node`, và FSM quyết định phần còn lại.
- `DRONE_TELEMETRY` là gói **do Pi tổng hợp**, không phải gói FC phát được chuyển tiếp.
- Số hiệu bản tin của hai kênh nằm ở **hai sổ đăng ký khác nhau** (mục 8.1) — cố ý không trùng
  dải, để người gỡ lỗi nhìn một dump là biết đang xem kênh nào.

---

## 2. Liên kết vật lý và định danh

### 2.1 Tầng vận chuyển [THOẢ THUẬN]

| Hạng mục | Giá trị | Vì sao |
|---|---|---|
| Giao thức | **UDP**, MAVLink **v2** | v2 cần cho ID bản tin > 255 và cho chữ ký gói (mục 7) |
| Pi lắng nghe | `local_port` = **14551** | đã có trong `config/comms.yaml` |
| GCS lắng nghe | `gcs_host:gcs_port` = **:14550** | |
| MTU | ≤ 1400 byte một datagram | tránh phân mảnh IP trên đường 4G |

**Pi gọi ra trước, GCS trả lời về địa chỉ nguồn nó thấy.** [THOẢ THUẬN]

Đây là điểm bắt buộc phải làm đúng, không phải chi tiết hiện thực: modem 4G hầu như luôn nằm sau
NAT của nhà mạng (thường là CGNAT), nên **GCS không thể chủ động mở kết nối tới Pi**. Pi phải phát
`HEARTBEAT` 1 Hz tới `gcs_host:gcs_port` ngay khi khởi động, kể cả khi chưa nhận được gì.
**GCS gửi về địa chỉ nguồn của gói HỢP LỆ GẦN NHẤT từ `(sysid 1, compid 191)`** — khi đã bật chữ ký
gói thì "hợp lệ" nghĩa là **chữ ký đúng**. Pi nhận và gửi trên **cùng một socket** bind ở 14551.

Kéo theo hai ràng buộc:

1. **GCS phải có điểm cuối ổn định** — IP công cộng tĩnh, DNS động, hoặc cả hai bên nằm trong một
   VPN. Đây là yêu cầu hạ tầng, không phải việc của code.
2. Ánh xạ NAT hết hạn nếu im lặng (thường 30–120 s). `HEARTBEAT` 1 Hz giữ cho nó sống. **Không
   được tắt heartbeat để tiết kiệm băng thông** — mất ánh xạ NAT là mất đường về.

**Pi nhận đề xuất này của GCS và đây là lý do đầy đủ** [THOẢ THUẬN]: bản 0.1 ghi "gói **đầu tiên**"
— sai theo cách chỉ lộ ra ngoài hiện trường, vì modem nối lại hoặc CGNAT đổi ánh xạ thì GCS vẫn gửi
về địa chỉ cũ và **mất đường lên cho tới khi khởi động lại GCS**. Nhưng "gói mới nhất bất kỳ" cũng
sai theo chiều ngược lại: một gói giả mạo cướp được đường về, và dù nó không ra lệnh được (nhờ chữ
ký) thì nó vẫn **cắt được lệnh khẩn của người vận hành**. Điều kiện "hợp lệ gần nhất" là chỗ duy nhất
cân được cả hai. Phép kiểm **C5**.

### 2.2 `sysid` / `compid` [THOẢ THUẬN]

| Bên | `sysid` | `compid` | Ghi chú |
|---|---|---|---|
| FC (chỉ trên kênh FC↔Pi) | 1 | 1 (`AUTOPILOT1`) | đã chốt ở `mavros.yaml` |
| **Pi trên kênh này** | **1** | **191** (`MAV_COMP_ID_ONBOARD_COMPUTER`) | |
| **GCS** | **255** | **190** (`MAV_COMP_ID_MISSIONPLANNER`) | quy ước phổ biến |

Pi dùng `sysid = 1` **giống FC** là có chủ ý: theo ngữ nghĩa MAVLink, Pi và FC là **hai thành phần
của cùng một phương tiện**, không phải hai phương tiện. Hệ quả tốt: công cụ MAVLink chuẩn hiển thị
đúng "một drone, hai thành phần". Không có xung đột vì hai kênh là hai mạng riêng và Pi không
chuyển tiếp (mục 1.2).

Bên nhận **bỏ qua im lặng** mọi gói có `sysid`/`compid` ngoài bảng này — không log ồn, không coi là lỗi.

---

## 3. GCS → Pi: nạp kế hoạch nhiệm vụ

### 3.1 Vì sao không dùng giao thức mission chuẩn của MAVLink [THOẢ THUẬN]

Quy tắc của dự án là ưu tiên tuyệt đối bản tin chuẩn (hợp đồng FC mục 10.3 bước 1). Ở đây vẫn
**cố ý không dùng** `MISSION_ITEM_INT`, và đây là lý do:

Drone này **không có GPS** và điều hướng bằng AprilTag: vị trí waypoint **suy ra từ
`expected_marker_id`** qua `config/tags.yaml`, còn `lat`/`lon`/`pos_ned` của `MissionWaypoint`
bị `mission_manager_node` **bỏ qua hoàn toàn**. Nhồi kế hoạch này vào `MISSION_ITEM_INT` buộc phải
mượn trường `x`/`y` (vốn là kinh độ/vĩ độ ×10⁷) để chở `marker_id` và `action`. Kết quả: mọi công cụ
MAVLink chuẩn sẽ hiển thị một nhiệm vụ GPS **sai hoàn toàn** ở toạ độ vô nghĩa — tức là tự tạo ra
đúng loại nhầm lẫn ngữ nghĩa mà hợp đồng FC nói là thứ đắt nhất.

Đổi lại, ta **giữ nguyên hình dạng bắt tay** của giao thức chuẩn (count → request → item → ack),
vì cơ chế đó đã được kiểm nghiệm nhiều năm: có số thứ tự, có phát lại, có chống mất gói.

**Không mất gì về tương thích QGroundControl:** QGC vốn không diễn đạt được `expected_marker_id`
hay `action`, nên kế hoạch của dự án này không bao giờ soạn được bằng QGC.

### 3.2 Chuỗi bắt tay [THOẢ THUẬN]

```
GCS                                    Pi
 |-- DRONE_MISSION_COUNT (n, id) ------->|   bắt đầu nạp
 |<------- DRONE_MISSION_REQUEST (0) ----|   Pi hỏi từng điểm, KHÔNG nhận cả khối
 |-- DRONE_MISSION_ITEM (0) ------------>|
 |<------- DRONE_MISSION_REQUEST (1) ----|
 |-- DRONE_MISSION_ITEM (1) ------------>|
 |<------- DRONE_MISSION_ACK (mã, lý do)-|   sau khi mission_manager_node nhận/từ chối
```

| Quy tắc | Giá trị | Ghi chú |
|---|---|---|
| Pi chờ một `ITEM` | `item_timeout_s` = **1,0 s** | hết giờ thì **phát lại `REQUEST` cùng `seq`** |
| Số lần phát lại mỗi `seq` | **5** | quá thì `DRONE_MISSION_ACK` mã `ERR_TIMEOUT`, huỷ cả lượt nạp |
| `seq` đến không đúng cái đang chờ | **bỏ qua**, không tăng bộ đếm | gói trùng do phát lại là bình thường |
| `COUNT` mới khi đang nạp | **huỷ lượt cũ**, bắt đầu lượt mới | GCS thao tác lại là hợp lệ |
| Số điểm tối đa | **16** | đủ cho nghiệp vụ hiện tại; tăng thì tăng MINOR |
| Nạp xong | Pi publish `/mission/plan` **một lần** | `mission_manager_node` kiểm rồi Pi gửi `ACK` |

**`ACK` chỉ được gửi sau khi `mission_manager_node` đã trả lời**, không gửi sớm khi mới nhận đủ
byte. Lý do: bên duy nhất được phán quyết tính hợp lệ là Pi (mục 1.1), và lý do từ chối của nó là
thông tin GCS cần nhất.

### 3.2b Một mục kế hoạch nghĩa là gì — trả lời 11.P14 [THOẢ THUẬN]

Bốn câu GCS hỏi, trả lời từ hành vi thật của `mission_fsm.py`:

**1. Kế hoạch KHÔNG tự về home ở cuối.** Xong hành động ở waypoint cuối, FSM hạ cánh **tại chỗ đó**
rồi DISARM. Muốn drone về nhà thì **GCS phải thêm một mục cuối là marker home với `action = NONE`**.
Hợp đồng không thêm hành vi ngầm — về nhà hay không là quyết định của người lập kế hoạch.

**2. KHÔNG cần mục đầu là home.** Drone cất cánh từ chỗ nó đang đứng.

**3. Một mục = bay tới → tìm marker → hạ chính xác → làm `action` → cất cánh lại.** Đúng như GCS đoán.

> **`action = NONE` ở mục KHÔNG phải mục cuối: Pi thừa nhận đây là lỗi của mình.** Pi chạy thử
> 2026-09-16 và thấy FSM vẫn **hạ cánh** rồi vào `ACTUATE_GRIPPER`, và vì hàm chỉ phân hai nhánh
> (PICKUP / còn lại) nên nó **xử lý như DROPOFF**: log ghi *"giữ ổn định trước khi **thả**"* và
> *"xong **thả** tại tag 1"* cho một điểm không có hành động nào. Không lệnh gripper nào thực sự được
> phát nên không gây hại, nhưng **log nói dối** và ngữ nghĩa là tình cờ chứ không phải thiết kế.
>
> **Đã sửa ở commit `fa6bb17`** (riêng, không trộn vào việc kênh GCS): `ACTION_NONE` có nhánh riêng
> — hạ cánh, giữ ổn định, **không dính gripper**, log *"ghé tại tag N"*, rồi đi mục tiếp. Hành vi bay
> giữ nguyên, chỉ tên gọi thành thật. Thêm một test khẳng định log không còn chứa "thả"/"gắp".

**4. `alt_m` là độ cao TẠI mục đó, so với TAG ĐÍCH của mục đó** — không phải so với điểm cất cánh.
Công thức thật trong `build_waypoints`: `target = (tag_x, tag_y, tag_z + alt_m)`. Vì
`position_controller_node` bay thẳng tới điểm 3D đó nên nó **cũng là độ cao hành trình của chặng** —
không có tham số độ cao hành trình riêng. Đặt `alt_m` khác nhau giữa hai mục thì drone lên/xuống dần
trong lúc bay ngang, không phải bay bằng rồi mới đổi cao độ.

### 3.3 Mã kết quả trong `DRONE_MISSION_ACK` [THOẢ THUẬN]

| Mã | Tên | Nghĩa |
|---|---|---|
| 0 | `ACCEPTED` | `mission_manager_node` đã nạp |
| 1 | `ERR_TIMEOUT` | thiếu `ITEM` sau 5 lần hỏi |
| 2 | `ERR_COUNT` | `count` = 0 hoặc > 16 |
| 3 | `ERR_UNKNOWN_TAG` | `expected_marker_id` không có trong `tags.yaml` |
| 4 | `ERR_PARAM` | tham số ngoài dải (`max_vel_mps`, `alt_m`…) |
| 5 | `ERR_BUSY` | Pi không ở `IDLE`, hoặc đang chờ cất cánh theo kế hoạch cũ |
| 6 | `ERR_CONTRACT` | `MAJOR` của GCS khác `MAJOR` của Pi (mục 6.2) |

Kèm `reason[50]` — **chuỗi lý do do `mission_manager_node` sinh ra**, chuyển thẳng lên không sửa.
Mã số để máy xử lý, chuỗi để người đọc. Hai thứ này **không được mâu thuẫn**; khi mâu thuẫn thì
chuỗi là thứ đúng, vì nó đến từ bên phán quyết.

---

## 4. GCS → Pi: lệnh

### 4.1 Dùng `COMMAND_LONG` chuẩn [THOẢ THUẬN]

Ở đây **có** bản tin chuẩn phù hợp nên dùng chuẩn: `COMMAND_LONG` (76) + `COMMAND_ACK` (77). Được
luôn quy tắc **R4 của hợp đồng FC — mọi lệnh đều phải có đường trả lời** — mà không phải tự chế.

| `MAV_CMD` | Tên | Ý nghĩa trên kênh này |
|---|---|---|
| 300 | `MISSION_START` | = gọi service `~/start`: cất cánh và chạy kế hoạch đã nạp |
| 20 | `NAV_RETURN_TO_LAUNCH` | yêu cầu RTH — đổi trạng thái FSM, **không** gửi xuống FC |
| 21 | `NAV_LAND` | hạ cánh tại chỗ (= `~/land`) |
| 400 | `COMPONENT_ARM_DISARM` | chỉ chấp nhận **disarm**: `param1 = 0`. `param2 = 21196` = cắt ở mọi độ cao |
| **42100** | `DRONE_ABORT_MISSION` | huỷ kế hoạch đang nạp/đang chạy, hạ cánh, về `IDLE` |
| 193 | `DO_PAUSE_CONTINUE` | `param1` 0 = dừng giữ vị trí, 1 = đi tiếp. Không đổi `current_wp_index`. **Đã cấp số, FSM chưa hiện thực → `UNSUPPORTED`** (11.P7) |

`COMMAND_ACK.result` dùng giá trị chuẩn: `ACCEPTED` (0), `TEMPORARILY_REJECTED` (1), `DENIED` (2),
`UNSUPPORTED` (3), `FAILED` (4). **Lệnh lạ → `UNSUPPORTED`, không bao giờ im lặng.**

Hai quy ước dễ lẫn, chốt rõ: [THOẢ THUẬN]

- **`ACCEPTED` cho lệnh chạy dài** (300, 20, 21, 42100) nghĩa là *FSM đã nhận và bắt đầu chuyển
  trạng thái*, **không** phải "đã xong". GCS theo dõi kết quả qua `mission_state`, **không** dùng
  `IN_PROGRESS`.
- **`ARM` (`param1 = 1`) trả `DENIED`, không trả `UNSUPPORTED`.** Lệnh 400 *có* được hỗ trợ; chỉ
  hành động arm bị từ chối vĩnh viễn (mục 9.3). `UNSUPPORTED` sẽ khiến GCS tưởng là lỗi phiên bản.

### 4.2 Phát lại và tính bất biến [THOẢ THUẬN]

**Bất biến đến từ TRẠNG THÁI, không từ bộ nhớ lệnh.** Mỗi lệnh ở mục 4.1 đưa FSM tới một trạng
thái đích. Nếu FSM **đã ở** hoặc **đang chuyển tới** trạng thái đó thì Pi trả `ACCEPTED` và không
làm gì thêm — ví dụ `NAV_RETURN_TO_LAUNCH` khi đang `RTH`, `MISSION_START` khi đang chạy đúng
`mission_id` đó.

Bản 0.1 định nhớ lệnh đã xử lý theo `(command, confirmation)`. Cách đó **tự triệt tiêu**: cơ chế
phát lại chuẩn của MAVLink **tăng `confirmation`** mỗi lần, nên bản phát lại luôn có khoá khác và
không bao giờ bị coi là trùng. Phép kiểm A7 của bản 0.1 lại gửi ba lần cùng một `confirmation` nên
không bao giờ lộ ra lỗi này — đã sửa A7 thành `confirmation` = 0, 1, 2.

Nếu về sau vẫn cần bộ nhớ lệnh (cho lệnh không ánh xạ được sang trạng thái), khoá là
`(command, param1…param7)` trong 5 s và **bỏ qua `confirmation`**.

| Nhóm | Lệnh | GCS phát lại | Vì sao |
|---|---|---|---|
| Thường | 300, 193 | 1,0 s × 3 rồi báo người vận hành | thao tác lại được bằng tay |
| **Khẩn** | 20, 21, 400, 42100 | mỗi **0,5 s, KHÔNG giới hạn** tới khi có `ACK` hoặc người vận hành huỷ | |

Lệnh khẩn không được bỏ cuộc: trên 4G chập chờn, một lệnh `LAND`/`DISARM` dừng thử sau 3 giây là
bỏ drone lại giữa không trung, mà người vận hành cũng không làm gì hơn máy được. Báo động cho người
vận hành sau lần thứ 3 nhưng **vẫn tiếp tục phát**. An toàn được nhờ tính bất biến ở trên — phát lại
vô hạn một lệnh bất biến thì vô hại.

Chu kỳ 0,5 s / 1,0 s là **giá trị khởi điểm**, chỉnh lại theo số đo `rtt_ms` ở phép kiểm C3.

**Lệnh không bao giờ được xếp sau telemetry** — xem hàng đợi ưu tiên mục 5.3.

---

## 5. Pi → GCS: telemetry và log

### 5.1 Bảng phát [THOẢ THUẬN]

| ID | Bản tin | Nhịp | Vai trò |
|---|---|---|---|
| 0 | `HEARTBEAT` | **1 Hz** | sống/chết, giữ ánh xạ NAT. **Không bao giờ tắt** |
| **32** | **`LOCAL_POSITION_NED`** | **5 Hz**, chỉ khi `POS_VALID` | vị trí + vận tốc để GCS vẽ 3D và xét vùng cấm |
| **30** | **`ATTITUDE`** | **5 Hz**, chỉ khi `POS_VALID` | tư thế; GCS dùng `yaw` |
| 42010 | `DRONE_TELEMETRY` | **2 Hz cố định** | trạng thái tổng hợp |
| **111** | **`TIMESYNC`** | **0,2 Hz** | đo `rtt_ms` (mục 7.4) |
| 42011 | `DRONE_LINK_STATS` | **0,2 Hz** | sức khoẻ đường truyền, để gỡ lỗi (mục 7.4) |
| 253 | `STATUSTEXT` | theo sự kiện, **≤ 2 gói/s** | log người đọc được |
| 77 | `COMMAND_ACK` | theo sự kiện | trả lời mục 4 |
| **22** | `PARAM_VALUE` | khi được hỏi | ngưỡng chỉ đọc (mục 9.4) |
| 42002/42004 | `DRONE_MISSION_REQUEST` / `_ACK` | theo sự kiện | bắt tay mục 3 |

**Vị trí ngang dùng bản tin chuẩn, theo đúng 6.4 bước 1.** Bốn điểm chốt kèm theo: [THOẢ THUẬN]

1. **Hệ NED, gốc = gốc khung `odom`**, mà `tags.yaml` khai toạ độ tag trong chính khung đó — nên
   đây cũng là gốc bản đồ tag của GCS. Không lấy điểm cất cánh làm gốc. **Xem mục 11.P18: gốc khung
   và "nhà" của RTH không nhất thiết trùng nhau.**
2. `LOCAL_POSITION_NED` và `ATTITUDE` **không có chỗ cho cờ hiệu lực**, nên quy tắc là **chỉ phát
   khi `POS_VALID` = 1 — và không điều kiện nào khác** (P20). Không phát = không biết. Đây là cách duy
   nhất giữ được R3 với bản tin chuẩn. Đang đậu trên đất mà `POS_VALID` = 1 thì **vẫn phát**: GCS cần
   vị trí trên đất cho cảnh báo trước `MISSION_START` (P18).
3. **`alt_m` của `DRONE_TELEMETRY` = `−z` của `LOCAL_POSITION_NED`**, tức **độ cao so với gốc bản
   đồ tag**, để hai con số không bao giờ lệch nhau được. **Khác gốc với `alt_m` của
   `DRONE_MISSION_ITEM`** (so với tag đích, mục 8.3) — hai trường trùng tên, khác gốc, GCS tự quy đổi
   khi vẽ và khi kiểm trần bay (P22).
4. `vel_ned` của `DRONE_TELEMETRY` **trùng** `vx/vy/vz` của bản tin 32. Giữ cả hai: gói 42010 phải
   tự đủ nghĩa khi bản tin 32 không được phát (lúc `POS_VALID` = 0).

**`DRONE_TELEMETRY` phát đúng nhịp kể cả khi không có nguồn nào cập nhật** — nguồn thiếu thì để
giá trị mặc định và **hạ cờ hiệu lực tương ứng**. Đây là quy tắc R5 của hợp đồng FC: trạng thái
phải hỏi lại được, vì GCS có thể khởi động lại bất cứ lúc nào và sẽ lỡ mọi sự kiện trước đó.


### 5.2 Cờ hiệu lực — phần dễ sai nhất của gói này [THOẢ THUẬN]

`DRONE_TELEMETRY` mang bitmask `valid_flags`. **Bit = 0 nghĩa là KHÔNG BIẾT, không phải là 0.**

| Bit | Tên | Hạ cờ khi |
|---|---|---|
| 0 | `POS_VALID` | **odom CHƯA neo** theo `tags.yaml` lần nào / EKF không khoẻ / `/odometry/filtered` quá hạn. **Xem 5.2b** |
| 1 | `GLOBAL_POS_VALID` | **hiện luôn = 0: FC chưa có GPS** |
| 2 | `BATTERY_VALID` | **hiện luôn = 0: FC chưa gửi `BATTERY_STATUS`** |
| 3 | `FC_LINK_VALID` | `/mavros/state.connected` = false |
| 4 | `MARKER_VALID` | không bám marker nào |
| 5 | `GRIPPER_VALID` | `/gripper/status` quá hạn (node chưa chạy hoặc đã chết) |
| 6 | `EKF_HEALTHY` | `/ekf/health.healthy` = false |
| **7** | **`RSSI_VALID`** | chưa đọc được RSSI modem (P10) |
| **8** | **`HOME_VALID`** | `home` chưa chốt, hoặc chốt khi odom chưa neo (P21, 5.2b) |

`status_flags` (`uint16`, trường riêng) mang trạng thái **không phải hiệu lực dữ liệu**: [THOẢ THUẬN]

| Bit | Tên | Nguồn | Hiệu lực theo |
|---|---|---|---|
| 0 | `ARMED` | `/mavros/state.armed` | `FC_LINK_VALID` |
| 1 | `PI_HAS_AUTHORITY` | `OB_AUTH` (ch5/ch8, giao ước FC 6.3) | `FC_LINK_VALID` |
| 2 | `CARRYING` | `gripper_state == CLOSED` | `GRIPPER_VALID` |

Không có `ARMED` thì GCS **không kiểm chứng được** lệnh `DISARM` đã có tác dụng chưa. Không có
`PI_HAS_AUTHORITY` thì người vận hành không hiểu vì sao `MISSION_START` bị từ chối — nguyên nhân
thường gặp nhất là người lái chưa gạt ch5/ch8 để trao quyền cho Pi, việc GCS không nhìn thấy được.

Đây là quy tắc **R3 của hợp đồng FC** áp nguyên văn: *"Dữ liệu không tin cậy thì GẮN CỜ, đừng thay
bằng giá trị an toàn — `0` là một lời nói dối khác."*

Hai bit `GLOBAL_POS_VALID` và `BATTERY_VALID` hiện **luôn bằng 0** vì FC chưa gửi GPS và chưa gửi
pin. Ghi thẳng vào hợp đồng để phía GCS **không vẽ đồng hồ pin rồi hiển thị 0 %** — và để không ai
nhầm rằng failsafe pin đang bảo vệ mình (mục 9.2).

> **GCS:** đồng ý nguyên tắc — GCS sẽ không vẽ đồng hồ pin/GPS khi bit = 0.

### 5.2b Khung toạ độ trước và sau khi odom neo — trả lời P19 [THOẢ THUẬN]

GCS đọc đúng chú thích `tags.yaml`. Ba hệ quả, Pi nhận cả ba:

**1. `POS_VALID` = odom ĐÃ NEO, không phải "EKF hội tụ".** [THOẢ THUẬN]

```
POS_VALID = (odom đã neo theo tags.yaml ít nhất một lần)
            AND (/ekf/health.healthy)
            AND (/odometry/filtered còn tươi)
```

Trước lần neo đầu, gốc `odom` là **chỗ EKF khởi động**, không phải gốc bản đồ tag. Bật `POS_VALID`
lúc đó thì GCS vẽ drone ở toạ độ thuộc một khung khác rồi thấy nó nhảy.

**2. "NED" trên kênh này KHÔNG phải Bắc/Đông địa lý.** [THOẢ THUẬN] N và E là trục của **bản đồ
tag**: `n = y`, `e = x` của `tags.yaml`. `ATTITUDE.yaw` đo so với trục N đó. Đừng đem so với la bàn
hay bản đồ nền — yaw tuyệt đối của FC hiện **vô nghĩa** vì từ kế chưa hiệu chuẩn (giao ước FC 10.6a).

**3. Trả lời câu hỏi của GCS: odom NHẢY TỨC THÌ, không trượt dần.** [THOẢ THUẬN]
`ekf_health_node` ép `/set_pose` của `robot_localization` về pose marker khi EKF lệch marker > 1 m
liên tục 1 s — đó là một **bước nhảy tức thì**. Nhật ký Phiên 8.1 đo được: đẩy EKF lệch 20 m thì nó
về marker sau **~1 s**. Nên **GCS đúng khi không nội suy qua bước nhảy > 1 m**. Việc này cũng đã được
ghi là nợ nội bộ của Pi (nhật ký 8.3 #5: bước nhảy làm sai số cruise nhảy theo).

**4. `home` của RTH bị sai như GCS dự đoán — Pi đã kiểm và xác nhận.** [THOẢ THUẬN]

Pi chạy thử `mission_fsm` ngày 2026-09-16 đúng kịch bản GCS nêu (cất cánh lệch `pad_home` 3 m):

```
home chốt lúc cất cánh (khung CHƯA neo) = (0.0, 0.0, 1.5)
--- odom neo: cùng một chỗ vật lý, toạ độ báo đổi (0,0) -> (3,0) ---
RTH bay tới        : (0.0, 0.0)
điểm cất cánh THẬT : (3.0, 0.0)
pad_home           : (0.0, 0.0)
```

**RTH bay về `pad_home` chứ không về điểm cất cánh** — tức nó vô tình cư xử đúng như phương án (b)
mà GCS vừa bác, và không ai biết.

Nguyên nhân: `home` là **một con số**, chốt một lần trong `_step_takeoff`, thuộc khung tại thời điểm
đó. Sau khi neo, con số đó trỏ sang một chỗ vật lý khác.

**Cách sửa đã chốt** — chốt `home` ở **tick đầu tiên có `POS_VALID` = 1 trong lúc TAKEOFF**, thay vì
tick đầu tiên có bất kỳ vị trí nào:

- Drone **leo thẳng đứng** nên x, y lúc neo vẫn là x, y của điểm cất cánh (mô phỏng đo được x giữ
  trong ±0,01 m suốt lúc leo).
- Neo xảy ra **sớm trong lúc leo**: drone nằm trên `pad_home` **không thấy được pad của chính nó**
  (đo trong mô phỏng: ở z = 0,055 m thì tag dưới bụng nằm dưới `min_range` 0,15 m và ngoài FOV dọc vì
  camera chếch 20° ra trước — `/marker/pose_odom` im, `anchored` = false).
  **Đo trong Gazebo 2026-09-16: neo xảy ra 1,60 s sau khi bắt đầu leo**, tức z ≈ 0,8 m ở tốc độ leo
  0,5 m/s. *(Ước ban đầu của Pi là ~0,6 s — số đo thắng, theo mục 0.2.)*
- Trong 1,6 s leo thẳng đứng đó, x và y **không đổi tới 0,01 m** (đo trên cùng chuyến), nên `home`
  vẫn đúng trong vòng vài centimét và **RTH vẫn dùng được** — không phải hy sinh RTH.
- **Đã kiểm chứng đầu-cuối trong Gazebo sau khi sửa:** mất GCS lúc ENROUTE ở x = 8,0 m → RTH → hạ tại
  **x = 0,04 / y = −0,01**, đúng điểm cất cánh.
- Chưa neo mà đã cất cánh (không thấy tag nào) → `home` = `None` → `HOME_VALID` = 0 → `_vao_rth` **tự
  rơi về hạ cánh tại chỗ** (hành vi đã có). An toàn và trung thực.

**Đã hiện thực xong phía Pi** (commit riêng, ngoài phạm vi hợp đồng): thêm `bool anchored` vào
`EkfHealth` — `ekf_health_node` bật cờ khi có pose marker mà EKF cách nó không quá `anchor_tol_m`
(mặc định 2,0 m, bằng `pose0_rejection_threshold`), hoặc ngay khi ép `/set_pose` về marker. Cờ
**latching**: đã neo rồi thì không hạ xuống; độ tin cậy tức thời xét bằng `healthy`, đúng như GCS đề
ra ở P19 ý 1. `mission_manager_node` chuyển cờ vào `Snapshot`, và `_step_takeoff` chỉ chốt `home` khi
cờ bật.

Mô phỏng cũng được bổ sung để kiểm được việc này: `sim_fc_bridge_node` phát TF `odom → base_link`
(thật thì `robot_localization` phát), và `sim_mission.launch.py` chạy thêm
`marker_pose_republisher_node` — **node thật, không sửa gì** — nên chuỗi neo trong mô phỏng đi đúng
đường của drone thật.

> **Một tương tác P19 × P20 mà cả hai bên chưa nêu:** vì drone **không thấy pad của chính nó** khi
> nằm trên đất, `POS_VALID` sẽ **bằng 0 trong hầu hết trường hợp trước khi cất cánh**. Nên cảnh báo
> "lệch `pad_home`" trước `MISSION_START` mà GCS đề ra ở P18 sẽ **thường rơi vào nhánh "chưa có vị
> trí"** chứ không so sánh được khoảng cách. GCS đã lường nhánh đó, nhưng nên biết rằng nó là nhánh
> **thường gặp**, không phải ngoại lệ. Muốn có vị trí trên đất thì phải đặt thêm một tag trong tầm
> nhìn nghiêng của camera lúc đậu — việc bố trí hiện trường, ngoài phạm vi hợp đồng.

### 5.3 Hàng đợi ưu tiên [THOẢ THUẬN]

`gcs_link_node` đã có `PriorityQueue` với ba mức; hợp đồng chốt ngữ nghĩa:

| Mức | Nội dung | Khi nghẽn |
|---|---|---|
| **0** `EMERGENCY` | `COMMAND_ACK` của lệnh khẩn, `STATUSTEXT` mức ≥ `CRITICAL` | không bao giờ bỏ |
| **1** `MISSION` | bắt tay nạp kế hoạch, `DRONE_MISSION_ACK` | không bao giờ bỏ |
| **2** `TELEMETRY` | `DRONE_TELEMETRY`, `DRONE_LINK_STATS`, `STATUSTEXT` thường | **BỎ gói cũ, giữ gói mới nhất** |

Telemetry **bị bỏ chứ không xếp hàng**: một gói trạng thái cũ 5 giây thì vô giá trị, mà lại chiếm
băng thông của gói hiện tại. Chỉ giữ tối đa **1** gói telemetry chờ trong hàng đợi.

### 5.4 Ngân sách băng thông [THOẢ THUẬN]

| Luồng | Cỡ gói (MAVLink 2, có chữ ký) | Nhịp | Băng thông |
|---|---|---|---|
| `HEARTBEAT` | ~34 B | 1 Hz | 34 B/s |
| `DRONE_TELEMETRY` | ~110 B | 2 Hz | 220 B/s |
| `LOCAL_POSITION_NED` | ~53 B | 5 Hz **khi đang bay** | 265 B/s |
| `ATTITUDE` | ~53 B | 5 Hz **khi đang bay** | 265 B/s |
| `TIMESYNC` | ~41 B | 0,2 Hz × 2 chiều | 16 B/s |
| `DRONE_LINK_STATS` | ~53 B | 0,2 Hz | 11 B/s |
| **Nghỉ trên đất** (`POS_VALID` = 0) | | | **≈ 0,28 kB/s ≈ 2,3 kbit/s** |
| **Đang bay** | | | **≈ 0,81 kB/s ≈ 6,5 kbit/s** |

Vẫn rất nhẹ so với 4G. Ngân sách này tồn tại **không phải để tiết kiệm tiền** mà để giữ dư địa cho
lệnh khẩn đi ngay khi đường truyền xấu.

**Ngoại lệ 5 Hz là có điều kiện** [THOẢ THUẬN]: hai luồng chỉ phát khi `POS_VALID` = 1. **Mất vị
trí thì hai luồng tắt**, đưa băng thông về mức nghỉ.

Bản 0.2 viết "nằm trên đất hoặc mất vị trí thì tắt" — **sai**, và GCS bắt đúng (P20): "nằm trên đất"
không suy ra `POS_VALID` = 0 từ định nghĩa nào cả, mà GCS lại **cần** vị trí trên đất cho cảnh báo
trước `MISSION_START`. Điều kiện duy nhất là `POS_VALID`.

Trên thực tế `POS_VALID` **thường bằng 0 khi đậu** vì drone không thấy pad của chính nó (5.2b) — nhưng
đó là **hoàn cảnh, không phải quy tắc**. Ngân sách phải lấy theo trường hợp xấu nhất: **6,5 kbit/s**.

**Không thêm luồng định kỳ nào** mà không sửa bảng này trước.

---

## 6. Phiên bản và tương thích

### 6.1 Số hợp đồng

`MAJOR.MINOR`, quy tắc tăng giống hợp đồng FC mục 10.1:

| Đổi gì | Tăng gì |
|---|---|
| Đổi nghĩa/dấu/đơn vị một trường đang dùng; bỏ một bản tin; đổi mã `MAV_CMD`; đổi ý nghĩa một bit `valid_flags` | **MAJOR** |
| Thêm bản tin; thêm trường **sau `<extensions/>`** của bản tin cũ (R2); thêm mã lỗi; thêm bit `valid_flags` mới | **MINOR** |
| Sửa chính tả, thêm giải thích, thêm số đo | không tăng |

### 6.2 Hai bên biết nhau ở phiên bản nào [THOẢ THUẬN]

`DRONE_TELEMETRY.contract_ver` = `MAJOR × 10000 + MINOR × 100`. Bản 0.1 → `100`, bản 0.2 → `200`, bản **0.3 → `300`**.

Nằm trong bản tin **định kỳ** chứ không phải bản tin bắt tay — đúng quy tắc R5: GCS restart thì
vẫn biết ngay, không phải hỏi lại.

**Quy tắc khi lệch `MAJOR`:**

- **Pi từ chối nạp kế hoạch**, trả `DRONE_MISSION_ACK` mã `ERR_CONTRACT`, vẫn tiếp tục phát
  telemetry và vẫn nhận lệnh khẩn (hạ cánh, RTH).
- **GCS hiển thị cảnh báo rõ ràng**, không cho người vận hành soạn nhiệm vụ.

Lý do vẫn nhận lệnh khẩn khi lệch MAJOR: lệnh hạ cánh là thứ **phải luôn đi được**, kể cả khi hai
bên hiểu khác nhau về mọi thứ khác. Bốn lệnh khẩn ở mục 4.2 (20, 21, 400, 42100) vì vậy **không bao giờ được đổi nghĩa** —
đó là phần bất biến nhất của hợp đồng này.

### 6.3 Năm quy tắc tương thích

Kế thừa nguyên văn từ hợp đồng FC mục 10.2, vì chúng không phụ thuộc đường truyền:

**R1. Bản tin lạ, trường lạ, mã enum lạ → BỎ QUA im lặng, không crash.** Điều kiện để một bên nạp
bản mới trước bên kia.

**R2. Trường mới chỉ được thêm SAU thẻ `<extensions/>`, luôn ở cuối danh sách extension.**
Không thêm, xoá, đổi kiểu hay đổi tên bất kỳ trường nằm **trước** `<extensions/>`. [THOẢ THUẬN]

Lý do, và đây là chỗ bản 0.1 viết sai:

`CRC_EXTRA` của mỗi bản tin được tính từ tên bản tin cộng tên và kiểu của **mọi trường không phải
extension**. Thêm một trường thường — dù viết ở cuối file XML — làm `CRC_EXTRA` **đổi**, và bên
đang dùng định nghĩa cũ sẽ **loại cả gói** vì sai CRC. Không có cảnh báo, không có gói nào tới
được, và triệu chứng giống hệt mất sóng. Trường sau `<extensions/>` **không** tham gia
`CRC_EXTRA`, nên bên cũ đọc được phần nó biết và bỏ qua phần còn lại.

Bản 0.1 lẫn cơ chế này với *trailing-zero trimming*. Cắt byte 0 ở cuối gói chỉ là **tối ưu kích
thước**; nó cho phép đọc payload **ngắn hơn** (bên nhận tự bù 0), chứ không cấp tương thích cho
trường mới. Hai cơ chế độc lập nhau.

Kèm theo: `mavgen` **tự sắp trường không phải extension theo kích thước kiểu** khi xếp lên dây, và
MAVLink không có byte đệm. Nên thứ tự khai báo trong XML **chỉ quan trọng với trường extension** —
ở đó thứ tự là phần của hợp đồng vì không có gì sắp lại giúp.

**R3. Dữ liệu không tin cậy thì GẮN CỜ, đừng thay bằng giá trị "an toàn".** Đã áp ở mục 5.2.

**R4. Mọi lệnh đều phải có đường trả lời.** Đã áp ở mục 4.1.

**R5. Trạng thái phải hỏi lại được** — mọi thứ GCS cần để quyết định phải có ở bản tin **định kỳ**.
Đã áp ở mục 5.1 và 6.2.

### 6.4 Quy trình thêm một bản tin — 6 bước

1. **Tìm bản tin chuẩn trước.** Không có thì ghi rõ vì sao (như mục 3.1 đã làm).
2. **Ghi vào sổ 8.1** với trạng thái [ĐỀ XUẤT] + bên nào làm.
3. **Chốt cách điền từng trường trong tài liệu này, TRƯỚC KHI viết code**: đơn vị, hệ quy chiếu,
   giá trị "không biết", và **trường/bit nào mang cờ hiệu lực**.
4. **Sửa `docs/mavlink/drone_gcs.xml`**, tăng MINOR ở cả XML và bảng lịch sử cùng lúc.
5. **Đo trên dây** bằng `tools/gcs_sim.py` (mục 7.3) — cả hai chiều.
6. **Đổi trạng thái sang [CHỐT].** Bước này không tăng số.

### 6.5 Phế bỏ một mục

1. Đánh **[PHẾ BỎ từ ngày…]**, nêu thứ thay thế. **Vẫn phát, vẫn nhận.**
2. Bên tiêu thụ chuyển sang cái mới, xác minh bằng phép đo.
3. Sau ít nhất một chu kỳ nghiệm thu, ngừng phát. Tăng **MAJOR**.
4. **Số ID và tên đã cấp vĩnh viễn không tái sử dụng** — để trống trong sổ 8.1.

---

## 7. Gỡ lỗi — mục được thiết kế riêng cho việc này

Kênh này chạy trên 4G, tức là **sự cố sẽ xảy ra ở chỗ không ai quan sát được**. Mục này tồn tại để
lúc đó có cái mà đọc.

### 7.1 Một nguồn sự thật máy đọc được

`docs/mavlink/drone_gcs.xml` là dialect duy nhất, **có version trong repo**. Cả Pi và GCS sinh mã
từ đúng file đó:

```bash
python3 -m pymavlink.tools.mavgen --lang=Python --wire-protocol=2.0 \
        -o gcs_dialect docs/mavlink/drone_gcs.xml
```

Hai bên tự viết bộ mã hoá bằng tay là **đường chắc chắn dẫn tới lệch**. Hợp đồng FC đã trả giá cho
việc so khớp chuỗi tên bằng tay (mục 9.1: *"tên đã cấp không bao giờ đổi"*).

### 7.2 Đường truyền là UDP thuần — bắt gói được ngay

```bash
sudo tcpdump -i any -n udp port 14550 or udp port 14551 -w /tmp/gcs.pcap
```

Đây là một trong những lý do chọn UDP thuần thay vì bọc thêm tầng nào. Gói bắt được **phát lại
được** bằng `pymavlink`, nên lỗi hiếm tái hiện được thay vì chỉ đoán.

**Nếu bật chữ ký gói (mục 7.6) thì `tcpdump` vẫn đọc được nội dung** — chữ ký chỉ xác thực, không
mã hoá. Có chủ ý: gỡ lỗi được quan trọng hơn giữ kín nội dung nhiệm vụ.

### 7.3 `tools/gcs_sim.py` — GCS tối thiểu, ở trong repo [THOẢ THUẬN]

Một script `pymavlink` khoảng 150 dòng, làm đúng bốn việc: nạp kế hoạch từ YAML, gửi các lệnh ở
mục 4.1, in `DRONE_TELEMETRY` và `STATUSTEXT`, đếm gói mất.

Vai trò: **cả hai bên test ngược vào nó.** Pi test khi chưa có GCS thật; GCS test bằng cách so với
hành vi của nó. Nó cũng là đặc tả chạy được — khi tài liệu và nó lệch nhau thì ít nhất có chỗ để đo.

Đọc chung định dạng YAML với `send_mission_plan` đã có, nên một file kế hoạch chạy được cả hai
đường (nội bộ ROS và qua dây).

### 7.4 `DRONE_LINK_STATS` — biến "đường truyền tệ" thành con số [THOẢ THUẬN]

| Trường | Nghĩa |
|---|---|
| `rx_ok` | gói MAVLink hợp lệ đã nhận |
| `rx_drop` | gói mất, suy từ khoảng trống của `seq` MAVLink |
| `rx_bad_crc` | gói sai CRC |
| `rx_bad_sig` | gói sai chữ ký (nếu bật mục 7.6) |
| `tx_sent` | gói đã gửi |
| `tx_dropped` | gói telemetry bị bỏ do nghẽn (mục 5.3) |
| `queue_depth` | độ sâu hàng đợi hiện tại |
| `rtt_ms` | vòng khứ hồi, đo bằng `TIMESYNC` (111) — xem ghi chú dưới bảng |

Không có bảng này thì mọi báo cáo sự cố đường truyền đều là *"thấy lag"*. Có nó thì phân biệt được
**mất gói** với **nghẽn hàng đợi** với **sai chữ ký** — ba nguyên nhân cần ba cách sửa khác nhau.

`rx_drop` đếm được vì MAVLink có `seq` 1 byte tăng dần **theo từng `(sysid, compid)`**; đếm khoảng
trống là ra số gói mất. Đây là công cụ chẩn đoán có sẵn, không dùng thì lãng phí.

`rtt_ms` **không đo được bằng `HEARTBEAT`** như bản 0.1 viết — bản tin đó không có trường thời gian
nào. Dùng `TIMESYNC` (111), vốn tồn tại đúng cho việc này: mỗi bên phát 0,2 Hz, bên kia trả lời ngay
với `tc1` điền theo quy ước chuẩn. [THOẢ THUẬN]

Trường bên phát **không đo được** thì điền `UINT32_MAX` (`uint16` → `0xFFFF`), không điền 0 — theo
đúng R3. Ví dụ `pymavlink` không tách được `rx_bad_crc`. [THOẢ THUẬN]

### 7.5 `STATUSTEXT` — log người đọc được, có phanh

Ánh xạ mức log ROS sang `MAV_SEVERITY`:

| ROS | `MAV_SEVERITY` |
|---|---|
| `FATAL` | 2 `CRITICAL` |
| `ERROR` | 3 `ERROR` |
| `WARN` | 4 `WARNING` |
| `INFO` | 6 `INFO` |

**Chỉ gửi `WARN` trở lên** theo mặc định, và **chặn nhịp ≤ 2 gói/s**. Không có phanh thì một node
lỗi sẽ tự làm nghẽn chính đường mà người vận hành cần để hạ cánh — đúng loại lỗi làm mất drone.

`CRITICAL` đi ở hàng đợi ưu tiên 0, ngang lệnh khẩn.

**Chia đoạn thay vì cắt** [THOẢ THUẬN]: `STATUSTEXT.text` chỉ 50 byte, mà chuỗi lý do của
`mission_manager_node` thường dài hơn — ví dụ *"tu choi ke hoach 903: dang cho cat canh theo ke hoach
cu - khong doi ke hoach luc nay"* là 88 byte. Dùng hai trường extension chuẩn của MAVLink 2 là `id`
(số hiệu chuỗi) và `chunk_seq` (thứ tự đoạn) để GCS ghép lại. Cắt ngắn sẽ mất đúng phần cuối câu —
phần thường chứa lý do.

### 7.6 Chữ ký gói — bắt buộc trước khi bay thật [THOẢ THUẬN]

**UDP thuần trên 4G công cộng nghĩa là bất cứ ai biết `IP:port` đều gửi được lệnh cho drone.** Bốn
lệnh ở mục 4.1 có cả `disarm` — tức là "cắt động cơ giữa không trung".

MAVLink 2 có **chữ ký gói** sẵn (`MAVLINK_SIGNING`, HMAC-SHA256 với khoá bí mật 32 byte chia sẻ
trước, kèm timestamp chống phát lại). Hợp đồng đề xuất:

| Hạng mục | Giá trị |
|---|---|
| Trạng thái | **[THOẢ THUẬN]** — hai bên đồng ý cơ chế, chưa hiện thực ở bên nào |
| Khi nào bắt buộc | trước **mọi** chuyến bay ngoài mạng kín |
| Gói không có chữ ký / sai chữ ký | **bỏ ngay**, tăng `rx_bad_sig`, **không xử lý lệnh** |
| Khoá | không bao giờ commit vào repo; nằm ở file ngoài, quyền `600` |

Giai đoạn phát triển trong mạng kín (Pi và GCS cùng LAN) thì tắt được, nhưng **cờ tắt phải nằm
trong `comms.yaml` và mặc định là BẬT**, để không ai vô tình bay với kênh mở.

Đây là mục tôi khuyến nghị đừng để thành nợ: nợ an toàn khác nợ tính năng ở chỗ nó không gây bất
tiện gì cho tới lúc gây thiệt hại.

---

## 8. Sổ đăng ký

Mọi thứ hai bên cùng dùng đều được cấp số ở đây. **Thêm mục mới = sửa bảng này trước, viết code sau.**

### 8.1 Bản tin

| Dải | Dùng cho |
|---|---|
| ID chuẩn `common.xml` | ưu tiên tuyệt đối |
| **`42000–42099`** | dialect riêng kênh GCS↔Pi |
| ~~`180–229`~~ | **của dialect FC**, hợp đồng FC mục 9.1. **Không dùng ở đây** |

Chọn `42000+` thay vì dùng chung dải `180–229` là có chủ ý: nhìn một `tcpdump` là biết ngay đang xem
kênh nào, không phải tra tài liệu. Dải này chỉ MAVLink 2 mới chở được — thêm một lý do bắt buộc v2.

| ID | Bản tin | Chiều | Trạng thái |
|---|---|---|---|
| 0 | `HEARTBEAT` | ↔ | [THOẢ THUẬN] |
| **21** | `PARAM_REQUEST_LIST` | GCS→Pi | [THOẢ THUẬN] — chỉ đọc, mục 9.4 |
| **22** | `PARAM_VALUE` | Pi→GCS | [THOẢ THUẬN] — mục 9.4 |
| **23** | ~~`PARAM_SET`~~ | GCS→Pi | **Pi BỎ QUA + `STATUSTEXT` WARN.** Ngưỡng chỉ sửa ở `safety.yaml` (mục 1.1) |
| **30** | `ATTITUDE` | Pi→GCS | [THOẢ THUẬN] — 5 Hz, chỉ khi `POS_VALID` |
| **32** | `LOCAL_POSITION_NED` | Pi→GCS | [THOẢ THUẬN] — 5 Hz, chỉ khi `POS_VALID` |
| 76 | `COMMAND_LONG` | GCS→Pi | [THOẢ THUẬN] |
| 77 | `COMMAND_ACK` | Pi→GCS | [THOẢ THUẬN] |
| **111** | `TIMESYNC` | ↔ | [THOẢ THUẬN] — 0,2 Hz, đo `rtt_ms` |
| 253 | `STATUSTEXT` | Pi→GCS | [THOẢ THUẬN] — chia đoạn, mục 7.5 |
| 42001 | `DRONE_MISSION_COUNT` | GCS→Pi | [THOẢ THUẬN] |
| 42002 | `DRONE_MISSION_REQUEST` | Pi→GCS | [THOẢ THUẬN] |
| 42003 | `DRONE_MISSION_ITEM` | GCS→Pi | [THOẢ THUẬN] |
| 42004 | `DRONE_MISSION_ACK` | Pi→GCS | [THOẢ THUẬN] |
| 42010 | `DRONE_TELEMETRY` | Pi→GCS | [THOẢ THUẬN] |
| 42011 | `DRONE_LINK_STATS` | ↔ | [THOẢ THUẬN] |

**Còn trống:** `42005–42009`, `42012–42099`. Chưa cấp cho ai.

**Bản tin 32 và 30 là bản tin CHUẨN được dùng đúng nghĩa chuẩn** — khác hẳn `MISSION_ITEM_INT` ở mục
3.1 mà ta từ chối vì phải bóp méo ngữ nghĩa. Đây là ranh giới của quy tắc "ưu tiên bản tin chuẩn":
dùng khi nó vừa, từ chối khi phải mượn trường sai nghĩa.

### 8.2 `MAV_CMD`

| Dải | Dùng cho |
|---|---|
| `MAV_CMD` chuẩn | ưu tiên tuyệt đối — bốn lệnh ở mục 4.1 |
| **`42100–42149`** | lệnh riêng kênh này |

| Mã | Tên | Trạng thái |
|---|---|---|
| 42100 | `MAV_CMD_DRONE_ABORT_MISSION` | [THOẢ THUẬN] |

**Còn trống:** `42101–42149`.

`MAV_CMD_DO_PAUSE_CONTINUE` (**193**) đã **cấp số** ở mục 4.1 nhưng FSM chưa hiện thực — Pi trả
`UNSUPPORTED` và GCS ẩn nút. Cấp số trước để khi làm không phải đổi giao thức (mục 11.P7).

### 8.3 Trường của `DRONE_MISSION_ITEM` — ánh xạ sang `MissionWaypoint`

| Trường trên dây | Kiểu | `MissionWaypoint` | Ghi chú |
|---|---|---|---|
| `seq` | `uint8` | `seq` | 0 trở lên, liên tục. `uint8` để khớp `MissionWaypoint.seq` |
| `expected_marker_id` | `int32` | `expected_marker_id` | **nguồn vị trí duy nhất.** Phải có trong `tags.yaml` |
| `action` | `uint8` | `action` | 0 `NONE`, 1 `PICKUP`, 2 `DROPOFF` |
| `alt_m` | `float` | `alt_m` | m, **so với tag đích của mục đó**: `z_odom = tag_z + alt_m` (P22) |
| `acceptance_radius_m` | `float` | `acceptance_radius_m` | m, ngang |
| `max_vel_mps` | `float` | `max_vel_mps` | m/s, trong `(0; 1,9]` |
| `loiter_s` | `float` | `loiter_s` | s |

**KHÔNG chở `lat`, `lon`, `pos_ned`.** Ba trường đó có trong message ROS `MissionWaypoint` vì nó
mirror `mission_waypoint_t` phía GCS/FC, nhưng `mission_manager_node` **bỏ qua hoàn toàn** — vị trí
suy từ `expected_marker_id`. Chở chúng lên dây chỉ tạo ra ảo giác là chúng có tác dụng.

Đây là chỗ **hợp đồng cố tình hẹp hơn message ROS**. Khi nào có GPS thì thêm trường mới sau
`<extensions/>` (MINOR, theo R2), đừng hồi sinh ba trường này.

### 8.4 Trường của ba bản tin còn lại

Đủ để viết `drone_gcs.xml` mà không phải đoán. **Tên, kiểu và việc trường nằm trước hay sau
`<extensions/>` là phần của hợp đồng**; thứ tự chỉ ràng buộc với trường extension (R2).

**`DRONE_MISSION_COUNT`** (42001, GCS→Pi) — mở đầu một lượt nạp:

| Trường | Kiểu | `MissionPlan` | Ghi chú |
|---|---|---|---|
| `mission_id` | `uint32` | `mission_id` | GCS cấp, Pi không sinh |
| `count` | `uint8` | `len(waypoints)` | 1–16; ngoài dải → `ERR_COUNT` |
| `max_retries` | `uint8` | `max_retries` | 0 = dùng `mission.yaml` |
| `search_timeout_s` | `float` | `search_timeout_s` | s; 0 = dùng `mission.yaml` |
| `issued_stamp_us` | `uint64` | `issued_stamp` | **micro giây UNIX UTC**. Pi chỉ ghi log, không dùng để quyết định |
| `plan_name` | `char[20]` | `plan_name` | **cắt còn 20 byte, không báo lỗi** — chỉ để người đọc |

`plan_name` cắt ngắn mà không báo lỗi là quyết định có chủ ý: tên kế hoạch không ảnh hưởng hành vi,
nên từ chối cả kế hoạch chỉ vì tên dài là hại nhiều hơn lợi. Ghi rõ ở đây để phía GCS không ngạc
nhiên khi thấy tên bị cắt trong log.

**`DRONE_MISSION_REQUEST`** (42002, Pi→GCS): `mission_id` (`uint32`), `seq` (`uint8`).
**`DRONE_MISSION_ACK`** (42004, Pi→GCS): `mission_id` (`uint32`), `result` (`uint8`, mục 3.3),
`reason` (`char[50]`, cắt ngắn không báo lỗi).

**`DRONE_TELEMETRY`** (42010, Pi→GCS, 2 Hz). Thứ tự dưới đây chỉ để **người đọc** dễ tra —
`mavgen` tự sắp trường không phải extension theo kích thước kiểu khi xếp lên dây (R2). Các trường
đánh **`ext`** nằm **sau `<extensions/>`**, ở đó thứ tự **là phần của hợp đồng**:

| Trường | Kiểu | Nguồn | Cờ hiệu lực |
|---|---|---|---|
| `stamp_us` | `uint64` | `stamp` | — |
| `mission_id` | `uint32` | `mission_id` | — |
| `contract_ver` | `uint32` | hằng số của Pi | — (mục 6.2) |
| `lat` | `int32` | `lat` × 10⁷ | `GLOBAL_POS_VALID` |
| `lon` | `int32` | `lon` × 10⁷ | `GLOBAL_POS_VALID` |
| `marker_id_tracking` | `int32` | `marker_id_tracking` | tag đang **BÁM**. `MARKER_VALID` = 0 thì **bỏ qua trường**, không đọc −1 |
| `alt_m` | `float` | `alt_m` | `POS_VALID` |
| `vel_ned` | `float[3]` | `vel_ned` | `POS_VALID`. **NED**, không phải FLU |
| `battery_pct` | `float` | `battery_pct` | `BATTERY_VALID` |
| `battery_v` | `float` | `battery_v` | `BATTERY_VALID` |
| `valid_flags` | `uint16` | (mới) | mục 5.2 |
| `status_flags` | `uint16` | (mới) | mục 5.2 |
| `gcs_rssi_dbm` | `int16` | `gcs_rssi_dbm` | `RSSI_VALID` |
| `mission_state` | `uint8` | `mission_state` | bảng 8.5 |
| `current_wp_index` | `uint8` | `current_wp_index` | — |
| `wp_total` | `uint8` | `len(waypoints)` | 0 = chưa nạp kế hoạch |
| `retry_count` | `uint8` | `retry_count` | — |
| `gripper_state` | `uint8` | `gripper_state` | `GRIPPER_VALID`, bảng 8.5 |
| `failsafe_type` | `uint8` | `failsafe_type` | bảng 8.5; 0 = bình thường |
| `expected_marker_id` | `int32` **`ext`** | `/mission/expected_marker_id` | tag đang **TÌM**; −1 = không tìm |
| `tagmap_crc` | `uint32` **`ext`** | tính từ `tags.yaml` | mục 8.6 |
| `home_n_mm` | `int32` **`ext`** | `MissionFsm.home[0]` × 1000 | mục 11.P18 |
| `home_e_mm` | `int32` **`ext`** | `MissionFsm.home[1]` × 1000 | mục 11.P18 |

**Bỏ trường `fc_connected` khỏi gói** [THOẢ THUẬN]: nó trùng nghĩa với bit `FC_LINK_VALID`. Giữ cả
hai thì phải nhớ hai quy ước cho cùng một thông tin, và chúng sẽ lệch nhau vào đúng lúc quan trọng.
Định nghĩa chốt: **`FC_LINK_VALID` = `/mavros/state` còn tươi VÀ `connected` = true.**

Phân biệt hai trường dễ lẫn: `marker_id_tracking` là tag **đang bám thấy**, `expected_marker_id` là
tag **đang đi tìm**. Hai số này khác nhau suốt pha MARKER_SEARCH và đó là lúc người vận hành cần
nhìn nhất.

`stamp_us` là **đồng hồ hệ thống của Pi**, có thể lệch nếu chưa đồng bộ NTP. Chỉ dùng để ghi log và
sắp thứ tự sự kiện; **xét độ tươi thì dùng thời điểm GCS nhận**, không dùng trường này. [THOẢ THUẬN]

`vel_ned` dùng **NED** vì đó là hệ của `ODOMETRY` từ FC (hợp đồng FC mục 3.1), không phải FLU của
`position_controller_node`. Đây đúng loại chỗ hợp đồng FC gọi là *"phần dễ rơi máy bay nhất"* — hai
hệ quy chiếu cùng tồn tại trong một hệ thống, đổi dấu z. **Điền sai dấu ở đây thì GCS hiển thị drone
đang lên khi nó đang xuống.**

**`DRONE_LINK_STATS`** (42011, ↔, 0,2 Hz): tám trường ở mục 7.4, `uint32` cho các bộ đếm,
`uint16` cho `queue_depth` và `rtt_ms`.

---

### 8.5 Bảng giá trị enum trên dây [THOẢ THUẬN]

Bắt buộc phải có ở đây theo 6.4 bước 3 — bản 0.1 chỉ ghi "theo hằng số `MissionState`", buộc phía
GCS phải đọc mã nguồn ROS. Hợp đồng phải tự đủ.

**Pi đã đối chiếu từng giá trị với hằng số ROS thật ngày 2026-09-16: bảng GCS gửi khớp chính xác cả
bốn, không sửa gì.**

| `mission_state` | | `failsafe_type` | |
|---|---|---|---|
| 0 | `IDLE` | 0 | `FS_NONE` |
| 1 | `TAKEOFF` | 1 | `FS_MARKER_TIMEOUT` |
| 2 | `ENROUTE` | 2 | `FS_GRIP_CONFIRM_FAIL` |
| 3 | `MARKER_SEARCH` | 3 | `FS_LINK_LOST` |
| 4 | `PRECISION_LAND` | 4 | `FS_LOW_BATTERY` |
| 5 | `ACTUATE_GRIPPER` | 5 | `FS_EKF_UNHEALTHY` |
| 6 | `RETRY_LOITER` | 6 | `FS_FC_COMM_LOST` |
| 7 | `RTH` | | |
| 8 | `EMERGENCY_LAND` | **`gripper_state`** | |
| 9 | `MISSION_COMPLETE` | 0 `OPEN` · 1 `CLOSED` · 2 `MOVING` · 3 `ERROR` | |
| 10 | `FAILSAFE` | **`action`** 0 `NONE` · 1 `PICKUP` · 2 `DROPOFF` | |

**Cảnh báo về tên `EMERGENCY_LAND`:** đây là trạng thái **hạ cánh chung**, không chỉ hạ cánh khẩn
cấp. Nó là trạng thái đích của cả `NAV_LAND` bình thường, cả "xong kế hoạch, hạ cánh", cả hạ cánh do
failsafe. **GCS không nên hiện chữ "KHẨN CẤP" khi thấy giá trị 8** — hãy phân biệt bằng
`failsafe_type`: bằng 0 thì đây là hạ cánh bình thường. (Trả lời câu hỏi trong 11.P8.)

### 8.6 `tagmap_crc` — công thức [THOẢ THUẬN, công thức đã sửa so với đề xuất GCS]

Mục đích theo 11.P2: phát hiện bản đồ tag của hai bên lệch nhau. `ERR_UNKNOWN_TAG` chỉ bắt **thiếu**
tag, không bắt **sai vị trí** — mà sai vị trí thì drone bay chỗ khác chỗ GCS vẽ, và phép kiểm vùng
cấm của GCS sai mà không ai biết.

**GCS đề xuất bản ghi gồm `yaw_cdeg`, `size_mm`, `kind`. Pi không tính được ba trường đó**, nên công
thức phải sửa. Trả lời câu hỏi của GCS về `tags.yaml`:

| Trường GCS đề nghị | Pi có không | Ghi chú |
|---|---|---|
| `tag_id`, vị trí | **có** | `known_tags: [id, x, y, z, ...]` — chỉ 4 số mỗi tag |
| `yaw` | **không có** | `tags.yaml` không khai yaw. Giao ước FC 10.6a ghi yaw tuyệt đối hiện **vô nghĩa** (chưa hiệu chuẩn từ kế), nên đưa vào CRC là đưa vào một số không ai tin |
| `size` | **có nhưng ở file khác** | `apriltag.yaml`, và `pad_a` đang **giả định** 0,122 m *chưa đo bằng thước*. Đưa một giá trị giả định vào CRC là khoá cứng một phỏng đoán |
| `kind` (home/pickup/dropoff) | **không có** | đây là khái niệm **lập kế hoạch của GCS**, không phải dữ liệu Pi. Pi chỉ biết `action` của từng waypoint, không phân loại bãi đáp |

**Công thức chốt cho 0.2** — CRC-32 IEEE (đa thức `0xEDB88320`, khởi tạo `0xFFFFFFFF`, đảo cuối)
trên chuỗi bản ghi **little-endian**, sắp theo `tag_id` **tăng dần**:

```
(uint16 tag_id, int32 n_mm, int32 e_mm, int32 d_mm)     # 14 byte mỗi tag
```

`n/e/d` = **NED milimét**, đổi từ `known_tags` (khai theo **ENU mét**):
`n_mm = round(y·1000)`, `e_mm = round(x·1000)`, `d_mm = round(−z·1000)`.

**Dùng `round()` của Python** (làm tròn nửa về số chẵn), **không dùng `int()`** — `int()` cắt cụt nên
0,1229 m thành 122 mm ở một bên và 123 mm ở bên kia là đủ để CRC lệch mãi mãi. [THOẢ THUẬN]

**Vectơ kiểm — Pi đã tự tính lại và khớp chính xác giá trị GCS gửi** (P23): với `tags.yaml` hiện tại
(tag 0 tại gốc, tag 1 tại x = 10 m), chuỗi 28 byte là

```
0000 00000000 00000000 00000000  0100 00000000 10270000 00000000
```

và `zlib.crc32` cho **`0x6BDEA0A6`**. Phép kiểm **A15** dùng đúng con số này, **chạy được bằng pytest,
không cần lên dây**. Đổi sang NED để khớp hệ của `LOCAL_POSITION_NED` ở mục 5.1 — **một hệ quy chiếu duy
nhất trên cả kênh**, không để hai hệ cùng tồn tại.

Chỉ gồm tag đang bật. Với `tags.yaml` hiện tại (tag 0 tại gốc, tag 1 tại x = 10 m) thì chuỗi là hai
bản ghi 14 byte.

**Thêm `size_mm` vào CRC là việc MINOR**, làm sau khi đo `pad_a` bằng thước (nợ nhật ký 6.2 #5) và
sau khi `tags.yaml` gánh luôn `size` thay vì để ở `apriltag.yaml`. Không làm bây giờ để CRC không
khoá cứng một con số giả định.

Quy trình vận hành [THOẢ THUẬN]: **GCS xuất `tags.yaml`** từ trang thiết kế khu vực rồi triển khai
lên Pi; GCS **khoá chức năng nạp kế hoạch** khi CRC lệch. Nạp bản đồ qua dây để MINOR sau nếu cần.

---

---

## 9. Bảng trạng thái — cái gì chưa có

### 9.1 Phía Pi: `gcs_link_node` là khung rỗng

Đo ngày 2026-09-15: node lên, đăng ký đủ ba topic, **không mở socket nào** (`ss -ulnp` không thấy
14550/14551). Bốn hàm đều là thân rỗng: `on_rx` (và **không có gì gọi nó**), `pump_tx`,
`on_telemetry`, `check_watchdog`.

`telemetry_aggregator_node` đã nối dây đủ 6 nguồn và lưu lại, nhưng `publish_packet` cũng là thân
rỗng nên `/telemetry/outgoing` **chưa bao giờ phát**.

Việc phải làm, theo thứ tự — **đây là danh sách triển khai của bản 0.2**:

| # | Việc | Ghi chú |
|---|---|---|
| 1 | `docs/mavlink/drone_gcs.xml` theo mục 8 | **Chặn mọi việc khác.** Trường mới phải nằm sau `<extensions/>` ngay từ đầu (R2) |
| 2 | Sửa `TelemetryPacket` (message ROS) | Thêm `valid_flags`, `status_flags`, `contract_ver`, `wp_total`, `retry_count`, `expected_marker_id`, `tagmap_crc`, `home_n_mm`, `home_e_mm`; **bỏ `fc_connected`**; `gcs_rssi_dbm` → `int16`. Phải xong **trước** khi hai bên sinh mã |
| 3 | `telemetry_aggregator_node.publish_packet` | Kèm điền `valid_flags`/`status_flags` — phần dễ sai nhất (5.2) |
| 4 | Hàm tính `tagmap_crc` từ `tags.yaml` | Mục 8.6. Viết thành hàm thuần để pytest được, giống `mission_fsm` |
| 5 | `gcs_link_node` | Socket, vòng nhận, bắt tay mục 3, lệnh mục 4, watchdog 9.2, hàng đợi 5.3 |
| 6 | Phát `LOCAL_POSITION_NED` + `ATTITUDE` 5 Hz có điều kiện `POS_VALID` | Mục 5.1 |
| 7 | `tools/gcs_sim.py` (7.3) | Làm **song song**, không làm sau: không có nó thì không test được |
| 8 | `PARAM_REQUEST_LIST`/`PARAM_VALUE` chỉ đọc | Mục 9.4 |
| ~~9~~ | ~~Thêm `bool anchored` vào `EkfHealth`~~ | **XONG** — `ekf_health_node` bật khi neo lần đầu (5.2b) |
| ~~10~~ | ~~`_step_takeoff` chỉ chốt `home` khi đã neo~~ | **XONG** — đã kiểm Gazebo: RTH hạ tại x = 0,04 / y = −0,01 |
| ~~11~~ | ~~Sửa chú thích `alt_m` trong `MissionWaypoint.msg`~~ | **XONG** — chú thích cũ ghi sai gốc (P22) |

Việc ngoài hợp đồng nhưng lộ ra khi phân giải P14: sửa `_step_actuate_gripper` cho `ACTION_NONE`
(mục 3.2b) — **đã xong ở commit `fa6bb17`**, riêng, không trộn vào việc kênh GCS.

### 9.2 Failsafe mất GCS hiện là cấu hình chết

`failsafe_monitor_node` chỉ tính mất GCS **khi `gcs_link_node` BÁO `false`**. Vì node đó không bao
giờ publish `/gcs_link/connected`, biến `gcs_disconnected_since_s` mãi là `None` → **không sự cố
nào sinh ra**, và không có cảnh báo nào cho người vận hành biết là mình đang không được bảo vệ.

Hợp đồng chốt hai tầng thời gian, **cố ý khác nhau**:

| Tầng | Ngưỡng | Ai giữ |
|---|---|---|
| `gcs_link_node` coi là mất liên kết | `link_timeout_s` = **5 s** | `comms.yaml` |
| `failsafe_monitor_node` leo thang RTH | `link_lost_timeout_s` = **10 s** | `safety.yaml` |

5 s để phát hiện, thêm 5 s nữa mới hành động — chống rung khi 4G chớp tắt ngắn, chuyện bình thường
trên đường di động.

**`/gcs_link/connected` phải publish khi đổi trạng thái VÀ định kỳ 1 Hz**, không chỉ khi đổi. Lý do
là R5: `failsafe_monitor_node` có thể restart và sẽ lỡ mọi sự kiện trước đó.

Ghi chú liên quan: failsafe **pin** cũng đang là cấu hình chết vì FC chưa gửi `BATTERY_STATUS` — xem
`nhat_ky_lam_viec.md` mục 10.1. Hai failsafe cùng nằm trong `safety.yaml`, cùng trông như đã bật,
cùng bất động.

### 9.4 Đối chiếu ngưỡng — chỉ đọc [THOẢ THUẬN]

Tài liệu kiến trúc (`thiet_ke_kien_truc_node_ros2.md` mục 4.6) vốn yêu cầu ngưỡng failsafe của Pi
phải **khớp bảng `system_config` phía GCS để hai bên không lệch ngưỡng**. Bản 0.1 bỏ sót việc này.

Dùng bản tin chuẩn, **một chiều đọc**: `PARAM_REQUEST_LIST` (21) → Pi trả `PARAM_VALUE` (22).
**`PARAM_SET` (23) thì Pi BỎ QUA và phát `STATUSTEXT` mức WARN** — không vi phạm mục 1.1 vì ngưỡng
chỉ được sửa ở `safety.yaml`, nơi có người chịu trách nhiệm và có lịch sử git.

Tên tham số ≤ 16 ký tự (giới hạn MAVLink), ánh xạ sang YAML:

| Tên trên dây | Nguồn |
|---|---|
| `LOW_BATT_PCT` | `safety.yaml: low_battery_pct` |
| `CRIT_BATT_PCT` | `safety.yaml: critical_battery_pct` |
| `LINK_LOST_S` | `safety.yaml: link_lost_timeout_s` |
| `MARKER_SRCH_S` | `safety.yaml: marker_search_timeout_s` |
| `GRIP_CONF_S` | `safety.yaml: grip_confirm_timeout_s` |
| `MAX_RETRIES` | `mission.yaml: max_retries` |
| `TAKEOFF_ALT_M` | `mission.yaml: takeoff_alt_m` |
| `ACCEPT_RAD_M` | `mission.yaml: acceptance_radius_m` |

**GCS nên hiển thị cảnh báo khi `LOW_BATT_PCT` đọc về mà `BATTERY_VALID` luôn = 0** — đó chính là
tình trạng hiện nay (mục 9.2): ngưỡng có cấu hình nhưng không bao giờ kích hoạt được.

### 9.3 Cái gì KHÔNG thuộc hợp đồng này

| Việc | Đi đường nào | Vì sao không qua đây |
|---|---|---|
| Điều khiển bay theo cần | RC → FC trực tiếp | kênh 4G có trễ và có thể mất; không bao giờ được dùng để lái |
| Vào/ra OFFBOARD, trao quyền cho Pi | công tắc RC ch5/ch8 → FC | hợp đồng FC mục 6.2, 6.3 — GCS **không** can thiệp được |
| ARM | GCS **không được** arm | chỉ `disarm` (mục 4.1). Arm là quyết định tại chỗ, cần người nhìn thấy drone |
| Ảnh camera, AprilTag thô, EKF | nội bộ ROS 2 | băng thông 4G không chở được, và GCS không cần |
| Log nghiệp vụ, ảnh xác nhận gắp/thả | `~/drone_logs/`, lấy sau chuyến bay | `mission_logger_node`; không stream |

**GCS không được arm** là mục tôi đề nghị giữ tuyệt đối. Arm từ xa nghĩa là động cơ quay khi người
ra lệnh không nhìn thấy drone.

> **GCS: đồng ý toàn bộ 9.3.** GCS gỡ nút ARM, TAKEOFF rời, GOTO, giữ vị trí tay khỏi giao diện.

---

## 10. Nghiệm thu

Chia theo thứ tự cần điều kiện tăng dần. Mỗi mục là một phép đo, không phải một ý kiến.

### 10.A — Hai tiến trình trên cùng một máy

Không cần drone, không cần 4G. Đủ để chốt phần lớn hợp đồng.

| # | Phép kiểm | Đạt khi |
|---|---|---|
| A1 | `gcs_sim.py` và `gcs_link_node` cùng máy, `gcs_host = 127.0.0.1` | `ss -ulnp` thấy cả hai cổng; `HEARTBEAT` hai chiều 1 Hz |
| A2 | Nạp kế hoạch 2 điểm hợp lệ | `DRONE_MISSION_ACK = ACCEPTED`; `/mission/plan` publish đúng một lần; nội dung khớp YAML từng trường |
| A3 | Nạp kế hoạch có tag lạ | `ERR_UNKNOWN_TAG` + chuỗi lý do **giống nguyên văn** log của `mission_manager_node` |
| A4 | Bỏ một `DRONE_MISSION_ITEM` giữa chuỗi | Pi phát lại `REQUEST` cùng `seq`; nạp xong bình thường |
| A5 | Bỏ hẳn một `seq` | sau 5 lần hỏi → `ERR_TIMEOUT`, không treo |
| A6 | Gửi `MAV_CMD` lạ | `COMMAND_ACK = UNSUPPORTED`, node không chết |
| A7 | Gửi cùng một lệnh 3 lần với `confirmation` = **0, 1, 2** | trạng thái đổi **đúng một lần**, cả 3 lần đều có `ACK`. Bản 0.1 dùng cùng `confirmation` nên không bao giờ lộ lỗi 4.2 |
| A8 | `DRONE_TELEMETRY` khi chưa có nguồn nào | vẫn phát đúng 2 Hz; `BATTERY_VALID` = 0, `GLOBAL_POS_VALID` = 0 |
| A9 | Cắt `gcs_sim.py` | sau 5 s `/gcs_link/connected` = false; sau 10 s nữa `failsafe_monitor_node` báo `FS_LINK_LOST` |
| A10 | Bật lại `gcs_sim.py` | `/gcs_link/connected` = true, sự cố được gỡ |
| A11 | Làm nghẽn: chặn đường ra 10 s rồi mở | `tx_dropped` tăng, `queue_depth` **không tăng vô hạn**, lệnh khẩn vẫn đi ngay khi mở |
| A12 | Lệch `MAJOR` giả lập | `ERR_CONTRACT`; telemetry **vẫn phát**; lệnh hạ cánh **vẫn đi** |
| **A13** | Sinh mã từ XML bản N và N+1 (N+1 thêm **một trường sau `<extensions/>`**), cho hai bên đọc gói của nhau **cả hai chiều** | cả bốn tổ hợp đọc được; bên cũ bỏ qua trường mới, bên mới thấy 0. **Đây là phép kiểm bảo vệ R2** — thiếu nó thì lỗi của bản 0.1 sẽ tái diễn |
| **A14** | Thêm một trường **trước** `<extensions/>` rồi lặp A13 | gói bị loại vì sai `CRC_EXTRA`. **Phép kiểm này PHẢI thất bại** — nó chứng minh vì sao R2 tồn tại |
| **A15** | (a) Hàm tính `tagmap_crc` của **cả hai bên** trả `0x6BDEA0A6` với `tags.yaml` hiện tại. (b) Đổi một số trong `tags.yaml` ở một bên | (a) khớp vectơ kiểm 8.6 — **pytest, không cần lên dây**. (b) CRC lệch; GCS khoá nạp kế hoạch |
| **A16** | `PARAM_REQUEST_LIST`, rồi thử `PARAM_SET` | đọc được 8 tham số mục 9.4; `PARAM_SET` bị bỏ qua + `STATUSTEXT` WARN |
| **A17** | `STATUSTEXT` dài 88 byte | GCS ghép đủ, không mất phần cuối (7.5) |

### 10.B — Chạy trong Gazebo

Thêm điều kiện: `sim_mission.launch.py`. Kiểm hợp đồng trong ngữ cảnh bay thật (mô phỏng).

| # | Phép kiểm | Đạt khi |
|---|---|---|
| B1 | Nạp kế hoạch 2 chặng qua dây, rồi `MISSION_START` | drone bay hết chuỗi gắp/thả như khi nạp bằng `send_mission_plan` |
| B2 | `NAV_RETURN_TO_LAUNCH` giữa ENROUTE | FSM sang RTH, về nhà, hạ cánh |
| B3 | Cắt liên kết giữa ENROUTE | sau 15 s tổng → RTH tự động (A9 + leo thang) |
| B4 | `DRONE_ABORT_MISSION` giữa PRECISION_LAND | hạ cánh, về `IDLE`, kế hoạch bị xoá |
| B5 | Đọc `DRONE_LINK_STATS` suốt chuyến | `rx_drop` = 0 trên loopback; `rtt_ms` < 5 ms |

### 10.C — Qua 4G thật

Thêm điều kiện: modem 4G, GCS có điểm cuối ổn định.

| # | Phép kiểm | Đạt khi |
|---|---|---|
| C1 | Pi gọi ra trước sau NAT | GCS nhận được `HEARTBEAT` mà **không** cần cấu hình chuyển cổng |
| C2 | Im lặng 5 phút rồi gửi lệnh | lệnh tới được — ánh xạ NAT còn sống nhờ heartbeat 1 Hz |
| C3 | Đo `rtt_ms` và `rx_drop` 10 phút | ghi số vào tài liệu này; đây là **mốc chuẩn** để so về sau |
| C4 | Bật chữ ký gói, gửi gói không chữ ký từ máy thứ ba | bị bỏ, `rx_bad_sig` tăng, **lệnh không được thi hành** |
| **C5** | Tắt rồi bật dữ liệu di động giữa chuyến | telemetry và lệnh thông lại **≤ 5 s**, không phải khởi động lại gì ở hai bên (kiểm mục 2.1 sửa theo 11.P12) |

---

## 11. Trạng thái phân giải — bản 0.2

### 11.1 Toàn bộ 17 đề xuất của GCS đã được xử lý

GCS gửi 17 đề xuất ngày 2026-09-16 (lập luận đầy đủ ở commit `a88364e`). Pi trả lời 2026-09-16; nội
dung đã **nhập vào thân tài liệu** nên mục này chỉ còn bảng truy vết — đúng quy tắc 0.2: một câu hỏi
một câu trả lời, không giữ hai bản song song.

| Mã | Phán quyết của Pi | Nằm ở mục |
|---|---|---|
| **P3** | **Nhận. Đây là LỖI của Pi trong bản 0.1**, và là lỗi nặng nhất: R2 lẫn *trailing-zero trimming* với `CRC_EXTRA`. Thêm trường thường làm bên cũ **loại cả gói**, không phải bỏ qua phần thừa | 6.3 R2 viết lại · A13, A14 |
| **P4** | **Nhận. Lỗi của Pi**: chống trùng theo `(command, confirmation)` tự triệt tiêu vì `confirmation` tăng mỗi lần phát lại | 4.2 viết lại · A7 sửa |
| **P6** | **Nhận. Lỗi của Pi**: `HEARTBEAT` không có trường thời gian nên không đo được RTT | 7.4 · `TIMESYNC` vào 8.1 |
| **P1** | Nhận nguyên văn, kèm 4 điểm chốt: gốc = gốc `odom`, chỉ phát khi `POS_VALID`, `alt_m = −z`, giữ cả `vel_ned` | 5.1 · 5.4 ngoại lệ có điều kiện |
| **P2** | Nhận **mục đích**, **sửa công thức**: Pi không có `yaw`/`kind`, và `size` của `pad_a` còn là giả định chưa đo. CRC chỉ trên `(tag_id, n_mm, e_mm, d_mm)` | 8.6 · A15 |
| **P5** | Nhận nguyên văn | 4.2 |
| **P8** | Nhận. **Pi đã đối chiếu: bảng GCS gửi khớp chính xác cả bốn với hằng số ROS** | 8.5 |
| **P7** | Nhận việc cấp số 193. **Trả lời: `RETRY_LOITER` KHÔNG dùng lại được** — nó cố định 3 s (`RETRY_LOITER_S`) và **tăng `retry_count`**, nên "giữ vô thời hạn" bằng nó sẽ tự đếm vào hạn mức thử lại rồi chuyển hạ cánh khẩn. Cần trạng thái riêng | 4.1 · 8.2 |
| **P9** | Nhận cả 4 trường | 5.2 `status_flags` · 8.4 |
| **P10** | Nhận cả ba: thêm `RSSI_VALID`, `int16`, **bỏ `fc_connected`** | 5.2 · 8.4 |
| **P11** | Nhận | 8.4 |
| **P12** | Nhận nguyên văn, đề xuất của GCS **tốt hơn** bản 0.1 | 2.1 · C5 |
| **P13** | Nhận | 9.4 mới · A16 |
| **P14** | Trả lời cả 4 câu. Câu 3 **làm lộ một lỗi trong `mission_fsm.py` của Pi** (`ACTION_NONE` bị xử lý như DROPOFF) — đã sửa ở commit `fa6bb17` | 3.2b |
| **P15** | Nhận | 7.5 · A17 |
| **P16** | Nhận cả hai | 4.1 |
| **P17** | Nhận | 7.4 |

**Không đề xuất nào bị bác.** Ba mục P3, P4, P6 là lỗi của Pi; P2 là đề xuất đúng hướng nhưng chưa
khả thi với dữ liệu Pi hiện có nên sửa công thức.

**Vì sao vẫn giữ 0.x chứ không lên 1.0:** theo 6.1, sửa nghĩa quy tắc tương thích (R2) là MAJOR.
Nhưng chưa mục nào [CHỐT], chưa byte nào chạy trên dây, và chưa bên nào nạp bản 0.1 vào sản phẩm —
nên tăng MAJOR ở đây chỉ tạo tiếng ồn. **Bản 1.0 là bản đầu tiên đi qua được toàn bộ phép kiểm 10.A.**

**GCS chấp nhận toàn bộ bảng trên (2026-09-16)**, gồm cả công thức `tagmap_crc` đã sửa (P2) và câu
trả lời về `RETRY_LOITER` (P7).

### 11.2 Mục mới do Pi nêu — đã được GCS trả lời

**P18 · Cao — "Nhà" của RTH và "home" trên bản đồ GCS có thể là hai chỗ khác nhau.**

Cả hai bên đều chưa nêu chỗ này. `mission_fsm.py` hiện lấy **vị trí lúc cất cánh** làm nhà (chốt
trong `_step_takeoff`, vì drone leo thẳng đứng nên x, y vẫn là của nhà). GCS thì sẽ vẽ "home" là
**tag 0 / `pad_home`** theo bản đồ khu vực. Hai chỗ đó **chỉ trùng nếu drone cất cánh đúng trên
`pad_home`**. Cất cánh lệch 3 m thì RTH về điểm cất cánh, còn người vận hành nhìn drone bay sai chỗ
so với hình họ thấy — và sẽ tưởng là lỗi điều khiển.

Ba phương án, Pi nghiêng về (a):

**(a) Giữ hành vi, báo sự thật** — RTH vẫn về điểm cất cánh, nhưng telemetry mang thêm
`home_n_mm`/`home_e_mm` (đã đưa vào 8.4 làm trường extension) để GCS **vẽ đúng chỗ drone sẽ về**
thay vì giả định. Không đổi hành vi bay, không cần đo thêm gì, và dùng được ngay.

**(b) RTH về tag 0.** Ưu: `pad_home` là điểm đã khảo sát trong bản đồ, còn điểm cất cánh chỉ là một
giá trị `odom` có thể đã trôi. Nhược: nếu drone cất cánh ở chỗ không phải bãi đáp thì tag 0 có thể
rất xa, mà RTH lại thường được kích hoạt vì **pin yếu**.

**(c) Hạ chính xác tại nhà** — RTH bay về rồi tìm tag 0 và hạ theo marker. Đúng nhất về độ chính xác
nhưng làm RTH phụ thuộc vào việc thấy tag, tức là đường thoát hiểm phụ thuộc thị giác. Pi không đề
nghị làm.

*Cần GCS trả lời:* bản đồ khu vực của GCS có bắt buộc drone cất cánh trên `pad_home` không? Nếu có
thì (a) và (b) trùng nhau trong thực tế và câu hỏi này tự hết.

**GCS trả lời (2026-09-16): chọn (a).** [THOẢ THUẬN, phụ thuộc P19 và P21]

- **GCS không bắt buộc cứng** việc cất cánh trên `pad_home`: bộ lập kế hoạch GCS chỉ bắt buộc bản đồ
  có đúng một tag `home`, không ràng buộc chỗ drone đứng. Bắt buộc cứng thì cần một phép đo trước cất
  cánh, và phép đo đó chính là thứ (a) cung cấp.
- **GCS vẽ "nhà của RTH" từ `home_n_mm`/`home_e_mm`**, tách biệt với biểu tượng `pad_home`. Khi
  `HOME_VALID` = 0 (P21) thì **không vẽ gì**, không vẽ thay bằng `pad_home`.
- **GCS cảnh báo trước `MISSION_START`** khi vị trí drone trên đất (`LOCAL_POSITION_NED`, cần P20) cách
  `pad_home` quá dung sai bãi đáp, hoặc khi chưa có vị trí. Chỉ cảnh báo, không chặn: quyết định cất
  cánh vẫn là của người vận hành và Pi (mục 1.1).
- **(b) không chọn** vì đúng lập luận của Pi: RTH hay đến do pin yếu, không được kéo dài đường về.
  **(c) không chọn**: đường thoát hiểm không được phụ thuộc thị giác.

**Điều kiện để (a) đúng — xem P19 mục 3:** `home` phải được chốt **trong khung đã neo theo tag**.
Nếu chốt khi odom còn chưa neo thì toạ độ nhà thuộc một khung khác với khung sau khi neo, và cả RTH
lẫn hình GCS vẽ đều sai.

### 11.3 Năm mục GCS nêu khi duyệt bản 0.2 — Pi đã phân giải

| Mã | Phán quyết của Pi | Nằm ở mục |
|---|---|---|
| **P19** | **Nhận cả ba ý.** Ý 3 là **lỗi thật của Pi, đã kiểm và tái hiện đúng kịch bản GCS nêu**: RTH bay về `pad_home` chứ không về điểm cất cánh. Trả lời câu hỏi: odom **nhảy tức thì**, không trượt dần | 5.2b · 5.2 bit 0 |
| **P20** | **Nhận. Lỗi diễn đạt của Pi** ở 5.4 — "nằm trên đất" không suy ra `POS_VALID` = 0. Điều kiện phát là `POS_VALID` **và không gì khác**; ngân sách lấy theo trường hợp xấu nhất 6,5 kbit/s | 5.1 ý 2 · 5.4 |
| **P21** | Nhận. Thêm bit 8 `HOME_VALID` | 5.2 |
| **P22** | **Nhận. Lỗi của Pi**: bản 0.2 ghi `alt_m` "so với điểm cất cánh" ở hai chỗ, trong khi code là `tag_z + alt_m`. **Chú thích trong `MissionWaypoint.msg` cũng sai y như vậy** — sai từ trước khi có hợp đồng này | 8.3 · 3.2b #4 · 5.1 ý 3 |
| **P23** | Nhận. **Pi tự tính lại vectơ kiểm và khớp chính xác `0x6BDEA0A6`** | 8.6 · A15 |

**Không mục nào bị bác.** P19 ý 3, P20 và P22 là lỗi của Pi; ba lỗi đều thuộc loại *"tài liệu nói một
đằng, code làm một nẻo"* — cùng loại với ba lỗi P3/P4/P6 ở bản 0.1, và đều chỉ lộ ra khi có người đọc
kỹ từ phía bên kia.

**Một tương tác P19 × P20 Pi nêu thêm** (ghi ở cuối 5.2b, không cần GCS trả lời): vì drone không thấy
pad của chính nó khi đậu, `POS_VALID` sẽ **thường bằng 0 trước khi cất cánh**, nên cảnh báo "lệch
`pad_home`" của GCS sẽ thường rơi vào nhánh "chưa có vị trí". GCS đã lường nhánh đó nhưng nên biết nó
là nhánh thường gặp.

### 11.4 Không còn mục mở

| Mã | Mức | Trạng thái |
|---|---|---|
| P1–P17 | | phân giải ở 11.1, đã nhập thân tài liệu |
| P18 | Cao | **GCS chọn (a)**; điều kiện đúng đắn của (a) được xử lý ở P19 ý 3 |
| P19 | Cao | **Pi nhận cả ba ý** — 5.2b |
| P20 | TB | **Pi nhận** — 5.1, 5.4 |
| P21 | TB | **Pi nhận** — bit 8 `HOME_VALID` |
| P22 | TB | **Pi nhận** — `alt_m` so với tag đích |
| P23 | Thấp | **Pi nhận**, vectơ kiểm đã đối chiếu khớp |

**Cả 23 mục đã phân giải. Không còn câu hỏi nào chờ bên nào trả lời.**

Điều kiện lên **1.0** vẫn không đổi: đi qua toàn bộ phép kiểm **10.A**. Hợp đồng giữ **0.x** cho tới
lúc đó, kể cả khi còn sửa nghĩa — vì chưa byte nào chạy trên dây nên chưa có ai để mà phá tương thích.


## Phụ lục A — Tra cứu nhanh

| Cần gì | Xem |
|---|---|
| Cổng, địa chỉ, NAT | 2.1 |
| `sysid`/`compid` | 2.2 |
| Nạp kế hoạch: chuỗi bắt tay | 3.2 |
| Mã lỗi khi từ chối kế hoạch | 3.3 |
| Lệnh GCS gửi được | 4.1 |
| Nhịp phát từng bản tin | 5.1 |
| Cờ hiệu lực — trường nào đang vô nghĩa | 5.2 |
| Lệch phiên bản thì làm gì | 6.2 |
| Bắt gói, đếm mất gói | 7.2, 7.4 |
| Bảo mật | 7.6 |
| Số ID còn trống | 8.1, 8.2 |
| Cái gì đang là cấu hình chết | 9.2 |
| Truy vết đề xuất hai bên, mục còn mở | 11 |

## Phụ lục B — Liên kết

| Tài liệu | Vai trò |
|---|---|
| `GIAO_UOC_FC_ROS2.md` | hợp đồng đường FC↔Pi. **Nguồn của R1–R5 và quy trình mở rộng** |
| `thiet_ke_kien_truc_node_ros2.md` mục 1, 4.5 | vì sao chọn phương án (b) — 4G gắn thẳng Pi |
| `nhat_ky_lam_viec.md` | lịch sử phép đo; mục 10.1 về failsafe pin, 11.x về gripper |
| `config/comms.yaml` | tham số phía Pi |
| `config/tags.yaml` | bản đồ tag — nguồn vị trí duy nhất khi không có GPS |
| `docs/mavlink/drone_gcs.xml` | **chưa có** — việc đầu tiên ở mục 9.1 |

## Lịch sử phiên bản

| Bản | Ngày | Sửa gì |
|---|---|---|
| 0.1 | 2026-09-15 | Bản thảo đầu. Toàn bộ là [ĐỀ XUẤT], chờ GCS phản hồi. Chốt: MAVLink 2/UDP, Pi gọi ra trước, dialect riêng `42000+` thay vì mượn `MISSION_ITEM_INT` (3.1), cờ hiệu lực (5.2), hàng đợi ưu tiên (5.3), chữ ký gói (7.6), sổ đăng ký (8), nghiệm thu A/B/C (10) |
| 0.1 | 2026-09-16 | **GCS phản hồi, không tăng số.** Đổi nhãn [ĐỀ XUẤT] → [THOẢ THUẬN] ở 2.1, 2.2, 3.1, 3.3, 5.3, 6.2, 7.3, 7.6. Thêm ghi chú `GCS → 11.Pn` tại chỗ và mục 11 (17 đề xuất, 6 mức cao: P1 vị trí/tư thế, P2 `tagmap_crc`, P3 R2 vs `<extensions/>`, P4 chống trùng lệnh, P5 lệnh khẩn, P8 bảng enum). Không sửa nội dung gốc của Pi |
| **0.2** | **2026-09-16** | **Pi phân giải toàn bộ 17 đề xuất của GCS, nhập vào thân tài liệu; mục 11 thành bảng truy vết.** Ba lỗi của bản 0.1 được sửa: **R2** (lẫn *trailing-zero trimming* với `CRC_EXTRA` — P3), **4.2** (chống trùng lệnh tự triệt tiêu vì `confirmation` tăng — P4), **7.4** (`HEARTBEAT` không có trường thời gian → `TIMESYNC` — P6). Nhận: `LOCAL_POSITION_NED`+`ATTITUDE` 5 Hz có điều kiện (P1), `tagmap_crc` **công thức sửa lại** theo dữ liệu Pi thật có (P2), lệnh khẩn phát lại vô hạn (P5), bảng enum 8.5 (P8), 193 (P7), `status_flags`+3 trường (P9), `RSSI_VALID` và bỏ `fc_connected` (P10), địa chỉ NAT theo gói hợp lệ gần nhất (P12), `PARAM_*` chỉ đọc 9.4 (P13), chia đoạn `STATUSTEXT` (P15), ngữ nghĩa `ACK`/`ARM` (P16), `UINT32_MAX` (P17). Trả lời 4 câu về cấu trúc kế hoạch (3.2b) — câu 3 làm lộ lỗi `ACTION_NONE` trong `mission_fsm.py`. Thêm phép kiểm A13–A17, C5. **Pi nêu P18** (nhà của RTH ≠ home trên bản đồ GCS) — chờ GCS |
| 0.2 | 2026-09-16 | **GCS duyệt bản 0.2, không tăng số.** Chấp nhận toàn bộ phân giải 11.1 (gồm công thức `tagmap_crc` sửa lại). Trả lời P18: chọn (a), GCS vẽ nhà RTH từ `home_*` và cảnh báo trước `MISSION_START`. Nêu P19–P23 (11.3): khung trước/sau khi odom neo và `POS_VALID`, điều kiện phát 32/30, bit `HOME_VALID`, ba cách hiểu `alt_m`, làm tròn và vectơ kiểm CRC `0x6BDEA0A6`. Sửa chữ không đổi nghĩa: câu cụt ở 2.1; "thêm vào cuối" còn sót ở 6.1, 8.3, 8.4 → "sau `<extensions/>`"; "bốn lệnh" ở 6.2, 7.3, Phụ lục A (4.1 nay có 6 lệnh; 6.2 là bốn lệnh **khẩn**); bỏ ghi chú GCS đã phân giải ở 10.A; xếp lại bảng lịch sử theo thời gian |
| **0.3** | **2026-09-16** | **Pi phân giải 5 mục GCS nêu khi duyệt 0.2; không còn mục mở.** Ba lỗi nữa của Pi được sửa: **P19 ý 3** — `home` chốt trong khung chưa neo nên **RTH bay về `pad_home` thay vì điểm cất cánh** (Pi tái hiện đúng kịch bản GCS nêu); **P20** — "nằm trên đất" không suy ra `POS_VALID` = 0, ngân sách phải lấy xấu nhất 6,5 kbit/s; **P22** — `alt_m` là so với **tag đích**, không phải điểm cất cánh (chú thích `MissionWaypoint.msg` cũng sai y vậy). Nhận: `POS_VALID` = đã neo (5.2b), N/E là trục bản đồ tag chứ không phải Bắc/Đông địa lý, bit 8 `HOME_VALID` (P21), `round()` + vectơ kiểm `0x6BDEA0A6` **Pi đã đối chiếu khớp** (P23). Trả lời: odom **nhảy tức thì** khi neo. Pi nêu tương tác P19×P20: `POS_VALID` thường = 0 khi đậu vì drone không thấy pad của chính nó. Thêm việc 9–11 vào danh sách 9.1 |
| 0.3 | 2026-09-16 | **GCS duyệt bản 0.3, không tăng số. GCS chấp nhận toàn bộ; bắt đầu triển khai phía GCS.** Sửa chữ không đổi nghĩa: dòng trạng thái đầu tài liệu còn ghi "chờ Pi P19–P23"; nhóm byte của vectơ kiểm 8.6 (bản ghi đầu là `uint16` rồi ba `int32`, giá trị không đổi); xếp lại bảng lịch sử theo thời gian |
