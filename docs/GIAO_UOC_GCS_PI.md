# Giao ước GCS ↔ Pi — kênh nhiệm vụ và telemetry

**Hợp đồng liên thông giữa trạm mặt đất (GCS) và ROS 2 trên Raspberry Pi 4, qua 4G/LTE.**

| | |
|---|---|
| Phiên bản hợp đồng | **0.1** |
| Ngày | 2026-09-15 |
| Trạng thái | **Bản thảo — GCS đã phản hồi (2026-09-16), chờ Pi trả lời mục 11.** Chưa mục nào [CHỐT] |
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

**Ở bản 0.1 này gần như mọi mục đều là [ĐỀ XUẤT]** — phía GCS chưa phản hồi. Nhãn được ghi ở
từng mục; chỗ nào không ghi thì hiểu là [ĐỀ XUẤT].

> **GCS (2026-09-16):** mục nào GCS đồng ý nguyên văn đã đổi nhãn sang **[THOẢ THUẬN]** (Pi đề
> xuất + GCS đồng ý = hai bên thống nhất, chưa chạy trên dây). Mục nào GCS đề nghị sửa **giữ
> nguyên [ĐỀ XUẤT]** và có ghi chú `GCS → 11.Pn` ngay tại chỗ; chi tiết ở **mục 11**. GCS **không
> sửa nội dung gốc** của Pi — Pi quyết định nhập hay bác từng đề xuất.

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

**Pi gọi ra trước, GCS trả lời về địa chỉ nguồn nó thấy.** [ĐỀ XUẤT]

Đây là điểm bắt buộc phải làm đúng, không phải chi tiết hiện thực: modem 4G hầu như luôn nằm sau
NAT của nhà mạng (thường là CGNAT), nên **GCS không thể chủ động mở kết nối tới Pi**. Pi phải phát
`HEARTBEAT` 1 Hz tới `gcs_host:gcs_port` ngay khi khởi động, kể cả khi chưa nhận được gì; GCS ghi
lại địa chỉ nguồn của gói đầu tiên và gửi mọi thứ về đúng địa chỉ đó.

Kéo theo hai ràng buộc:

1. **GCS phải có điểm cuối ổn định** — IP công cộng tĩnh, DNS động, hoặc cả hai bên nằm trong một
   VPN. Đây là yêu cầu hạ tầng, không phải việc của code.
2. Ánh xạ NAT hết hạn nếu im lặng (thường 30–120 s). `HEARTBEAT` 1 Hz giữ cho nó sống. **Không
   được tắt heartbeat để tiết kiệm băng thông** — mất ánh xạ NAT là mất đường về.

> **GCS → 11.P12:** đồng ý Pi gọi ra trước. Riêng "địa chỉ nguồn của gói **đầu tiên**" sẽ mất
> đường về khi IP 4G đổi. Đề nghị: theo gói **hợp lệ gần nhất** (chữ ký đúng) từ `(1, 191)`; Pi
> nhận/gửi trên **cùng một socket** bind 14551.

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

### 3.2 Chuỗi bắt tay [ĐỀ XUẤT]

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

> **GCS → 11.P14:** đồng ý chuỗi bắt tay và các ngưỡng; cần Pi trả lời 4 câu về **một mục kế hoạch
> nghĩa là gì** (có tự về home không, `action = NONE` có hạ không…) thì giới hạn 16 điểm mới có nghĩa.

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

### 4.1 Dùng `COMMAND_LONG` chuẩn [ĐỀ XUẤT]

Ở đây **có** bản tin chuẩn phù hợp nên dùng chuẩn: `COMMAND_LONG` (76) + `COMMAND_ACK` (77). Được
luôn quy tắc **R4 của hợp đồng FC — mọi lệnh đều phải có đường trả lời** — mà không phải tự chế.

| `MAV_CMD` | Tên | Ý nghĩa trên kênh này |
|---|---|---|
| 300 | `MISSION_START` | = gọi service `~/start`: cất cánh và chạy kế hoạch đã nạp |
| 20 | `NAV_RETURN_TO_LAUNCH` | yêu cầu RTH — đổi trạng thái FSM, **không** gửi xuống FC |
| 21 | `NAV_LAND` | hạ cánh tại chỗ (= `~/land`) |
| 400 | `COMPONENT_ARM_DISARM` | chỉ chấp nhận **disarm**: `param1 = 0`. `param2 = 21196` = cắt ở mọi độ cao |
| **42100** | `DRONE_ABORT_MISSION` | huỷ kế hoạch đang nạp/đang chạy, hạ cánh, về `IDLE` |

`COMMAND_ACK.result` dùng giá trị chuẩn: `ACCEPTED` (0), `TEMPORARILY_REJECTED` (1), `DENIED` (2),
`UNSUPPORTED` (3), `FAILED` (4). **Lệnh lạ → `UNSUPPORTED`, không bao giờ im lặng.**

> **GCS → 11.P7:** thiếu tạm dừng/tiếp tục — đề nghị cấp số `MAV_CMD_DO_PAUSE_CONTINUE` (193).
> **GCS → 11.P16:** ghi rõ `ACCEPTED` = FSM đã nhận và bắt đầu chuyển (không phải đã xong);
> `ARM` (`param1 = 1`) trả `DENIED`.

### 4.2 Phát lại và tính bất biến [ĐỀ XUẤT]

GCS phát lại `COMMAND_LONG` cho tới khi có `COMMAND_ACK`, mỗi lần **tăng trường `confirmation`**
(cơ chế chuẩn). Pi phải **bất biến với lệnh trùng**: cùng một lệnh nhận hai lần thì trạng thái chỉ
đổi một lần, và lần thứ hai vẫn trả `ACK`.

| Quy tắc | Giá trị |
|---|---|
| GCS chờ `ACK` | 1,0 s rồi phát lại |
| Số lần phát lại | 3, rồi báo cho người vận hành |
| Pi nhớ lệnh đã xử lý | theo `(command, confirmation)` trong 5 s |

**Lệnh không bao giờ được xếp sau telemetry** — xem hàng đợi ưu tiên mục 5.3.

> **GCS → 11.P4 (cao):** bản phát lại luôn có `confirmation` khác, nên nhớ theo
> `(command, confirmation)` **không bao giờ bắt được lệnh trùng**; A7 dùng cùng `confirmation` nên
> không lộ lỗi. Đề nghị bất biến **theo trạng thái FSM**.
> **GCS → 11.P5 (cao):** lệnh khẩn (20, 21, 400, 42100) phát lại mỗi 0,5 s **không giới hạn** tới
> khi có `ACK`, thay vì bỏ cuộc sau 3 lần.

---

## 5. Pi → GCS: telemetry và log

### 5.1 Bảng phát [ĐỀ XUẤT]

| ID | Bản tin | Nhịp | Vai trò |
|---|---|---|---|
| 0 | `HEARTBEAT` | **1 Hz** | sống/chết, giữ ánh xạ NAT. **Không bao giờ tắt** |
| 42010 | `DRONE_TELEMETRY` | **2 Hz cố định** | trạng thái tổng hợp |
| 42011 | `DRONE_LINK_STATS` | **0,2 Hz** | sức khoẻ đường truyền, để gỡ lỗi (mục 7.4) |
| 253 | `STATUSTEXT` | theo sự kiện, **≤ 2 gói/s** | log người đọc được |
| 77 | `COMMAND_ACK` | theo sự kiện | trả lời mục 4 |
| 42002/42004 | `DRONE_MISSION_REQUEST` / `_ACK` | theo sự kiện | bắt tay mục 3 |

**`DRONE_TELEMETRY` phát đúng nhịp kể cả khi không có nguồn nào cập nhật** — nguồn thiếu thì để
giá trị mặc định và **hạ cờ hiệu lực tương ứng**. Đây là quy tắc R5 của hợp đồng FC: trạng thái
phải hỏi lại được, vì GCS có thể khởi động lại bất cứ lúc nào và sẽ lỡ mọi sự kiện trước đó.

> **GCS → 11.P1 (cao):** bảng phát **không có vị trí ngang và yaw** — GCS không vẽ được 3D, không
> giám sát được vùng cấm. Đề nghị thêm `LOCAL_POSITION_NED` (32) + `ATTITUDE` (30) ở **5 Hz**, gốc =
> gốc `tags.yaml`, chỉ phát khi `POS_VALID`.
> **GCS → 11.P6:** `rtt_ms` không đo được bằng `HEARTBEAT` — đề nghị `TIMESYNC` (111).

### 5.2 Cờ hiệu lực — phần dễ sai nhất của gói này [ĐỀ XUẤT]

`DRONE_TELEMETRY` mang bitmask `valid_flags`. **Bit = 0 nghĩa là KHÔNG BIẾT, không phải là 0.**

| Bit | Tên | Hạ cờ khi |
|---|---|---|
| 0 | `POS_VALID` | EKF chưa hội tụ / chưa thấy tag / `/odometry/filtered` quá hạn |
| 1 | `GLOBAL_POS_VALID` | **hiện luôn = 0: FC chưa có GPS** |
| 2 | `BATTERY_VALID` | **hiện luôn = 0: FC chưa gửi `BATTERY_STATUS`** |
| 3 | `FC_LINK_VALID` | `/mavros/state.connected` = false |
| 4 | `MARKER_VALID` | không bám marker nào |
| 5 | `GRIPPER_VALID` | `/gripper/status` quá hạn (node chưa chạy hoặc đã chết) |
| 6 | `EKF_HEALTHY` | `/ekf/health.healthy` = false |

Đây là quy tắc **R3 của hợp đồng FC** áp nguyên văn: *"Dữ liệu không tin cậy thì GẮN CỜ, đừng thay
bằng giá trị an toàn — `0` là một lời nói dối khác."*

Hai bit `GLOBAL_POS_VALID` và `BATTERY_VALID` hiện **luôn bằng 0** vì FC chưa gửi GPS và chưa gửi
pin. Ghi thẳng vào hợp đồng để phía GCS **không vẽ đồng hồ pin rồi hiển thị 0 %** — và để không ai
nhầm rằng failsafe pin đang bảo vệ mình (mục 9.2).

> **GCS:** đồng ý nguyên tắc — GCS sẽ không vẽ đồng hồ pin/GPS khi bit = 0.
> **GCS → 11.P10:** `gcs_rssi_dbm` "0 = chưa đọc được" tự vi phạm R3 (đề nghị thêm bit 7
> `RSSI_VALID`); `fc_connected` trùng nghĩa bit 3.

### 5.3 Hàng đợi ưu tiên [THOẢ THUẬN]

`gcs_link_node` đã có `PriorityQueue` với ba mức; hợp đồng chốt ngữ nghĩa:

| Mức | Nội dung | Khi nghẽn |
|---|---|---|
| **0** `EMERGENCY` | `COMMAND_ACK` của lệnh khẩn, `STATUSTEXT` mức ≥ `CRITICAL` | không bao giờ bỏ |
| **1** `MISSION` | bắt tay nạp kế hoạch, `DRONE_MISSION_ACK` | không bao giờ bỏ |
| **2** `TELEMETRY` | `DRONE_TELEMETRY`, `DRONE_LINK_STATS`, `STATUSTEXT` thường | **BỎ gói cũ, giữ gói mới nhất** |

Telemetry **bị bỏ chứ không xếp hàng**: một gói trạng thái cũ 5 giây thì vô giá trị, mà lại chiếm
băng thông của gói hiện tại. Chỉ giữ tối đa **1** gói telemetry chờ trong hàng đợi.

### 5.4 Ngân sách băng thông [ĐỀ XUẤT]

| Luồng | Cỡ gói (MAVLink 2, có chữ ký) | Nhịp | Băng thông |
|---|---|---|---|
| `HEARTBEAT` | ~34 B | 1 Hz | 34 B/s |
| `DRONE_TELEMETRY` | ~90 B | 2 Hz | 180 B/s |
| `DRONE_LINK_STATS` | ~45 B | 0,2 Hz | 9 B/s |
| **Tổng ở trạng thái nghỉ** | | | **≈ 0,23 kB/s ≈ 1,8 kbit/s** |

Rất nhẹ so với 4G. Ngân sách này tồn tại **không phải để tiết kiệm tiền** mà để giữ dư địa cho
lệnh khẩn đi ngay khi đường truyền xấu. **Không thêm luồng định kỳ nào vượt 2 Hz** mà không sửa
mục này trước.

> **GCS → 11.P1:** đúng tinh thần câu trên — đề nghị sửa mục này để cho phép hai luồng
> vị trí/tư thế 5 Hz (≈ 530 B/s có chữ ký); tổng mọi luồng vẫn < 6 kbit/s.

---

## 6. Phiên bản và tương thích

### 6.1 Số hợp đồng

`MAJOR.MINOR`, quy tắc tăng giống hợp đồng FC mục 10.1:

| Đổi gì | Tăng gì |
|---|---|
| Đổi nghĩa/dấu/đơn vị một trường đang dùng; bỏ một bản tin; đổi mã `MAV_CMD`; đổi ý nghĩa một bit `valid_flags` | **MAJOR** |
| Thêm bản tin; thêm trường vào cuối bản tin cũ; thêm mã lỗi; thêm bit `valid_flags` mới | **MINOR** |
| Sửa chính tả, thêm giải thích, thêm số đo | không tăng |

### 6.2 Hai bên biết nhau ở phiên bản nào [THOẢ THUẬN]

`DRONE_TELEMETRY.contract_ver` = `MAJOR × 10000 + MINOR × 100`. Bản 0.1 → **`100`**.

Nằm trong bản tin **định kỳ** chứ không phải bản tin bắt tay — đúng quy tắc R5: GCS restart thì
vẫn biết ngay, không phải hỏi lại.

**Quy tắc khi lệch `MAJOR`:**

- **Pi từ chối nạp kế hoạch**, trả `DRONE_MISSION_ACK` mã `ERR_CONTRACT`, vẫn tiếp tục phát
  telemetry và vẫn nhận lệnh khẩn (hạ cánh, RTH).
- **GCS hiển thị cảnh báo rõ ràng**, không cho người vận hành soạn nhiệm vụ.

Lý do vẫn nhận lệnh khẩn khi lệch MAJOR: lệnh hạ cánh là thứ **phải luôn đi được**, kể cả khi hai
bên hiểu khác nhau về mọi thứ khác. Bốn lệnh ở mục 4.1 vì vậy **không bao giờ được đổi nghĩa** —
đó là phần bất biến nhất của hợp đồng này.

### 6.3 Năm quy tắc tương thích

Kế thừa nguyên văn từ hợp đồng FC mục 10.2, vì chúng không phụ thuộc đường truyền:

**R1. Bản tin lạ, trường lạ, mã enum lạ → BỎ QUA im lặng, không crash.** Điều kiện để một bên nạp
bản mới trước bên kia.

**R2. Trường mới chỉ được THÊM vào cuối, không bao giờ đổi nghĩa trường cũ.** MAVLink 2 cắt bỏ
byte 0 ở cuối gói (*trailing-zero trimming*), nên thêm trường vào cuối là tương thích hai chiều:
bên cũ đọc gói mới thì bỏ qua phần thừa, bên mới đọc gói cũ thì thấy 0 ở trường mới. **Chèn trường
vào giữa sẽ phá vỡ mọi thứ một cách âm thầm.**

> **GCS → 11.P3 (cao):** quy tắc này **chỉ đúng khi trường mới nằm sau thẻ `<extensions/>`**.
> Viết thêm vào cuối danh sách trường thường thì `mavgen` sắp lại theo kích thước kiểu và
> **`CRC_EXTRA` đổi** → bên cũ loại **cả gói** vì sai CRC, không phải "bỏ qua phần thừa".

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

Một script `pymavlink` khoảng 150 dòng, làm đúng bốn việc: nạp kế hoạch từ YAML, gửi bốn lệnh ở
mục 4.1, in `DRONE_TELEMETRY` và `STATUSTEXT`, đếm gói mất.

Vai trò: **cả hai bên test ngược vào nó.** Pi test khi chưa có GCS thật; GCS test bằng cách so với
hành vi của nó. Nó cũng là đặc tả chạy được — khi tài liệu và nó lệch nhau thì ít nhất có chỗ để đo.

Đọc chung định dạng YAML với `send_mission_plan` đã có, nên một file kế hoạch chạy được cả hai
đường (nội bộ ROS và qua dây).

### 7.4 `DRONE_LINK_STATS` — biến "đường truyền tệ" thành con số [ĐỀ XUẤT]

| Trường | Nghĩa |
|---|---|
| `rx_ok` | gói MAVLink hợp lệ đã nhận |
| `rx_drop` | gói mất, suy từ khoảng trống của `seq` MAVLink |
| `rx_bad_crc` | gói sai CRC |
| `rx_bad_sig` | gói sai chữ ký (nếu bật mục 7.6) |
| `tx_sent` | gói đã gửi |
| `tx_dropped` | gói telemetry bị bỏ do nghẽn (mục 5.3) |
| `queue_depth` | độ sâu hàng đợi hiện tại |
| `rtt_ms` | vòng khứ hồi, đo bằng `HEARTBEAT` |

Không có bảng này thì mọi báo cáo sự cố đường truyền đều là *"thấy lag"*. Có nó thì phân biệt được
**mất gói** với **nghẽn hàng đợi** với **sai chữ ký** — ba nguyên nhân cần ba cách sửa khác nhau.

> **GCS → 11.P6:** `rtt_ms` đổi sang đo bằng `TIMESYNC`. **11.P17:** trường bên phát không đo
> được (vd. `rx_bad_crc` phía GCS) thì = `UINT32_MAX`.

`rx_drop` đếm được vì MAVLink có `seq` 1 byte tăng dần **theo từng `(sysid, compid)`**; đếm khoảng
trống là ra số gói mất. Đây là công cụ chẩn đoán có sẵn, không dùng thì lãng phí.

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

> **GCS → 11.P15:** chuỗi dài hơn 50 byte — đề nghị **chia đoạn** bằng `id`/`chunk_seq` (extension
> MAVLink 2) thay vì cắt; GCS ghép đoạn được.

### 7.6 Chữ ký gói — bắt buộc trước khi bay thật [THOẢ THUẬN]

**UDP thuần trên 4G công cộng nghĩa là bất cứ ai biết `IP:port` đều gửi được lệnh cho drone.** Bốn
lệnh ở mục 4.1 có cả `disarm` — tức là "cắt động cơ giữa không trung".

MAVLink 2 có **chữ ký gói** sẵn (`MAVLINK_SIGNING`, HMAC-SHA256 với khoá bí mật 32 byte chia sẻ
trước, kèm timestamp chống phát lại). Hợp đồng đề xuất:

| Hạng mục | Giá trị |
|---|---|
| Trạng thái | **[ĐỀ XUẤT]** — chưa hiện thực |
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
| 0 | `HEARTBEAT` | ↔ | [ĐỀ XUẤT] |
| 76 | `COMMAND_LONG` | GCS→Pi | [ĐỀ XUẤT] |
| 77 | `COMMAND_ACK` | Pi→GCS | [ĐỀ XUẤT] |
| 253 | `STATUSTEXT` | Pi→GCS | [ĐỀ XUẤT] |
| 42001 | `DRONE_MISSION_COUNT` | GCS→Pi | [ĐỀ XUẤT] |
| 42002 | `DRONE_MISSION_REQUEST` | Pi→GCS | [ĐỀ XUẤT] |
| 42003 | `DRONE_MISSION_ITEM` | GCS→Pi | [ĐỀ XUẤT] |
| 42004 | `DRONE_MISSION_ACK` | Pi→GCS | [ĐỀ XUẤT] |
| 42010 | `DRONE_TELEMETRY` | Pi→GCS | [ĐỀ XUẤT] |
| 42011 | `DRONE_LINK_STATS` | ↔ | [ĐỀ XUẤT] |

**Còn trống:** `42005–42009`, `42012–42099`. Chưa cấp cho ai.

### 8.2 `MAV_CMD`

| Dải | Dùng cho |
|---|---|
| `MAV_CMD` chuẩn | ưu tiên tuyệt đối — bốn lệnh ở mục 4.1 |
| **`42100–42149`** | lệnh riêng kênh này |

| Mã | Tên | Trạng thái |
|---|---|---|
| 42100 | `MAV_CMD_DRONE_ABORT_MISSION` | [ĐỀ XUẤT] |

**Còn trống:** `42101–42149`.

### 8.3 Trường của `DRONE_MISSION_ITEM` — ánh xạ sang `MissionWaypoint`

| Trường trên dây | Kiểu | `MissionWaypoint` | Ghi chú |
|---|---|---|---|
| `seq` | `uint8` | `seq` | 0 trở lên, liên tục. `uint8` để khớp `MissionWaypoint.seq` |
| `expected_marker_id` | `int32` | `expected_marker_id` | **nguồn vị trí duy nhất.** Phải có trong `tags.yaml` |
| `action` | `uint8` | `action` | 0 `NONE`, 1 `PICKUP`, 2 `DROPOFF` |
| `alt_m` | `float` | `alt_m` | m, so với điểm cất cánh |
| `acceptance_radius_m` | `float` | `acceptance_radius_m` | m, ngang |
| `max_vel_mps` | `float` | `max_vel_mps` | m/s, trong `(0; 1,9]` |
| `loiter_s` | `float` | `loiter_s` | s |

**KHÔNG chở `lat`, `lon`, `pos_ned`.** Ba trường đó có trong message ROS `MissionWaypoint` vì nó
mirror `mission_waypoint_t` phía GCS/FC, nhưng `mission_manager_node` **bỏ qua hoàn toàn** — vị trí
suy từ `expected_marker_id`. Chở chúng lên dây chỉ tạo ra ảo giác là chúng có tác dụng.

Đây là chỗ **hợp đồng cố tình hẹp hơn message ROS**. Khi nào có GPS thì thêm trường mới vào cuối
(MINOR, theo R2), đừng hồi sinh ba trường này.

> **GCS → 11.P8 (cao):** giá trị `action` và mọi hằng số `mission_state` / `gripper_state` /
> `failsafe_type` phải liệt kê ở đây và trong XML (`<enum>`), không chỉ tham chiếu hằng số ROS.
> **GCS → 11.P2 (cao):** vị trí suy từ `tags.yaml` nhưng GCS có trang thiết kế khu vực riêng — cần
> `tagmap_crc` để phát hiện hai bản đồ lệch nhau.

### 8.4 Trường của ba bản tin còn lại

Đủ để viết `drone_gcs.xml` mà không phải đoán. **Thứ tự trường là phần của hợp đồng** — MAVLink 2
chỉ cho thêm vào cuối (R2).

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

**`DRONE_TELEMETRY`** (42010, Pi→GCS, 2 Hz) — thứ tự trường xếp giảm dần theo kích thước để không
sinh byte đệm:

> **GCS → 11.P3:** `mavgen` **tự** sắp trường theo kích thước khi xếp lên dây và MAVLink không có
> byte đệm — thứ tự trong XML không quyết định vị trí trên dây. Thứ tự chỉ còn quan trọng với
> trường sau `<extensions/>`.
> **GCS → 11.P9:** đề nghị thêm `status_flags` (ARMED, PI_HAS_AUTHORITY, CARRYING), `wp_total`,
> `expected_marker_id`, `retry_count`, và `tagmap_crc` (11.P2). **11.P11:** ghi rõ `stamp_us` theo
> đồng hồ nào.

| Trường | Kiểu | Nguồn | Cờ hiệu lực |
|---|---|---|---|
| `stamp_us` | `uint64` | `stamp` | — |
| `mission_id` | `uint32` | `mission_id` | — |
| `contract_ver` | `uint32` | hằng số của Pi | — (mục 6.2) |
| `lat` | `int32` | `lat` × 10⁷ | `GLOBAL_POS_VALID` |
| `lon` | `int32` | `lon` × 10⁷ | `GLOBAL_POS_VALID` |
| `gcs_rssi_dbm` | `int32` | `gcs_rssi_dbm` | 0 = chưa đọc được |
| `marker_id_tracking` | `int32` | `marker_id_tracking` | `MARKER_VALID`; −1 = không bám |
| `alt_m` | `float` | `alt_m` | `POS_VALID` |
| `vel_ned` | `float[3]` | `vel_ned` | `POS_VALID`. **NED**, không phải FLU |
| `battery_pct` | `float` | `battery_pct` | `BATTERY_VALID` |
| `battery_v` | `float` | `battery_v` | `BATTERY_VALID` |
| `valid_flags` | `uint16` | (mới) | mục 5.2 |
| `mission_state` | `uint8` | `mission_state` | hằng số `MissionState` |
| `current_wp_index` | `uint8` | `current_wp_index` | — |
| `gripper_state` | `uint8` | `gripper_state` | `GRIPPER_VALID` |
| `failsafe_type` | `uint8` | `failsafe_type` | hằng số `FailsafeEvent`; 0 = bình thường |
| `fc_connected` | `uint8` | `fc_connected` | `FC_LINK_VALID` |

`vel_ned` dùng **NED** vì đó là hệ của `ODOMETRY` từ FC (hợp đồng FC mục 3.1), không phải FLU của
`position_controller_node`. Đây đúng loại chỗ hợp đồng FC gọi là *"phần dễ rơi máy bay nhất"* — hai
hệ quy chiếu cùng tồn tại trong một hệ thống, đổi dấu z. **Điền sai dấu ở đây thì GCS hiển thị drone
đang lên khi nó đang xuống.**

**`DRONE_LINK_STATS`** (42011, ↔, 0,2 Hz): tám trường ở mục 7.4, `uint32` cho các bộ đếm,
`uint16` cho `queue_depth` và `rtt_ms`.

---

---

## 9. Bảng trạng thái — cái gì chưa có

### 9.1 Phía Pi: `gcs_link_node` là khung rỗng

Đo ngày 2026-09-15: node lên, đăng ký đủ ba topic, **không mở socket nào** (`ss -ulnp` không thấy
14550/14551). Bốn hàm đều là thân rỗng: `on_rx` (và **không có gì gọi nó**), `pump_tx`,
`on_telemetry`, `check_watchdog`.

`telemetry_aggregator_node` đã nối dây đủ 6 nguồn và lưu lại, nhưng `publish_packet` cũng là thân
rỗng nên `/telemetry/outgoing` **chưa bao giờ phát**.

Việc phải làm, theo thứ tự:

1. Viết `docs/mavlink/drone_gcs.xml` theo mục 8.
2. `telemetry_aggregator_node.publish_packet` — **kèm `valid_flags`**, nên `TelemetryPacket` cần
   thêm hai trường `valid_flags` và `contract_ver`. Đây là sửa message ROS, phải làm trước khi
   hai bên viết code sinh từ XML.
3. `gcs_link_node`: socket, vòng nhận, bắt tay mục 3, lệnh mục 4, watchdog mục 9.2.
4. `tools/gcs_sim.py` (mục 7.3) — làm **song song**, không làm sau: không có nó thì không test được.

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
| A7 | Gửi cùng một lệnh 3 lần cùng `confirmation` | trạng thái đổi **đúng một lần**, cả 3 lần đều có `ACK` |
| A8 | `DRONE_TELEMETRY` khi chưa có nguồn nào | vẫn phát đúng 2 Hz; `BATTERY_VALID` = 0, `GLOBAL_POS_VALID` = 0 |
| A9 | Cắt `gcs_sim.py` | sau 5 s `/gcs_link/connected` = false; sau 10 s nữa `failsafe_monitor_node` báo `FS_LINK_LOST` |
| A10 | Bật lại `gcs_sim.py` | `/gcs_link/connected` = true, sự cố được gỡ |
| A11 | Làm nghẽn: chặn đường ra 10 s rồi mở | `tx_dropped` tăng, `queue_depth` **không tăng vô hạn**, lệnh khẩn vẫn đi ngay khi mở |
| A12 | Lệch `MAJOR` giả lập | `ERR_CONTRACT`; telemetry **vẫn phát**; lệnh hạ cánh **vẫn đi** |

> **GCS đề nghị** (chờ Pi): sửa **A7** gửi với `confirmation` = 0, 1, 2 (11.P4); thêm **A13** kiểm
> tương thích trường extension (11.P3); thêm **C5** đổi IP modem giữa chừng (11.P12).

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

---

## 11. Phản hồi của GCS cho bản 0.1 — [ĐỀ XUẤT từ GCS, chờ Pi]

*Phía GCS viết ngày 2026-09-16. Pi trả lời ở cột cuối bảng 11.3. Mục nào Pi nhập vào thân tài
liệu thì đánh dấu ở bảng đó và xoá ghi chú `GCS → 11.Pn` tương ứng; khi mọi dòng đã có trả lời thì
xoá cả mục 11.*

### 11.1 GCS đồng ý nguyên văn — đã đổi nhãn sang [THOẢ THUẬN]

2.1 (bảng vận chuyển), 2.2, 3.1, 3.3, 5.3, 6.2, 7.3, 7.6. Ngoài ra GCS đồng ý **nội dung** (không có
nhãn riêng để đổi) của 0.2–0.4, 1.1, 1.2, 6.1, 6.3 R1/R3/R4/R5, 6.4, 6.5, 7.1, 7.2, 8.1, 8.2, 9.2, 9.3.

Hệ quả phía GCS: bỏ ARM, TAKEOFF rời, GOTO, giữ vị trí tay, ghi tham số và dongle ESP-NOW; bỏ phương
án mavlink-router; sinh mã từ `docs/mavlink/drone_gcs.xml` của repo này, không chép tay.

### 11.2 Đề xuất chi tiết

Mức: **Cao** = chặn việc viết mã · **TB** = cần trước nghiệm thu 10.B · **Thấp** = làm rõ.

**P1 · Cao — Telemetry không có vị trí ngang và tư thế.** `DRONE_TELEMETRY` chỉ có `lat/lon` (luôn
không hợp lệ), `alt_m`, `vel_ned`. Không có `x/y` trong hệ bản đồ tag và `yaw` thì GCS mất ba chức năng
đã có: cảnh 3D, giám sát vùng cấm/vùng bay, kiểm chứng lệnh có hiệu lực. 2 Hz cũng quá thưa cho 3D
(GCS nội suy với trễ hiển thị 150 ms, cần ≥ 5 Hz). Đề xuất theo 6.4 bước 1 — dùng chuẩn:
`LOCAL_POSITION_NED` (32) và `ATTITUDE` (30), **5 Hz**.
- Hệ NED, **gốc = gốc `tags.yaml`** (cùng hệ với bản đồ khu vực GCS), không lấy điểm cất cánh làm gốc.
- Bản tin chuẩn không có chỗ cho cờ → quy tắc: **chỉ phát khi `POS_VALID` = 1**; không phát = không biết.
- Băng thông ≈ (28 + 12 + 13 B) × 2 × 5 Hz ≈ 530 B/s; sửa 5.4 cho phép ngoại lệ này.
- Định nghĩa `alt_m = −z` của `LOCAL_POSITION_NED` để hai số không thể lệch.

**P2 · Cao — Không có cơ chế phát hiện bản đồ tag lệch.** Vị trí waypoint suy từ `tags.yaml` trên Pi,
còn GCS đặt tag và vùng cấm bằng trang thiết kế khu vực. Hai bản lệch thì drone bay chỗ khác chỗ GCS
vẽ, kiểm tra vùng cấm của GCS sai mà không ai biết; `ERR_UNKNOWN_TAG` chỉ bắt **thiếu** tag, không bắt
**sai vị trí**. Đề xuất tối thiểu:
1. Thêm `tagmap_crc` (`uint32`) vào `DRONE_TELEMETRY` (sau `<extensions/>`, xem P3).
2. CRC-32 IEEE trên bản ghi little-endian `(uint16 tag_id, int32 n_mm, int32 e_mm, int32 d_mm,
   int16 yaw_cdeg, uint16 size_mm, uint8 kind)`, sắp theo `tag_id` tăng dần, chỉ tag đang bật;
   `kind`: 0 home, 1 pickup, 2 dropoff, 3 waypoint. (GCS đã hiện thực đúng công thức này.)
3. GCS **xuất `tags.yaml`** từ thiết kế khu vực → triển khai lên Pi; GCS **khoá nạp kế hoạch** khi CRC lệch.
4. Nạp bản đồ qua dây: để MINOR sau nếu cần.

*Hỏi Pi:* `tags.yaml` hiện có những trường nào (`yaw`, `size`, `kind`)? Thiếu thì thống nhất tập trường tính CRC.

**P3 · Cao — R2 và câu "xếp giảm dần để không sinh byte đệm" sai với cách MAVLink làm việc.**
- Thêm trường "vào cuối" chỉ tương thích khi trường nằm **sau `<extensions/>`**. Nếu không, `mavgen`
  sắp lại trường theo kích thước kiểu và `CRC_EXTRA` (tính trên mọi trường không phải extension) **đổi**
  → bên cũ loại **cả gói** vì sai CRC.
- `mavgen` tự sắp trường không phải extension theo kích thước khi xếp lên dây; MAVLink không có byte
  đệm. Thứ tự trong XML chỉ quan trọng với trường extension.

Đề xuất R2 mới: *"Trường mới chỉ được thêm sau thẻ `<extensions/>`, luôn ở cuối danh sách extension.
Không thêm/xoá/đổi kiểu/đổi tên trường nằm trước `<extensions/>`."* Thêm phép kiểm **A13**: sinh mã từ
XML bản N và N+1 (thêm một trường extension), hai bên đọc gói của nhau cả hai chiều.

**P4 · Cao — Chống trùng lệnh mâu thuẫn với phát lại.** 4.2: GCS phát lại và **tăng `confirmation`**;
Pi nhớ theo `(command, confirmation)` → bản phát lại luôn khác khoá, không bao giờ bị coi là trùng. A7
dùng cùng `confirmation` nên không lộ. Đề xuất **bất biến theo trạng thái**: mỗi lệnh đưa FSM tới một
trạng thái đích; nếu FSM đã ở hoặc đang chuyển tới trạng thái đó thì trả `ACCEPTED`, không làm gì thêm
(vd. `RTL` khi đang `RTH`; `MISSION_START` khi đang chạy đúng `mission_id`). Nếu vẫn muốn bộ nhớ: khoá
`(command, param1…7)` trong 5 s, **bỏ qua `confirmation`**. Sửa **A7**: `confirmation` = 0, 1, 2.

**P5 · Cao — Lệnh khẩn không được bỏ cuộc sau 3 lần.** Trên 4G chập chờn, `DISARM`/`LAND`/`RTL` dừng
sau 3 s là không chấp nhận được, và người vận hành không làm gì hơn máy được.

| Nhóm | Lệnh | GCS phát lại |
|---|---|---|
| Thường | 300, 193 (P7) | 1,0 s × 3 rồi báo |
| **Khẩn** | 20, 21, 400, 42100 | mỗi **0,5 s, không giới hạn** tới khi có `ACK` hoặc người vận hành huỷ; báo động sau lần 3 nhưng vẫn phát |

An toàn nhờ P4. Chu kỳ chờ `ACK` chỉnh lại theo số đo C3.

**P6 · TB — `rtt_ms` không đo được bằng `HEARTBEAT`** (không có trường thời gian). Dùng `TIMESYNC`
(111): mỗi bên gửi 0,2 Hz, bên kia trả ngay. Thêm 111 vào sổ 8.1.

**P7 · TB — Thiếu tạm dừng/tiếp tục.** Khác RTH: người vận hành thấy người/vật cản đi vào khu vực, muốn
drone đứng chờ rồi đi tiếp. Đề xuất `MAV_CMD_DO_PAUSE_CONTINUE` (193), `param1` 0 = dừng, 1 = tiếp;
giữ vị trí, không đổi `current_wp_index`. FSM chưa có thì trả `UNSUPPORTED` và GCS ẩn nút — nhưng
**cấp số ngay** trong 4.1. *Hỏi Pi:* `RETRY_LOITER` dùng lại được cho "giữ vô thời hạn" không?

**P8 · Cao — Thiếu bảng giá trị enum trên dây.** Theo 6.4 bước 3, giá trị phải nằm trong tài liệu/XML.
GCS đang dùng (đề nghị Pi đối chiếu hằng số ROS thật và sửa cho đúng):

| Enum | Giá trị GCS đang dùng |
|---|---|
| `mission_state` | 0 IDLE, 1 TAKEOFF, 2 ENROUTE, 3 MARKER_SEARCH, 4 PRECISION_LAND, 5 ACTUATE_GRIPPER, 6 RETRY_LOITER, 7 RTH, 8 EMERGENCY_LAND, 9 MISSION_COMPLETE, 10 FAILSAFE |
| `failsafe_type` | 0 NONE, 1 MARKER_TIMEOUT, 2 GRIP_CONFIRM_FAIL, 3 LINK_LOST, 4 LOW_BATTERY, 5 EKF_UNHEALTHY, 6 FC_COMM_LOST |
| `gripper_state` | 0 OPEN, 1 CLOSED, 2 MOVING, 3 ERROR |
| `action` | 0 NONE, 1 PICKUP, 2 DROPOFF |

*Hỏi Pi:* `NAV_LAND` (hạ tại chỗ, không tìm tag) báo `mission_state` nào?

**P9 · TB — `DRONE_TELEMETRY` thiếu trường để hiển thị và kiểm chứng.**

| Trường | Kiểu | Vì sao |
|---|---|---|
| `status_flags` | `uint16` | bit 0 `ARMED`, bit 1 `PI_HAS_AUTHORITY` (ch5/ch8), bit 2 `CARRYING`; hiệu lực theo `FC_LINK_VALID`. Không có `ARMED` thì GCS không kiểm chứng được `DISARM`; không có quyền Pi thì người vận hành không hiểu vì sao `MISSION_START` bị từ chối |
| `wp_total` | `uint8` | "chặng 3/7" |
| `expected_marker_id` | `int32` | tag đang **tìm** (khác đang bám); −1 = không tìm |
| `retry_count` | `uint8` | cảnh báo trước khi thành failsafe |

**P10 · Thấp — Trường tự vi phạm R3 / trùng nghĩa.** `gcs_rssi_dbm` "0 = chưa đọc được" → thêm bit 7
`RSSI_VALID`, đổi `int32` → `int16`. `fc_connected` trùng bit 3 `FC_LINK_VALID` → bỏ trường, hoặc định
nghĩa bit 3 = "`/mavros/state` còn tươi", trường = giá trị `connected`. `marker_id_tracking`: ghi rõ
"`MARKER_VALID` = 0 thì bỏ qua trường" để không phải nhớ hai quy ước.

**P11 · Thấp — `stamp_us` theo đồng hồ nào.** GCS dùng thời điểm nhận để xét độ tươi, không dùng
`stamp_us`. Ghi rõ: "đồng hồ hệ thống Pi, có thể lệch nếu chưa NTP; chỉ để log và sắp thứ tự".

**P12 · TB — Địa chỉ trả về sau NAT.** Theo "gói đầu tiên": modem nối lại/CGNAT đổi ánh xạ → GCS gửi về
địa chỉ cũ → mất đường về tới khi khởi động lại GCS. Theo "gói mới nhất" bất kỳ: một gói giả cướp được
đường về (không ra lệnh được nhờ chữ ký, nhưng cắt được lệnh khẩn). Đề xuất: *"GCS gửi về địa chỉ nguồn
của gói **hợp lệ gần nhất** từ `(1, 191)`; khi bật chữ ký, hợp lệ = chữ ký đúng. Pi nhận và gửi trên
**cùng một socket** bind 14551."* Thêm **C5**: tắt/bật dữ liệu di động giữa chừng → telemetry và lệnh
thông lại ≤ 5 s, không khởi động lại gì.

**P13 · Thấp (có thể MINOR sau) — Đối chiếu ngưỡng, chỉ đọc.** GCS so `system_config` với `safety.yaml`
(yêu cầu sẵn trong `thiet_ke_kien_truc_node_ros2.md`). Không vi phạm 1.1 vì chỉ đọc: `PARAM_REQUEST_LIST`
(21) → `PARAM_VALUE` (22); `PARAM_SET` → Pi bỏ qua + `STATUSTEXT` WARN. Tên ≤ 16 ký tự, vd.
`LOW_BATT_PCT`, `CRIT_BATT_PCT`, `LINK_LOST_S`, `MARKER_SRCH_S`, `MAX_RETRIES`, `TAKEOFF_ALT_M`, `ACCEPT_RAD_M`.

**P14 · TB — Một mục kế hoạch nghĩa là gì.** GCS sẽ đổi bộ lập kế hoạch sang "một mục cho mỗi điểm
dừng theo marker". Cần Pi trả lời:
1. Kế hoạch có **tự về home** ở cuối, hay GCS thêm mục cuối là marker home, `action = NONE`?
2. Có cần mục đầu là home không?
3. Một mục = bay tới → tìm marker → hạ chính xác → `action` → cất lại? `action = NONE` có hạ không?
4. `alt_m` là độ cao tới mục đó hay độ cao hành trình cả chặng?

**P15 · Thấp — `STATUSTEXT` > 50 byte.** Đề nghị **chia đoạn** bằng `id`/`chunk_seq` (extension MAVLink 2)
thay vì cắt — chuỗi lý do của `mission_manager_node` thường dài hơn 50. GCS ghép đoạn được.

**P16 · Thấp — Ngữ nghĩa `ACK` của lệnh chạy dài.** `ACCEPTED` cho 300/20/21 = FSM đã nhận và bắt đầu
chuyển, không phải đã xong; GCS theo dõi kết quả qua `mission_state`; không dùng `IN_PROGRESS`. `ARM`
(`param1 = 1`) trả `DENIED`, không trả `UNSUPPORTED`.

**P17 · Thấp — `DRONE_LINK_STATS` hai chiều.** GCS cũng phát, nhưng có trường không đo được (vd.
pymavlink không tách `rx_bad_crc`). Ghi rõ: *"trường bên phát không đo được thì = `UINT32_MAX`"*.

### 11.3 Bảng theo dõi

| Mã | Mức | Tóm tắt | Pi trả lời |
|---|---|---|---|
| P1 | Cao | `LOCAL_POSITION_NED` + `ATTITUDE` 5 Hz, gốc `tags.yaml` | |
| P2 | Cao | `tagmap_crc` trong telemetry, GCS khoá nạp khi lệch | |
| P3 | Cao | Sửa R2: trường mới chỉ sau `<extensions/>`; thêm A13 | |
| P4 | Cao | Bất biến theo trạng thái, bỏ nhớ theo `confirmation`; sửa A7 | |
| P5 | Cao | Lệnh khẩn phát lại không giới hạn | |
| P8 | Cao | Bảng giá trị enum vào tài liệu/XML | |
| P6 | TB | RTT bằng `TIMESYNC` | |
| P7 | TB | Cấp số `DO_PAUSE_CONTINUE` (193) | |
| P9 | TB | `status_flags`, `wp_total`, `expected_marker_id`, `retry_count` | |
| P12 | TB | Địa chỉ trả về theo gói hợp lệ gần nhất, cùng socket; thêm C5 | |
| P14 | TB | 4 câu hỏi về cấu trúc kế hoạch | |
| P10 | Thấp | `RSSI_VALID`, bỏ trùng `fc_connected` | |
| P11 | Thấp | Đồng hồ của `stamp_us` | |
| P13 | Thấp | Tham số chỉ đọc qua `PARAM_*` | |
| P15 | Thấp | Chia đoạn `STATUSTEXT` | |
| P16 | Thấp | Ngữ nghĩa `ACK`, `ARM` → `DENIED` | |
| P17 | Thấp | `UINT32_MAX` cho trường không đo được | |

---

## Phụ lục A — Tra cứu nhanh

| Cần gì | Xem |
|---|---|
| Cổng, địa chỉ, NAT | 2.1 |
| `sysid`/`compid` | 2.2 |
| Nạp kế hoạch: chuỗi bắt tay | 3.2 |
| Mã lỗi khi từ chối kế hoạch | 3.3 |
| Bốn lệnh GCS gửi được | 4.1 |
| Nhịp phát từng bản tin | 5.1 |
| Cờ hiệu lực — trường nào đang vô nghĩa | 5.2 |
| Lệch phiên bản thì làm gì | 6.2 |
| Bắt gói, đếm mất gói | 7.2, 7.4 |
| Bảo mật | 7.6 |
| Số ID còn trống | 8.1, 8.2 |
| Cái gì đang là cấu hình chết | 9.2 |
| Phản hồi của GCS cho bản 0.1 | 11 |

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
