# Camera OV9281 — Thông tin & hướng dẫn triển khai

Tài liệu ghi lại toàn bộ thông số đã kiểm chứng trên máy hiện tại và các bước cần
làm để đưa camera này sang máy khác (Raspberry Pi 5, Jetson Orin Nano).

Cập nhật: 2026-09-03 · Máy tham chiếu: Raspberry Pi 4 Model B Rev 1.2

---

## 1. Nhận dạng phần cứng

| Hạng mục | Giá trị | Cách xác minh |
|---|---|---|
| Sensor | OmniVision **OV9281** | Chip ID `0x9281` đọc từ thanh ghi `0x300A/0x300B` |
| Loại | Mono (đơn sắc), **global shutter** | ISP trả về ảnh xám, không có ma trận Bayer |
| Độ phân giải gốc | 1280 × 800 (1 MP) | `rpicam-hello --list-cameras` |
| Độ sâu bit | 8-bit (`R8`) và 10-bit (`R10_CSI2P`) | như trên |
| Địa chỉ I²C | **0x60** (7-bit) | `i2cdetect -y 10` |
| Driver kernel | `ov9282` (mainline, dùng chung cho OV9281/OV9282) | `modinfo ov9282` |
| Compatible string | `ovti,ov9281` | device-tree node |
| Ống kính | Góc siêu rộng (fisheye), lấy nét cố định | quan sát ảnh chụp |
| EEPROM nhận dạng | **Không có** | quét sạch dải 0x50–0x57, trống |

### Về hãng sản xuất (OEM)

**Không thể xác định bằng phần mềm.** Module không gắn EEPROM nên không lưu bất kỳ
mã nhà sản xuất nào đọc được. Muốn biết hãng phải nhìn chữ in trên mặt sau board.

Tuy nhiên điều này **không ảnh hưởng gì đến việc sử dụng**: board tuân thủ đúng
thiết kế tham chiếu của Raspberry Pi (I²C 0x60, 2 lane MIPI, clock 24 MHz lấy từ
Pi, nguồn lấy từ chân regulator của đầu CSI), nên overlay `ov9281` có sẵn trong
Raspberry Pi OS chạy thẳng — **không cần driver riêng của hãng** (khác với các
module kiểu Arducam Pivariety phải cài driver độc quyền).

Ngoài sensor ở 0x60, trên bus còn một thiết bị ở **0x70** chưa xác định (nhiều khả
năng là I²C switch trên module). Không cần đụng tới.

---

## 2. Thông số giao tiếp MIPI CSI-2

Đây là các con số cần khai báo lại khi viết device-tree cho máy khác:

| Tham số | Giá trị |
|---|---|
| Số data lane | **2** (`data-lanes = <1 2>`) |
| Clock lane | 0 |
| Link frequency | **400 MHz** (`400000000`) |
| Tốc độ mỗi lane | 800 Mbps (DDR) — tổng 1.6 Gbps |
| External clock (XCLK/MCLK) | **24 MHz**, do máy chủ cấp qua chân CSI |
| Nguồn | AVDD / DOVDD / DVDD lấy từ regulator của đầu CSI |

> **Quan trọng:** sensor **không có dao động thạch anh riêng** — nó phụ thuộc vào
> clock 24 MHz từ máy chủ. Nếu chưa nạp overlay/driver để bật clock, sensor sẽ
> **không phản hồi trên I²C**. Đừng vội kết luận camera hỏng khi `i2cdetect` trống.

---

## 3. Tốc độ khung hình đã đo thực tế

Đo trên Pi 4 bằng `camera_view.py`:

| Chế độ | FPS lý thuyết | FPS đo được |
|---|---|---|
| 1280×800 `R8` | 143.7 | 30.0 (mặc định libcamera) |
| 1280×720 `R8` | 171.8 | — |
| 640×400 `R8` | 309.8 | **119.1** (đặt `--fps 120`) |
| 1280×800 `R10_CSI2P` | 114.9 | — |
| 640×400 `R10_CSI2P` | 247.8 | — |

Mặc định libcamera chạy 30 FPS. Muốn nhanh hơn **phải chỉ định rõ** `--fps`.
FPS thực tế còn bị giới hạn bởi thời gian phơi sáng: muốn 120 FPS thì exposure
phải ≤ 8.3 ms, nên cần đủ sáng hoặc phải tăng gain.

---

## 4. Cấu hình Raspberry Pi 4 (máy hiện tại — ĐÃ XONG)

### 4.1 Vì sao camera không tự nhận

`camera_auto_detect=1` chỉ dò được vài sensor của Raspberry Pi (ov5647, imx219,
imx477, imx708…). **OV9281 không nằm trong danh sách** → không overlay nào được
nạp → không có `/dev/video0`, `rpicam-hello` báo "No cameras available!".

### 4.2 Sửa `/boot/firmware/config.txt`

```ini
camera_auto_detect=0
dtoverlay=ov9281
```

Áp dụng ngay không cần reboot:

```bash
sudo dtoverlay ov9281
```

Tham số bổ sung của overlay (`dtoverlay=ov9281,<param>=<val>`):

| Tham số | Ý nghĩa |
|---|---|
| `rotation` | Góc lắp sensor: `0` hoặc `180` (mặc định 0) |
| `orientation` | `0`=front, `1`=rear, `2`=external (mặc định 2) |
| `cam0` | Dùng đầu CSI0 thay vì CSI1 (cho Compute Module / Pi 5) |
| `media-controller` | Bật/tắt Media Controller API (mặc định on) |

Ví dụ lắp camera ngược: `dtoverlay=ov9281,rotation=180`

### 4.3 Kiểm tra

```bash
rpicam-hello --list-cameras          # phải thấy: 0 : ov9281 [1280x800 10-bit MONO]
sudo i2cdetect -y 10                 # 0x60 hiện "UU" = driver đã chiếm
lsmod | grep -E 'ov9282|unicam'      # cả hai module phải nạp
sudo dmesg | grep -i ov928
```

### 4.4 Phần mềm

```bash
sudo apt install -y rpicam-apps python3-picamera2 python3-opencv
```

Đã cài trên máy này: `libcamera 0.3.2`, `rpicam-apps 1.5.2`, `picamera2 0.3.27`,
`OpenCV 4.6.0`, `numpy 1.24.2`.

> **Cảnh báo:** cài OpenCV bằng **apt**, không dùng `pip install opencv-python`.
> Bản pip kéo theo numpy 2.x sẽ làm hỏng `picamera2` (vốn build với numpy 1.24).

### 4.5 Ubuntu trên Pi 4 (không phải Raspberry Pi OS)

Ubuntu **không có** `rpicam-apps` / `python3-picamera2` trong apt — dùng libcamera thuần:

```bash
sudo apt install -y libcamera-tools libcamera-v4l2 libcamera-ipa \
                    python3-libcamera python3-opencv i2c-tools v4l-utils
```

`scripts/install.sh` tự làm việc này khi phát hiện `ID=ubuntu`.

> **`libcamera-ipa` là gói dễ bị bỏ sót nhất.** Nó chứa `ipa_rpi_vc4.so` (IPA cho
> pipeline vc4 của Pi 4) và các file tuning trong `/usr/share/libcamera/ipa/rpi/vc4/`
> (OV9281 dùng `ov9281_mono.json`). Nếu **thiếu** gói này:
>
> - kernel vẫn probe sensor bình thường (`i2cdetect -y 10` thấy `UU` ở `0x60`,
>   `media-ctl -p -d /dev/media0` thấy entity `ov9281 10-0060` nối vào `unicam-image`),
> - nhưng `cam -l` **rỗng** kèm log:
>   ```
>   WARN  IPAManager  No IPA found in '/usr/lib/aarch64-linux-gnu/libcamera'
>   ERROR RPI pipeline_base.cpp:804 Failed to load a suitable IPA library
>   ERROR RPI vc4.cpp:206 Failed to register camera ov9281 10-0060: -22
>   ```
>
> Sửa: `sudo apt install -y libcamera-ipa` rồi chạy lại `cam -l` (không cần reboot).
> Khi đã đúng, `cam -l` in `1: 'ov9281' (/base/soc/i2c0mux/i2c@1/ov9281@60)`.
>
> Các dòng `Symbol ipaModuleInfo not found` / `v4l2-compat.so: IPA module has no
> valid info` và `No static properties available for 'ov9281'` là **nhiễu vô hại**,
> vẫn còn sau khi sửa — bỏ qua.

Ubuntu không có `python3-picamera2` qua apt: nếu cần Picamera2 thì `pip install picamera2`
(dựa trên `python3-libcamera` vừa cài), hoặc đọc camera qua OpenCV / GStreamer
`libcamerasrc`, hoặc build `camera_ros` từ source.

---

### 4.6 Đọc thẳng V4L2 (bỏ qua libcamera) — đường chạy được trên Ubuntu + ROS 2

OV9281 là **mono global-shutter**, không có ma trận Bayer → không cần ISP, không cần debayer,
không cần IPA. Đọc thẳng `/dev/video0` của `bcm2835-unicam` là đường đơn giản và ổn định nhất
trên Ubuntu (xem mục 9 để biết vì sao `camera_ros`/libcamera không dùng được ở đây).

```bash
./scripts/camera_v4l2_setup.sh --exposure 1200 --gain 100
```

Script làm 2 việc, **cả hai đều reset sau mỗi lần reboot**:

**a) Ép định dạng subdev cho khớp với thứ ứng dụng sẽ xin.**
`bcm2835-unicam` **không đổi độ sâu bit**. Sensor mặc định `Y10_1X10` (10-bit); nếu ứng dụng xin
`GREY` (8-bit) thì stream vẫn chạy đủ FPS **nhưng mọi khung hình toàn số 0**.

```bash
media-ctl -d /dev/media0 --set-v4l2 '"ov9281 10-0060":0[fmt:Y8_1X8/1280x800]'
```

Sensor hỗ trợ đúng 2 mã: `MEDIA_BUS_FMT_Y8_1X8` (0x2001) và `MEDIA_BUS_FMT_Y10_1X10` (0x200a).
Cặp khớp: `Y8_1X8` ↔ `GREY`, `Y10_1X10` ↔ `Y10P`.

**b) Đặt exposure / gain bằng tay.** Không có IPA thì **không có auto-exposure** — sensor giữ
nguyên giá trị lần chạy trước, thường rất thấp → ảnh đen. Lưu ý các control này nằm trên
**subdev của sensor**, không phải trên `/dev/video0` (nên `v4l2-ctl -d /dev/video0 --list-ctrls`
trả về rỗng, và `v4l2_camera` in `Available controls:` trống — đó là bình thường):

```bash
v4l2-ctl -d /dev/v4l-subdev0 --list-ctrls
v4l2-ctl -d /dev/v4l-subdev0 --set-ctrl exposure=1200,analogue_gain=100
```

| Control | Dải | Ghi chú |
|---|---|---|
| `exposure` | 1 … *(tuỳ vblank)* (đơn vị: dòng) | Trần = `height + vertical_blanking` trừ lề |
| `analogue_gain` | 16 … 255 | 16 = 1.0× |
| `vertical_blanking` | 110 … 51540 (mặc định driver 1022) | **Quyết định FPS.** Xem mục c |
| `horizontal_blanking` | 250 … 31487 | Thường để yên |
| `pixel_rate` | 200 MHz (chỉ đọc) | Dùng để tính thời gian dòng |

**c) `vertical_blanking` quyết định FPS — đây là nút thắt lớn nhất.**

```
thời gian 1 khung = (height + vertical_blanking) × (width + horizontal_blanking) / pixel_rate
```

Sau khi libcamera từng chạy, `vertical_blanking` hay bị bỏ lại ở giá trị rất lớn (đo được 3085)
→ chỉ còn ~23 FPS. Hạ xuống mức tối thiểu là được tốc độ thật của sensor:

```bash
./scripts/camera_v4l2_setup.sh --vblank 110 --exposure 800 --gain 120
```

FPS **đo thực tế** qua `ros2 topic hz /image_raw` trên Pi 4 (mono8, đường V4L2 trực tiếp):

| Độ phân giải | `vertical_blanking` | FPS trần lý thuyết | **FPS đo được** |
|---|---|---|---|
| 1280×800 | 3085 (sót lại từ libcamera) | 23.7 | **22.8** |
| 1280×800 | 1022 (mặc định driver) | 50.6 | — |
| 1280×800 | **110** (tối thiểu) | 101.3 | **~95** |
| 640×400 | **110** | 256.3 | **~246** |

> **Thứ tự đặt control rất quan trọng:** driver chốt trần `exposure` theo `height + vertical_blanking`.
> Phải hạ `vblank` **trước** rồi mới đặt `exposure`, nếu không exposure sẽ bị cắt **im lặng**.
> `camera_v4l2_setup.sh` làm đúng thứ tự này và cảnh báo nếu giá trị bị cắt.
> Hệ quả: vblank càng thấp → FPS càng cao nhưng trần phơi sáng càng ngắn → cần nhiều sáng hơn
> hoặc phải tăng `analogue_gain`. Ở `vblank=110`, trần exposure chỉ còn 885 dòng (≈ 9.6 ms).

**Chẩn đoán ảnh đen — phân biệt 2 nguyên nhân bằng thống kê pixel:**

```bash
v4l2-ctl -d /dev/video0 --set-fmt-video=width=1280,height=800,pixelformat=GREY \
         --stream-mmap --stream-count=5 --stream-to=/tmp/ov9281.raw
python3 -c "
import numpy as np; n=1280*800
f=np.fromfile('/tmp/ov9281.raw',dtype=np.uint8)[-n:].reshape(800,1280)
print('min',f.min(),'max',f.max(),'mean',round(float(f.mean()),1),'std',round(float(f.std()),1))"
```

| Kết quả | Nghĩa là |
|---|---|
| `min=0 max=0` tuyệt đối | **Sai format** — subdev và video node lệch độ sâu bit. Chạy `camera_v4l2_setup.sh` |
| `mean≈16`, `std<2` | Format đúng, nhưng **không có ánh sáng** (16 là black level). Tháo nắp ống kính / bật đèn / tăng exposure-gain |
| `std` vài chục, `max` gần 255 | ✅ Ảnh thật, phơi sáng hợp lý |

Mẹo phân biệt nhanh: đổi `analogue_gain` mà **chỉ có `std` (nhiễu) tăng còn `mean` đứng yên** thì
đúng là không có ánh sáng — gain đang khuếch đại nhiễu chứ không có tín hiệu để khuếch đại.

**Xuất ra topic ROS 2:**

```bash
sudo apt install -y ros-jazzy-v4l2-camera ros-jazzy-image-transport-plugins

# Gộp cả 2 bước (cấu hình V4L2 rồi khởi động node) trong một lệnh:
./scripts/run_camera_node.sh --vblank 110 --exposure 800 --gain 120
```

`scripts/run_camera_node.sh` gọi `camera_v4l2_setup.sh` trước, chỉ khởi động node nếu bước cấu hình
thành công, tự `source` ROS nếu chưa, và `exec` sang node để Ctrl+C ăn thẳng. Tương đương với:

```bash
./scripts/camera_v4l2_setup.sh --vblank 110 --exposure 800 --gain 120
ros2 run v4l2_camera v4l2_camera_node --ros-args \
  -p video_device:=/dev/video0 -p pixel_format:=GREY \
  -p output_encoding:=mono8 -p image_size:="[1280,800]"
```

`output_encoding:=mono8` là quan trọng: mặc định của `v4l2_camera` là `rgb8`, gây cảnh báo
`performing possibly slow conversion: mono8 => rgb8` và nhân ba băng thông một cách vô ích.

Đo được trên Pi 4: `/image_raw` ra `1280x800`, `encoding=mono8`, `step=1280`, **~95 Hz**
(hoặc **~246 Hz** ở 640×400) — với điều kiện đã hạ `--vblank`. Xem bảng ở mục c.

Dòng `Unable to open camera calibration file [.../unicam.yaml]` là **bình thường** khi chưa hiệu
chỉnh — node vẫn publish ảnh, chỉ `camera_info` là rỗng. Chỉ cần hiệu chỉnh thật (`camera_calibration`
với bàn cờ) khi làm ArUco/AprilTag đo pose 3D.

### 4.7 Hiệu chỉnh camera trên máy KHÔNG có GUI

`ros2 run camera_calibration cameracalibrator` **bắt buộc mở cửa sổ đồ hoạ** (xem bàn cờ, bấm
CALIBRATE/SAVE). Máy này là Ubuntu Server thuần: không có desktop, `DISPLAY` rỗng → chạy thẳng
qua SSH thường là **không được**. Ba đường đi:

| Cách | Ưu | Nhược |
|---|---|---|
| **A. Script không GUI** (`scripts/calibrate_camera.py`) | Chạy thẳng qua SSH, không cần cài gì thêm | Không xem được ảnh trực tiếp, phải dựa vào lưới phủ in ra terminal |
| **B. GUI qua X11 forwarding** (`ssh -X`, MobaXterm/VcXsrv/XQuartz) | Dùng đúng công cụ chuẩn của ROS, thấy ảnh trực tiếp | Máy khách phải có X server; qua Wi-Fi thì giật |
| **C. Ghi rosbag rồi hiệu chỉnh trên laptop** | Chuẩn ROS, thoải mái làm lại | Phải có ROS 2 trên laptop, file bag nặng |

**Khuyến nghị dùng cách A** — companion computer của drone vốn luôn headless.

#### A0. Chuẩn bị bàn cờ

**Sinh file in** (script tự tính kích thước, cảnh báo nếu tràn giấy):

```bash
./scripts/calibrate_camera.py pattern --size 9x6 --square 0.025 --out chessboard.svg
# khổ lớn hơn cho dễ chụp xa:
./scripts/calibrate_camera.py pattern --size 9x6 --square 0.035 --paper a3 --out chessboard_a3.svg
```

Mặc định ra bàn cờ **10×7 ô, ô 25 mm** trên **A4 ngang** (250×175 mm, lề trắng còn 23.5×17.5 mm).

**`--size` đếm SỐ GÓC BÊN TRONG, không đếm ô.** Đây là lỗi phổ biến nhất:

```
bàn cờ 10 x 7 ô   ->   9 x 6 góc trong   ->   --size 9x6
```

Vì sao 10×7 chứ không phải 10×10: một chiều **chẵn**, một chiều **lẻ** thì bàn cờ không đối xứng
xoay, OpenCV mới xác định được hướng duy nhất. Chọn hai chiều cùng chẵn hoặc cùng lẻ sẽ gây nhập
nhằng 180°.

**In:**

1. Mở file `.svg` bằng trình duyệt → Print.
2. Đặt tỉ lệ **100% / "Actual size"**. **Tuyệt đối không chọn "Fit to page"** — nó co ảnh lại và
   mọi số đo sau đó đều sai theo.
3. Chọn giấy **mờ (matte)**, đừng dùng giấy bóng — phản quang làm mất góc.

**Kiểm tra sau khi in — bước không được bỏ qua:**

File có sẵn một **thước 100 mm** ở lề dưới. Lấy thước thật đo lại:

- Đúng 100 mm → dùng `--square 0.025` như bình thường.
- Lệch (ví dụ ra 96 mm) → **đừng in lại**, chỉ cần đo cạnh một ô thật bằng thước rồi truyền số
  đo đó vào `--square`. Sai số ở `--square` đi **thẳng** vào sai số khoảng cách: `--square` lệch
  4% thì mọi khoảng cách đo được cũng lệch 4%.

Đo chính xác hơn: đo trọn **8 ô liền nhau** rồi chia 8, thay vì đo 1 ô.

**Gắn lên đế cứng:**

| Yêu cầu | Vì sao |
|---|---|
| Dán **phẳng tuyệt đối** lên bìa cứng / ván ép / mica | Thuật toán giả định bàn cờ là **mặt phẳng lý tưởng**. Giấy cong vênh vài mm là hỏng toàn bộ kết quả |
| Dán kín mép, không bong bóng | Chỗ phồng làm lệch vị trí góc |
| **Giữ nguyên vành trắng** quanh bàn cờ | OpenCV cần vành trắng mới dò được ô ngoài cùng. Đừng cắt sát mép ô đen |
| Bề mặt mờ, không bóng | Tránh loá đèn |

Dùng keo xịt hoặc băng keo hai mặt dán toàn bộ mặt sau. Đừng chỉ dán 4 góc.

**Chọn kích thước ô cho đúng camera này:** ống kính fisheye ~120°, `fx` ≈ 380 px. Bề rộng bàn cờ
trên ảnh: `w = fx × B / Z` với `B` là bề ngang bàn cờ.

| Bàn cờ | Khoảng cách 20 cm | 30 cm | 50 cm |
|---|---|---|---|
| A4, ô 25 mm (B=250 mm) | 475 px (37% khung) | 317 px (25%) | 190 px (15%) |
| A3, ô 35 mm (B=350 mm) | 665 px (52%) | 443 px (35%) | 266 px (21%) |

Nhắm cho bàn cờ chiếm khoảng **25–50% bề ngang khung hình**. Với A4 thì làm việc ở **20–35 cm**;
muốn đứng xa thoải mái hơn thì in A3.

#### A. Chạy hiệu chỉnh (không cần GUI)

```bash
# DỪNG node camera trước — /dev/video0 chỉ cho một tiến trình mở
./scripts/camera_v4l2_setup.sh --vblank 110 --exposure 800 --gain 120

# 1) Chụp: cầm bàn cờ đi khắp khung hình, script tự lọc ảnh hợp lệ và in lưới phủ
./scripts/calibrate_camera.py capture --size 9x6 --square 0.025

# 2) Tính và ghi thẳng ra file camera_info của ROS
./scripts/calibrate_camera.py solve --size 9x6 --square 0.025 --write-ros
```

`--square` phải là **số đo thật sau khi in**, tính bằng mét (25 mm → `0.025`).

**Cách di chuyển bàn cờ khi chụp** — mục tiêu là làm cho các ảnh càng **khác nhau** càng tốt:

- Đưa bàn cờ tới **4 góc khung hình** và 4 cạnh, không chỉ ở giữa. Ống kính fisheye méo mạnh nhất
  ở rìa, mà thiếu dữ liệu ở rìa thì hệ số méo tính ra sẽ tồi.
- **Nghiêng** bàn cờ trái/phải/trên/dưới khoảng 30–45°. Ảnh chụp bàn cờ song song với cảm biến
  hầu như không cho thêm thông tin — nghiêng mới tách được `fx` khỏi khoảng cách.
- Đổi **khoảng cách**: vài tấm gần, vài tấm xa.
- Xoay bàn cờ quanh trục quang vài tấm.
- Giữ **yên tay lúc chụp** — mờ nhoè làm góc lệch. Đủ sáng để `--exposure` ngắn.

Script chỉ nhận ảnh khi tâm bàn cờ đã dịch đủ xa các ảnh trước (mặc định 60 px, đổi bằng
`--min-move`) nên bạn cứ di chuyển liên tục, nó tự lọc.

Bước `capture` chỉ nhận ảnh khi tâm bàn cờ đã dịch đủ xa các ảnh trước (mặc định 60 px) để bộ ảnh
có độ đa dạng, và in lưới 6×6 cho biết đã phủ vùng nào:

```
    . . # # . .
    . # # # # .
    . # # # # #
    # # # # # .
    . # # # . .
    . . # . . .
    phủ 21/36 ô lưới
```

Nhắm phủ càng kín càng tốt, **đặc biệt là 4 góc** — ống kính fisheye méo mạnh nhất ở rìa, mà đó
chính là chỗ marker hay xuất hiện khi bay.

Bước `solve` in ra `fx/fy/cx/cy`, hệ số méo, góc nhìn ước lượng, sai số tái chiếu (RMS) và 3 ảnh
tệ nhất. Đọc RMS: **< 0.5 px là tốt**, 0.5–1.0 tạm được, **> 1.0 nên chụp lại**.

`--write-ros` ghi ra `~/.ros/camera_info/unicam.yaml` (tự sao lưu bản cũ thành `.bak`).
`v4l2_camera` đọc file này tự động — khởi động lại node là hết cảnh báo
`Unable to open camera calibration file`.

> **Bẫy OpenCV giống hệt mục 4.6:** `cv2.VideoCapture` mặc định tự thương lượng format và xin
> YUYV → lệch với subdev đang ở `Y8_1X8` → **đọc ra khung hình toàn số 0**. Bắt buộc phải ép:
> ```python
> cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"GREY"))
> cap.set(cv2.CAP_PROP_CONVERT_RGB, 0)
> ```
> Script đã làm sẵn, nhưng nếu bạn tự viết node OpenCV thì phải nhớ.

#### B. Dùng GUI chuẩn của ROS qua X11 forwarding (MobaXterm / VcXsrv / XQuartz)

Phía Pi đã sẵn sàng: `X11Forwarding yes` trong `sshd_config`, có `xauth`, và OpenCV bản apt
(`python3-opencv` 4.6) được build kèm **GUI Qt5** → `cameracalibrator` mở được cửa sổ qua SSH.

**1) Máy Windows — MobaXterm**

- Biểu tượng **X server** ở góc trên-phải phải đang **bật (xanh)**; chưa bật thì bấm vào để bật.
- Session → SSH → *Advanced SSH settings* → tick **X11-Forwarding** (nên tick cả **Compression**).
- Mở session rồi kiểm tra đường X11 trước khi làm gì khác:
  ```bash
  echo $DISPLAY        # phải ra dạng localhost:10.0 — RỖNG là forwarding chưa chạy
  xeyes                # cửa sổ đôi mắt hiện trên Windows -> đường X11 thông
  ```
  Trên Linux/macOS thì thay bằng `ssh -Y -C pc@<ip-cua-pi>` (macOS cần XQuartz).

**2) Terminal 1 — chạy node camera**, đúng độ phân giải sẽ dùng khi bay:

```bash
cd ~/ros2_ws
./scripts/run_camera_node.sh --width 1280 --height 800 --vblank 110 --exposure 1200 --gain 100
```

**3) Terminal 2 (cũng phải là session có X11 forwarding) — mở cameracalibrator:**

```bash
source /opt/ros/jazzy/setup.bash
cp -n ~/.ros/camera_info/unicam.yaml ~/.ros/camera_info/unicam.yaml.pre-gui   # giữ bản cũ
ros2 run camera_calibration cameracalibrator \
  --size 9x6 --square 0.025 -k 3 \
  --ros-args \
  -r image:=/image_raw \
  -r /camera/set_camera_info:=/v4l2_camera/set_camera_info
```

> **Remap `set_camera_info` là bắt buộc.** Node hiệu chỉnh gọi service `camera/set_camera_info`,
> còn `v4l2_camera` lại advertise `/v4l2_camera/set_camera_info`. Thiếu dòng remap thì node báo
> `no camera service available` rồi thoát ngay lúc khởi động; nếu thêm `--no-service-check` để
> bỏ qua thì nút **COMMIT** cuối cùng vẫn vô tác dụng. Đã kiểm chứng trên máy này: có remap thì
> log in ra `Waiting for service camera/set_camera_info ... OK`.

**4) Thao tác trong cửa sổ**

Cầm bàn cờ đã in (mục A0) đi khắp khung hình. 4 thanh bên phải phải chạy đầy sang xanh:

| Thanh | Cách làm đầy |
|---|---|
| **X** | đưa bàn cờ sang trái ↔ phải, tới sát mép |
| **Y** | đưa lên trên ↔ xuống dưới |
| **Size** | đưa lại gần ↔ ra xa (chiếm gần hết khung ↔ còn nhỏ) |
| **Skew** | nghiêng/xoay bàn cờ so với mặt phẳng ảnh |

- Nút **CALIBRATE** sáng khi đủ dữ liệu → bấm rồi chờ 10–60 s. Pi 4 tính khá lâu, **cửa sổ đứng
  hình lúc này là bình thường**, đừng tắt.
- **SAVE** → ghi `/tmp/calibrationdata.tar.gz` (ảnh + kết quả, để dành hiệu chỉnh lại offline).
- **COMMIT** → gọi service, `v4l2_camera` ghi thẳng `~/.ros/camera_info/unicam.yaml`. Khởi động
  lại node là hết cảnh báo `Unable to open camera calibration file`.

**5) Nếu quá giật**

Mỗi khung 1280×800 mono ≈ 1 MB phải chui qua đường X11. Wi-Fi yếu thì hạ bước 2 xuống
`--width 640 --height 400` — nhưng nhớ khi bay cũng phải chạy đúng 640×400 (xem "Lưu ý chung").

**Lỗi hay gặp**

| Hiện tượng | Xử lý |
|---|---|
| `echo $DISPLAY` rỗng | X server của MobaXterm chưa bật, hoặc session chưa tick X11-Forwarding |
| `qt.qpa.xcb: could not connect to display` | như trên; hoặc do chạy bằng `sudo` nên mất `~/.Xauthority` → **không** chạy calibrator với sudo |
| `Could not load the Qt platform plugin "xcb"` dù `$DISPLAY` có giá trị | `export QT_XKB_CONFIG_ROOT=/usr/share/X11/xkb` rồi chạy lại |
| `no camera service available` | thiếu dòng remap `set_camera_info`, hoặc node camera ở terminal 1 chưa chạy |
| Cửa sổ mở nhưng ảnh đen | `ros2 topic hz /image_raw`; không có nhịp hoặc ảnh toàn số 0 → xem mục 4.6 |

#### Lưu ý chung cho mọi cách

- **Hiệu chỉnh gắn với độ phân giải.** Calibrate ở 1280×800 rồi chạy 640×400 là **sai** — `fx`,
  `fy`, `cx`, `cy` co theo tỉ lệ. Hãy hiệu chỉnh ở đúng độ phân giải sẽ dùng khi bay.
- Đổi ống kính hoặc chỉnh nét lại → phải hiệu chỉnh lại. Đổi exposure/gain thì không sao.
- Kiểm chứng nhanh kết quả: `Z = fx × S / w`. Đặt marker cạnh `S` mét ở khoảng cách đã biết, xem
  bề rộng `w` pixel trên ảnh có khớp không.

---

## 5. Chuyển sang Raspberry Pi 5

Gần như giống hệt Pi 4, có 3 điểm khác:

**a) Cáp.** Pi 5 dùng đầu FPC **22 chân** (nhỏ hơn), module này nhiều khả năng
dùng đầu **15 chân**. Cần **cáp chuyển 15↔22 chân** (loại "Raspberry Pi Camera
Cable — Standard to Mini"). Đây là lỗi hay gặp nhất khi chuyển máy.

**b) Chọn đầu cắm.** Pi 5 có 2 đầu camera:

```ini
camera_auto_detect=0
dtoverlay=ov9281,cam0      # đầu CAM0
# hoặc
dtoverlay=ov9281           # đầu CAM1 (mặc định)
```

Cắm cả 2 camera thì khai báo 2 dòng.

**c) Tầng thu khác.** Pi 5 dùng driver `rp1-cfe` thay cho `bcm2835-unicam`.
libcamera/picamera2 che hết khác biệt này nên **mã Python không cần sửa gì**.
Chỉ khác nếu bạn thao tác trực tiếp V4L2 — số hiệu `/dev/videoN` sẽ khác.

Chạy chương trình y hệt:

```bash
python3 camera_view.py --width 640 --height 400 --fps 120
```

---

## 6. Chuyển sang Jetson Orin Nano

**Đây là phần khó nhất — không cắm-là-chạy như Raspberry Pi.** Hãy dự trù thời gian.

### 6.1 Ba việc bắt buộc

**a) Cáp và đầu cắm.** Orin Nano Devkit dùng đầu FPC 22 chân. Cần cáp chuyển
15→22 chân. Lưu ý **thứ tự chân CSI của Jetson khác Raspberry Pi** — phải dùng
đúng cáp cho Jetson, không dùng lại cáp của Pi.

**b) Driver kernel.** L4T/JetPack **không kèm sẵn** driver OV9281. Có 3 hướng:

1. *Mua module đã hỗ trợ Jetson* (Arducam, Leopard Imaging, e-con) — họ cấp sẵn
   driver + overlay dạng gói `.deb`. **Đây là hướng nhanh và ít rủi ro nhất.**
2. *Port driver mainline `ov9282.c`* vào cây nguồn kernel Jetson rồi build lại.
   Driver có sẵn ở `drivers/media/i2c/ov9282.c`, cần thêm khai báo vào
   `tegra-camera-platform` trong device tree.
3. *Dùng board chuyển đổi* có sẵn hỗ trợ.

**c) Device tree.** Phải viết overlay khai báo cho tầng VI của Tegra, dùng đúng
các số ở **mục 2**: `data-lanes = <1 2>`, link frequency 400 MHz, MCLK 24 MHz,
I²C address `0x60`, cùng khối `mode0` mô tả `pixel_phase = "y"` (mono),
`csi_pixel_bit_depth = "10"`, `active_w = 1280`, `active_h = 800`.

### 6.2 Đừng dùng `nvarguscamerasrc`

Argus (ISP của NVIDIA) được thiết kế cho sensor Bayer màu và cần file hiệu chỉnh
ISP riêng. **Sensor mono như OV9281 thường không chạy được qua Argus.**

Hướng đúng là **đọc thẳng V4L2**, bỏ qua ISP — điều này hoàn toàn ổn vì ảnh mono
không cần khử Bayer, cân bằng trắng hay chỉnh màu:

```bash
python3 camera_view.py --backend v4l2 --device /dev/video0 \
        --width 1280 --height 800
```

Kiểm tra trước bằng công cụ hệ thống:

```bash
v4l2-ctl --list-devices
v4l2-ctl -d /dev/video0 --list-formats-ext     # cần thấy 'GREY' hoặc 'Y10 '
v4l2-ctl -d /dev/video0 --set-fmt-video=width=1280,height=800,pixelformat=GREY \
         --stream-mmap --stream-count=30        # test luồng thô
```

Nếu vendor có cấp ISP config và Argus chạy được thì mới dùng:

```bash
python3 camera_view.py --backend gstreamer --width 1280 --height 800 --fps 60
```

### 6.3 Phần mềm trên Jetson

```bash
sudo apt install -y python3-opencv v4l-utils
```

JetPack đã kèm OpenCV. **Không có `picamera2` trên Jetson** — đó là thư viện
riêng của Raspberry Pi. Vì vậy `camera_view.py` mới tách backend ra.

---

## 7. Bảng đối chiếu nhanh 3 nền tảng

| | Raspberry Pi 4 | Raspberry Pi 5 | Jetson Orin Nano |
|---|---|---|---|
| Cắm là chạy | ✅ (chỉ cần sửa config.txt) | ✅ (chỉ cần sửa config.txt) | ❌ cần driver + DT |
| Đầu cáp | 15 chân | 22 chân (cần cáp chuyển) | 22 chân (cáp riêng của Jetson) |
| Driver | `ov9282` có sẵn | `ov9282` có sẵn | phải tự thêm |
| Tầng thu | `bcm2835-unicam` | `rp1-cfe` | Tegra VI |
| Backend Python | `picamera2` | `picamera2` | `v4l2` |
| Lệnh chạy | `--backend auto` | `--backend auto` | `--backend v4l2` |

---

## 8. Chương trình đi kèm

[`camera_view.py`](camera_view.py) — đọc và hiển thị ảnh bằng OpenCV, tự chọn
backend theo máy.

```bash
python3 camera_view.py                                  # tự dò backend
python3 camera_view.py --width 640 --height 400 --fps 120
python3 camera_view.py --headless --frames 200          # không cần màn hình
python3 camera_view.py --backend v4l2 --device /dev/video0
```

Phím tắt: `q` / `ESC` thoát · `s` lưu ảnh vào `captures/`

### 8.1 Cạm bẫy `ScalerCrop` (quan trọng khi dùng picamera2)

Khi chạy picamera2 mà **không đặt rõ** control `ScalerCrop`, libcamera có thể áp
giá trị **nhỏ nhất** của control thay vì giá trị mặc định:

```
ctrl ScalerCrop = (min, max, default)
                = ((0,0,64,64), (0,0,1280,800), (0,0,1280,800))
                     ^^^^^^^^^^ bị lấy giá trị này
```

Kết quả: ISP cắt đúng **64×64 pixel ở góc trên-trái** rồi phóng to lên
1280×800 — ảnh ra chỉ là một mảng xám nhoè, mất hoàn toàn khung hình.

`rpicam-still` / `rpicam-vid` **không dính lỗi này** vì rpicam-apps luôn đặt
`ScalerCrop` tường minh. Nên nếu `rpicam-hello` cho ảnh đúng mà mã Python của bạn
cho ảnh sai thì gần như chắc chắn là lỗi này.

Cách sửa (đã áp dụng trong `camera_view.py`):

```python
cam.configure(cfg)
crop = cam.camera_properties["ScalerCropMaximum"]   # (0, 0, 1280, 800)
cam.start()
cam.set_controls({"ScalerCrop": crop})
```

Chương trình in ra vùng cắt thực tế mỗi lần khởi động để bạn kiểm tra ngay:

```
[i] Vùng cắt ISP (ScalerCrop): (0, 0, 1280, 800)     <- đúng
[i] Vùng cắt ISP (ScalerCrop): (0, 0, 64, 64)        <- sai, ảnh sẽ hỏng
```

Lưu ý khi port sang máy khác: đây là đặc thù của **libcamera/picamera2**, nên
Raspberry Pi 5 cũng dính. Jetson đi đường V4L2 trực tiếp thì **không** gặp.

Chương trình tự xử lý các tình huống: chạy qua SSH không có `DISPLAY` (tự đẩy
cửa sổ ra màn hình vật lý `:0`, hoặc chuyển sang chế độ headless in FPS ra
terminal), và ảnh 1 kênh của sensor mono (tự đổi sang BGR để vẽ HUD).

---

## 9. Xử lý sự cố

| Hiện tượng | Nguyên nhân & cách xử lý |
|---|---|
| `No cameras available!` | Chưa nạp overlay. Kiểm tra `config.txt`, chạy `sudo dtoverlay ov9281` |
| `cam -l` rỗng + `Failed to load a suitable IPA library` / `No IPA found` / `Failed to register camera ov9281 10-0060: -22` (Ubuntu) | Thiếu gói **`libcamera-ipa`** (không có `ipa_rpi_vc4.so`). `sudo apt install -y libcamera-ipa` rồi chạy lại `cam -l`. Xem mục 4.5. Sensor đã được kernel nhận đúng — đây thuần là lỗi userspace |
| `Symbol ipaModuleInfo not found` / `v4l2-compat.so: IPA module has no valid info` | Nhiễu vô hại của libcamera (quét nhầm `v4l2-compat.so`). Bỏ qua — không phải nguyên nhân camera không chạy |
| **`ros2 topic hz` báo nhịp bình thường nhưng ảnh toàn số 0** (`min=0 max=0`) | Subdev sensor còn ở `Y10_1X10` trong khi app xin `GREY` (8-bit). `bcm2835-unicam` không đổi độ sâu bit. Chạy `./scripts/camera_v4l2_setup.sh`. Xem mục 4.6. **Cảnh báo: `topic hz` không phát hiện được lỗi này** — phải kiểm tra thống kê pixel |
| Ảnh có `mean≈16`, `std<2`, đổi gain chỉ thấy nhiễu tăng | Format đúng nhưng **không có ánh sáng** (16 = black level). Tháo nắp ống kính, bật đèn, tăng `exposure`/`analogue_gain`. Xem mục 4.6 |
| `v4l2-ctl -d /dev/video0 --list-ctrls` rỗng / `v4l2_camera` in `Available controls:` trống | Bình thường — control exposure/gain nằm trên **subdev sensor** (`/dev/v4l-subdev0`), không phải trên video node |
| `camera_ros`: `FATAL control_serializer.cpp:626 ... requires a ControlInfoMap` → `Failed to call start: -110` | `ros-jazzy-libcamera` 0.7.1 chạy IPA **isolated** qua IPC và crash. Không sửa được từ phía ứng dụng — dùng đường V4L2 trực tiếp (mục 4.6) |
| `camera_ros`: `FATAL ipa_base.cpp:396 assertion "it != buffers_.end()" failed in prepareIsp()` | libcamera 0.2.0 + `camera_ros` 0.5.2: IPA thiếu buffer bayer/embedded chưa được map. Cũng không sửa được từ ứng dụng — dùng mục 4.6 |
| `error: 'libcamera::ControlList::MergePolicy' has not been declared` khi build `camera_ros` | Source `camera_ros` ≥ 0.6.0 cần libcamera ≥ 0.3, Ubuntu 24.04 chỉ có 0.2.0. Dùng tag `0.5.2` |
| `Unable to open camera calibration file [.../unicam.yaml]` | Bình thường khi chưa hiệu chỉnh. Node vẫn publish ảnh; chỉ cần hiệu chỉnh thật khi làm ArUco/AprilTag đo pose |
| `performing possibly slow conversion: mono8 => rgb8` | Thêm `-p output_encoding:=mono8` cho `v4l2_camera_node` |
| Đường V4L2 chỉ đạt ~23 FPS ở 1280×800 | `vertical_blanking` bị libcamera bỏ lại ở 3085. Hạ về tối thiểu: `--vblank 110` → ~95 FPS. Xem mục 4.6c |
| Đặt `--exposure` cao nhưng giá trị thực bị cắt | Trần exposure = `height + vertical_blanking` trừ lề. Muốn phơi sáng lâu thì phải **tăng** `--vblank` (đổi lại giảm FPS). Phải đặt vblank trước exposure |
| `i2cdetect` không thấy 0x60 | Bình thường **khi chưa nạp overlay** — sensor chưa được cấp clock 24 MHz. Nạp overlay rồi quét lại bus `i2c-10` |
| 0x60 hiện `UU` | **Đúng rồi** — nghĩa là driver đã chiếm địa chỉ này |
| **Ảnh chỉ hiện một góc, phóng to nhoè** | Control `ScalerCrop` của ISP bị lấy giá trị **nhỏ nhất** (64×64 ở góc trên-trái) thay vì mặc định. Phải đặt rõ `ScalerCrop = ScalerCropMaximum` sau khi `start()`. Xem mục 8.1 |
| Ảnh đen thui | Thiếu sáng, hoặc chưa tháo nắp ống kính. Sensor mono nhạy hồng ngoại — thử đèn IR |
| Chỉ đạt 30 FPS | Mặc định của libcamera. Phải truyền `--fps` rõ ràng |
| Đặt FPS cao nhưng không đạt | Exposure quá dài. Cần sáng hơn hoặc giảm exposure/tăng gain |
| `ModuleNotFoundError: picamera2` trên Jetson | Đúng như dự kiến — dùng `--backend v4l2` |
| `picamera2` lỗi sau khi cài pip | Do numpy 2.x. Gỡ `opencv-python` của pip, cài `python3-opencv` bằng apt |
| Cảnh báo `no sharpen algorithm` | Bình thường với sensor mono, không phải lỗi |
| Cáp lỏng / cắm ngược | Mặt tiếp xúc màu xanh của FPC quay về phía đúng; đẩy hết cỡ rồi khoá chốt |

---

## 10. Sao lưu

Bản `config.txt` trước khi sửa: `/boot/firmware/config.txt.bak-20260903-172840`
