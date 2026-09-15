"""tagmap - CRC ban do tag, thuan Python de pytest duoc khong can ROS.

Muc dich (giao uoc GCS <-> Pi muc 8.6): phat hien ban do tag cua hai ben LECH NHAU. Vi tri
waypoint suy tu tags.yaml tren Pi, con GCS dat tag bang trang thiet ke khu vuc. Hai ban lech thi
drone bay cho khac cho GCS ve, va phep kiem vung cam cua GCS sai ma khong ai biet.
ERR_UNKNOWN_TAG chi bat THIEU tag, khong bat SAI VI TRI.

Khong dua yaw / size / kind vao CRC, va day la ly do:
  - tags.yaml khong khai yaw, va yaw tuyet doi cua FC hien vo nghia vi tu ke chua hieu chuan
    (giao uoc FC 10.6a) - dua vao CRC la dua vao mot so khong ai tin;
  - size nam o apriltag.yaml, va size cua pad_a dang GIA DINH 0,122 m chua do bang thuoc
    (no nhat ky 6.2 #5) - dua mot phong doan vao CRC la khoa cung no mai mai;
  - kind (home/pickup/dropoff) la khai niem lap ke hoach cua GCS, khong phai du lieu Pi.
Them size_mm la viec MINOR sau khi do pad_a.
"""

import struct
import zlib

BAN_GHI = '<Hiii'          # uint16 tag_id, int32 n_mm, int32 e_mm, int32 d_mm = 14 byte
CO_BAN_GHI = struct.calcsize(BAN_GHI)


def doc_known_tags(flat):
    """[id, x, y, z, ...] (ENU met, dung dinh dang tags.yaml) -> {id: (x, y, z)}."""
    if len(flat) % 4:
        raise ValueError(f'known_tags phai co boi so cua 4 phan tu, dang co {len(flat)}')
    return {int(flat[i]): (float(flat[i + 1]), float(flat[i + 2]), float(flat[i + 3]))
            for i in range(0, len(flat), 4)}


def tagmap_crc(tags):
    """tags: {id: (x, y, z)} theo ENU met -> CRC-32 IEEE (giao uoc muc 8.6).

    Ban ghi little-endian (tag_id, n_mm, e_mm, d_mm), sap theo tag_id TANG DAN.
    Doi ENU -> NED: n = y, e = x, d = -z. Dung he NED cho khop LOCAL_POSITION_NED o muc 5.1 -
    MOT he quy chieu duy nhat tren ca kenh, khong de hai he cung ton tai.

    round() cua Python (lam tron nua ve so chan), KHONG dung int(): int() cat cut nen 0,1229 m
    thanh 122 mm o mot ben va 123 mm o ben kia la du de CRC lech mai mai.
    """
    buf = b''
    for tag_id in sorted(tags):
        x, y, z = tags[tag_id]
        buf += struct.pack(BAN_GHI, tag_id, round(y * 1000), round(x * 1000), round(-z * 1000))
    return zlib.crc32(buf)
