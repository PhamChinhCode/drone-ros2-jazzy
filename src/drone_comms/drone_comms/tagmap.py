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

Goc WGS84 cua ban do (giao uoc 0.7, muc 8.6/8.7): co goc thi noi THEM mot ban ghi sau cac tag;
khong co goc thi chuoi y het ban 0.6 - ban do cu giu nguyen CRC, khong ben nao phai doi gi.
"""

import struct
import zlib

BAN_GHI = '<Hiii'          # uint16 tag_id, int32 n_mm, int32 e_mm, int32 d_mm = 14 byte
CO_BAN_GHI = struct.calcsize(BAN_GHI)
# int32 lat_e7, int32 lon_e7, int32 alt_mm, int16 north_yaw_cdeg = 14 byte, sau moi ban ghi tag.
BAN_GHI_GOC = '<iiih'


def goc_nguyen(goc):
    """(lat_deg, lon_deg, alt_m, north_yaw_deg) -> so nguyen tren day (lat_e7, lon_e7, alt_mm,
    north_yaw_cdeg). round() nhu toa do tag - hai ben phai lam tron giong het nhau."""
    lat, lon, alt, yaw = goc
    return round(lat * 1e7), round(lon * 1e7), round(alt * 1000), round(yaw * 100)


def doc_goc(valid, lat, lon, alt, yaw):
    """Tham so geo_origin_* cua tags.yaml -> tuple goc hoac None khi chua khai."""
    return (float(lat), float(lon), float(alt), float(yaw)) if valid else None


def doc_known_tags(flat):
    """[id, x, y, z, ...] (ENU met, dung dinh dang tags.yaml) -> {id: (x, y, z)}."""
    if len(flat) % 4:
        raise ValueError(f'known_tags phai co boi so cua 4 phan tu, dang co {len(flat)}')
    return {int(flat[i]): (float(flat[i + 1]), float(flat[i + 2]), float(flat[i + 3]))
            for i in range(0, len(flat), 4)}


def tagmap_crc(tags, goc=None):
    """tags: {id: (x, y, z)} theo ENU met -> CRC-32 IEEE (giao uoc muc 8.6).

    goc: (lat_deg, lon_deg, alt_m, north_yaw_deg) hoac None. Co goc thi noi them ban ghi
    BAN_GHI_GOC sau cac tag (0.7); None thi CRC y het 0.6.

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
    if goc is not None:
        buf += struct.pack(BAN_GHI_GOC, *goc_nguyen(goc))
    return zlib.crc32(buf)


def doc_apriltag_declared(duong):
    """apriltag.yaml -> {tag_id: frame_name} (giao ước 11.6, Pi trả lời câu 2).

    Đây là NGUỒN DUY NHẤT cho tên khung TF: apriltag_ros bỏ qua tag không nằm trong tag.frames, và
    tag.ids/tag.frames/tag.sizes chỉ đổi được khi khởi động lại node — nạp bản đồ qua dây do đó CHỈ
    đổi được toạ độ của tag ĐÃ KHAI, không thêm được tag mới.
    """
    import yaml
    with open(duong) as f:
        data = yaml.safe_load(f) or {}
    tham_so = (data.get('/**') or {}).get('ros__parameters') or {}
    tag = tham_so.get('tag') or {}
    return {int(i): ten for i, ten in zip(tag.get('ids', []), tag.get('frames', []))}


def ghi_tags_override(duong, tags, khung_theo_id, goc=None):
    """{tag_id: (x, y, z) ENU mét} + {tag_id: frame_name} -> ghi file tham số ROS kiểu tags.yaml.

    Ghi ra NGOÀI cây build (giao ước 11.6, Pi trả lời câu 1): launch đọc bản chép trong install/,
    colcon build ghi đè nó, nên bản nhận qua dây phải nằm ở một chỗ riêng launch ưu tiên đọc trước.
    Sắp theo tag_id tăng dần cho khớp thứ tự CRC (mục 8.6).

    goc: (lat_deg, lon_deg, alt_m, north_yaw_deg) hoặc None — None thì ghi geo_origin_valid: false
    để bản đồ nạp qua dây KHÔNG kế thừa gốc của tags.yaml gốc (gốc thuộc về đúng bản đồ đi kèm).
    """
    import os
    ids = sorted(tags)
    os.makedirs(os.path.dirname(duong), exist_ok=True)
    with open(duong, 'w') as f:
        f.write('# Sinh tu dong boi gcs_link_node khi GCS nap ban do tag qua day (giao uoc 8.7, P31).\n')
        f.write('# KHONG sua tay - lan nap ke tiep se ghi de. Ban goc dong bo van o config/tags.yaml.\n')
        f.write('/**:\n  ros__parameters:\n')
        nums = ', '.join(f'{tid:.1f}, {tags[tid][0]:.3f}, {tags[tid][1]:.3f}, {tags[tid][2]:.3f}'
                         for tid in ids)
        f.write(f'    known_tags: [{nums}]\n')
        f.write('    tag_frames: [' + ', '.join(khung_theo_id[tid] for tid in ids) + ']\n')
        if goc is None:
            f.write('    geo_origin_valid: false\n')
        else:
            # 7 chu so thap phan = dung do phan giai e7 tren day, doc lai round() ra dung so cu.
            f.write('    geo_origin_valid: true\n')
            f.write(f'    geo_origin_lat: {goc[0]:.7f}\n')
            f.write(f'    geo_origin_lon: {goc[1]:.7f}\n')
            f.write(f'    geo_origin_alt: {goc[2]:.3f}\n')
            f.write(f'    geo_north_yaw_deg: {goc[3]:.2f}\n')


def to_ascii(chuoi, toi_da):
    """Bo dau va cat cho truong char[] cua dialect (giao uoc muc 8.4).

    pymavlink giai ma char[] bang ASCII, nen chu tieng Viet co dau ve toi ben kia thanh rac
    ("Lay" thanh "L???y"): khong gay loi, khong ai de y, va hong dung cho nguoi van hanh can doc.
    Ben GUI bo dau, khong de ben nhan doan lai.

    Cat theo BYTE sau khi da bo dau. Cat truoc khi bo dau se xe doi mot ky tu nhieu byte.
    """
    thay = {'à': 'a', 'á': 'a', 'ả': 'a', 'ã': 'a', 'ạ': 'a', 'ă': 'a', 'â': 'a',
            'è': 'e', 'é': 'e', 'ẻ': 'e', 'ẽ': 'e', 'ẹ': 'e', 'ê': 'e',
            'ì': 'i', 'í': 'i', 'ỉ': 'i', 'ĩ': 'i', 'ị': 'i',
            'ò': 'o', 'ó': 'o', 'ỏ': 'o', 'õ': 'o', 'ọ': 'o', 'ô': 'o', 'ơ': 'o',
            'ù': 'u', 'ú': 'u', 'ủ': 'u', 'ũ': 'u', 'ụ': 'u', 'ư': 'u',
            'ỳ': 'y', 'ý': 'y', 'ỷ': 'y', 'ỹ': 'y', 'ỵ': 'y', 'đ': 'd'}
    import unicodedata
    ra = []
    for ch in unicodedata.normalize('NFC', chuoi):
        thuong = ch.lower()
        co_ban = thay.get(thuong)
        if co_ban is None:
            # Bo dau chung: tach to hop roi loai dau ket hop.
            co_ban = ''.join(c for c in unicodedata.normalize('NFD', thuong)
                             if not unicodedata.combining(c)) or '?'
        ra.append(co_ban.upper() if ch.isupper() else co_ban)
    b = ''.join(ra).encode('ascii', 'replace')
    return b[:toi_da]
