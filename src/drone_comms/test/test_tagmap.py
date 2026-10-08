"""Kiem CRC ban do tag - giao uoc GCS <-> Pi muc 8.6, phep kiem A15(a)."""

import pytest

from drone_comms.tagmap import doc_known_tags, tagmap_crc

# Dung y tags.yaml hien tai: tag 0 tai goc, tag 1 tai x = 10 m.
TAGS_YAML = [0.0, 0.0, 0.0, 0.0,
             1.0, 10.0, 0.0, 0.0]


def test_vecto_kiem_hai_ben_da_thong_nhat():
    """A15(a): con so nay do phia GCS gui va Pi tinh lai doc lap - hai ben PHAI khop.

    Lech con so nay nghia la mot ben doi cach dong goi ma khong ghi vao giao uoc.
    """
    assert tagmap_crc(doc_known_tags(TAGS_YAML)) == 0x6BDEA0A6


def test_doi_mot_milimet_thi_crc_doi():
    """Muc dich cua CRC la bat lech vi tri, nen mot milimet cung phai lam no doi."""
    goc = tagmap_crc(doc_known_tags(TAGS_YAML))
    lech = list(TAGS_YAML)
    lech[5] = 10.001                                  # x cua tag 1: 10,000 -> 10,001 m
    assert tagmap_crc(doc_known_tags(lech)) != goc


def test_thu_tu_khai_khong_anh_huong():
    """Sap theo tag_id nen khai nguoc thu tu van ra cung CRC - hai ben khong phai giu thu tu."""
    nguoc = [1.0, 10.0, 0.0, 0.0,
             0.0, 0.0, 0.0, 0.0]
    assert tagmap_crc(doc_known_tags(nguoc)) == 0x6BDEA0A6


def test_doi_ENU_sang_NED_dung_chieu():
    """n = y, e = x, d = -z. Sai cho nay thi CRC hai ben lech ma khong ai hieu vi sao."""
    a = tagmap_crc({0: (1.0, 2.0, 3.0)})
    b = tagmap_crc({0: (2.0, 1.0, 3.0)})              # doi cho x va y
    assert a != b, 'x va y phai vao hai truong khac nhau'
    assert tagmap_crc({0: (0.0, 0.0, 1.0)}) != tagmap_crc({0: (0.0, 0.0, -1.0)})


def test_round_chu_khong_phai_int():
    """int() cat cut: 0,1235 -> 123 mm; round() -> 124 mm (nua ve so chan cho 0,1225)."""
    assert tagmap_crc({0: (0.0, 0.0001235, 0.0)}) == tagmap_crc({0: (0.0, 0.000124, 0.0)})


# --------------------------------------------------- goc WGS84 cua ban do (ban 0.7)

GOC = (21.0285110, 105.8048170, 15.0, -1.5)       # lat, lon, alt_m, north_yaw_deg


def test_khong_goc_thi_crc_nhu_0_6():
    """Ban do khong co goc giu NGUYEN CRC cu - GCS/Pi ban 0.6 khong bi anh huong."""
    assert tagmap_crc(doc_known_tags(TAGS_YAML), None) == 0x6BDEA0A6


def test_vecto_kiem_co_goc():
    """Vecto kiem 0.7 - GCS tinh lai doc lap phai ra dung so nay (test_tagmap GCS).

    Chuoi = 28 byte tag cua vecto 0.6 + 14 byte goc little-endian
    36b2880c aa88103f 983a0000 6aff  (lat_e7 210285110, lon_e7 1058048170, alt_mm 15000,
    north_yaw_cdeg -150).
    """
    assert tagmap_crc(doc_known_tags(TAGS_YAML), GOC) == 0xA35B41F9


def test_goc_lech_mot_don_vi_thi_crc_doi():
    lat, lon, alt, yaw = GOC
    goc = tagmap_crc(doc_known_tags(TAGS_YAML), GOC)
    for khac in [(lat + 1e-7, lon, alt, yaw), (lat, lon, alt + 0.001, yaw),
                 (lat, lon, alt, yaw + 0.01)]:
        assert tagmap_crc(doc_known_tags(TAGS_YAML), khac) != goc


def test_ghi_override_doc_lai_ra_dung_crc(tmp_path):
    """File Pi ghi khi GCS nap ban do phai doc lai ra DUNG CRC GCS da khai - khong thi GCS khoa
    nap ke hoach mai mai (quy tac 2 muc 8.7)."""
    import yaml

    from drone_comms.tagmap import doc_goc, ghi_tags_override
    tags = doc_known_tags(TAGS_YAML)
    duong = tmp_path / 'tags_override.yaml'
    ghi_tags_override(str(duong), tags, {0: 'pad_home', 1: 'pad_a'}, GOC)
    p = yaml.safe_load(duong.read_text())['/**']['ros__parameters']
    goc = doc_goc(p['geo_origin_valid'], p['geo_origin_lat'], p['geo_origin_lon'],
                  p['geo_origin_alt'], p['geo_north_yaw_deg'])
    assert tagmap_crc(doc_known_tags(p['known_tags']), goc) == 0xA35B41F9

    ghi_tags_override(str(duong), tags, {0: 'pad_home', 1: 'pad_a'}, None)
    p = yaml.safe_load(duong.read_text())['/**']['ros__parameters']
    assert p['geo_origin_valid'] is False


def test_known_tags_sai_dinh_dang_thi_bao_loi():
    with pytest.raises(ValueError):
        doc_known_tags([0.0, 1.0, 2.0])               # khong phai boi so cua 4


# --------------------------------------------------- char[] ASCII (muc 8.4, ban 0.4)

def test_bo_dau_truoc_khi_gui():
    """pymavlink giai ma char[] bang ASCII: chu co dau ve toi ben kia thanh rac ma khong bao loi."""
    from drone_comms.tagmap import to_ascii
    assert to_ascii('Lấy hàng tại bãi A', 50) == b'Lay hang tai bai A'
    assert to_ascii('Đường về', 50) == b'Duong ve'


def test_cat_sau_khi_bo_dau_khong_xe_ky_tu():
    """Cat theo byte TRUOC khi bo dau se xe doi mot ky tu nhieu byte."""
    from drone_comms.tagmap import to_ascii
    ra = to_ascii('ắ' * 30, 10)
    assert ra == b'a' * 10 and len(ra) == 10


def test_chuoi_ascii_san_khong_doi():
    from drone_comms.tagmap import to_ascii
    assert to_ascii('tu choi ke hoach 903', 50) == b'tu choi ke hoach 903'


# --------------------------------------------------- nap ban do tag qua day (muc 8.7, P31)

def test_doc_apriltag_declared(tmp_path):
    from drone_comms.tagmap import doc_apriltag_declared
    p = tmp_path / 'apriltag.yaml'
    p.write_text("/**:\n  ros__parameters:\n    tag:\n      ids: [0, 1, 2]\n"
                "      frames: [pad_home, pad_a, pad_b]\n      sizes: [0.122, 0.122, 0.122]\n")
    assert doc_apriltag_declared(str(p)) == {0: 'pad_home', 1: 'pad_a', 2: 'pad_b'}


def test_ghi_tags_override_dung_dinh_dang_va_thu_tu(tmp_path):
    """Ghi ra phai doc lai duoc bang doc_known_tags, va sap theo tag_id tang dan (khop CRC 8.6)."""
    from drone_comms.tagmap import doc_known_tags, ghi_tags_override, tagmap_crc
    duong = tmp_path / 'ghi_de' / 'tags_override.yaml'
    tags = {1: (10.0, 0.0, 0.0), 0: (0.0, 0.0, 0.0)}       # co y dua nguoc thu tu
    khung = {0: 'pad_home', 1: 'pad_a'}
    ghi_tags_override(str(duong), tags, khung)
    noi_dung = duong.read_text()
    assert 'known_tags: [0.0, 0.000, 0.000, 0.000, 1.0, 10.000, 0.000, 0.000]' in noi_dung
    assert 'tag_frames: [pad_home, pad_a]' in noi_dung
    # doc lai qua yaml.safe_load + doc_known_tags phai ra dung CRC nhu neu khai truc tiep muc 8.6
    import yaml
    flat = yaml.safe_load(noi_dung)['/**']['ros__parameters']['known_tags']
    assert tagmap_crc(doc_known_tags(flat)) == tagmap_crc(tags)


def test_ghi_tags_override_tao_thu_muc_neu_chua_co(tmp_path):
    from drone_comms.tagmap import ghi_tags_override
    duong = tmp_path / 'chua_ton_tai' / 'sau' / 'tags_override.yaml'
    ghi_tags_override(str(duong), {0: (0.0, 0.0, 0.0)}, {0: 'pad_home'})
    assert duong.exists()


# --------------------------------------------------- huong tag (ban 0.8, muc 8.6/8.7)

HUONG = {0: 90.0, 1: -45.0}


def test_khong_huong_thi_crc_nhu_0_7():
    """Ban do khong tag nao co huong giu NGUYEN CRC cu - GCS/Pi 0.7 khong bi anh huong."""
    assert tagmap_crc(doc_known_tags(TAGS_YAML), None, {}) == 0x6BDEA0A6
    assert tagmap_crc(doc_known_tags(TAGS_YAML), GOC, None) == 0xA35B41F9


def test_vecto_kiem_co_huong():
    """Vecto kiem 0.8 trong giao uoc 8.6 - GCS tinh lai doc lap phai ra dung cac so nay.

    8 byte noi them: 0000 2823 0100 6cee (tag 0: 9000 cdeg, tag 1: -4500 cdeg).
    """
    tags = doc_known_tags(TAGS_YAML)
    assert tagmap_crc(tags, None, HUONG) == 0xD3A58731
    assert tagmap_crc(tags, None, {0: 90.0}) == 0xAADD4B3C
    assert tagmap_crc(tags, GOC, HUONG) == 0x6D6CED89      # goc truoc, huong sau


def test_huong_dang_chuan_tac():
    """180 do va -180 do la mot huong: phai cung ra -18000, khong thi CRC hai ben lech."""
    from drone_comms.tagmap import huong_cdeg
    assert huong_cdeg(180.0) == -18000
    assert huong_cdeg(-180.0) == -18000
    assert huong_cdeg(359.99) == -1
    assert huong_cdeg(-90.0) == -9000
    assert huong_cdeg(179.99) == 17999
    tags = doc_known_tags(TAGS_YAML)
    assert tagmap_crc(tags, None, {0: 180.0}) == tagmap_crc(tags, None, {0: -180.0})


def test_doi_huong_mot_cdeg_thi_crc_doi():
    tags = doc_known_tags(TAGS_YAML)
    assert tagmap_crc(tags, None, {0: 90.0}) != tagmap_crc(tags, None, {0: 90.01})


def test_doc_huong():
    from drone_comms.tagmap import doc_huong
    assert doc_huong([0.0, 90.0, 1.0, -45.0]) == HUONG
    assert doc_huong([]) == {}
    with pytest.raises(ValueError):
        doc_huong([0.0, 90.0, 1.0])


def test_ghi_override_co_huong_doc_lai_ra_dung_crc(tmp_path):
    import yaml
    from drone_comms.tagmap import doc_huong, ghi_tags_override
    duong = tmp_path / 'tags_override.yaml'
    tags = doc_known_tags(TAGS_YAML)
    ghi_tags_override(str(duong), tags, {0: 'home', 1: 'a'}, None, HUONG)
    p = yaml.safe_load(duong.read_text())['/**']['ros__parameters']
    assert tagmap_crc(doc_known_tags(p['known_tags']), None,
                      doc_huong(p['known_tags_heading'])) == 0xD3A58731
    ghi_tags_override(str(duong), tags, {0: 'home', 1: 'a'})
    p = yaml.safe_load(duong.read_text())['/**']['ros__parameters']
    assert p['known_tags_heading'] == []

