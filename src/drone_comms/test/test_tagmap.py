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
