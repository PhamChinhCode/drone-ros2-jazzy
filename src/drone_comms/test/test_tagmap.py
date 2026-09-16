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
