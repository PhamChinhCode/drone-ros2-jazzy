import math

import pytest

from drone_mission.camera_tilt import approach_moves, camera_link_pitch, TiltTable

# Bang do 2026-10-22 (mission.yaml).
BANG = TiltTable([-20.0, -15.9, -1.1, 0.0, 19.0, 20.0, 48.8],
                 [1244, 1300, 1500, 1514, 1750, 1762, 2050])


def test_doi_goc_ra_xung_theo_bang_do():
    assert BANG.to_us(0.0) == 1514
    assert BANG.to_us(20.0) == 1762
    assert BANG.to_us(-20.0) == 1244
    assert BANG.to_us(10.0) == pytest.approx(1514 + 10 / 19.0 * (1750 - 1514), abs=1)


def test_ngoai_bang_thi_kep_khong_ngoai_suy():
    assert BANG.to_us(-60.0) == 1244
    assert BANG.to_us(90.0) == 2050


def test_xung_ra_goc_nguoc_lai():
    assert BANG.to_deg(1762) == pytest.approx(20.0)
    assert BANG.to_deg(BANG.to_us(-7.3)) == pytest.approx(-7.3, abs=0.1)


def test_bang_sai_chieu_bi_tu_choi():
    with pytest.raises(ValueError):
        TiltTable([0.0, 20.0], [1762, 1514])
    with pytest.raises(ValueError):
        TiltTable([0.0], [1514])


def test_tien_toi_tu_phia_xung_thap():
    assert approach_moves(1514, 1762, 100) == [1762]          # dang len: di thang
    assert approach_moves(1762, 1514, 100) == [1414, 1514]    # dang xuong: vuot qua roi len
    assert approach_moves(None, 1514, 100) == [1414, 1514]    # chua biet: vuot cho chac
    assert approach_moves(1514, 1514, 100) == [1514]


def test_pitch_camera_link_khop_gia_tri_cu():
    assert camera_link_pitch(20.0) == pytest.approx(1.2217, abs=1e-4)
    assert camera_link_pitch(0.0) == pytest.approx(math.pi / 2)
    assert camera_link_pitch(-20.0) == pytest.approx(math.radians(110))
