"""Kiem tieu chi cham dat 11.1 #12e bang chuoi mau tu sinh."""

import pytest

from drone_mission.landing_detector import GROUND_REF_DEFAULT_M, LandingDetector

DT = 0.05


def chay(det, n, range_m, vz, cmd_vz, t0=0.0):
    ket_qua = False
    for i in range(n):
        ket_qua = det.step(range_m, vz, cmd_vz, t0 + i * DT)
    return ket_qua, t0 + n * DT


def test_cham_dat_sau_du_1_giay():
    det = LandingDetector()
    ok, t = chay(det, 20, 0.18, 0.0, -0.3)          # 0,95 s
    assert not ok
    ok, _ = chay(det, 2, 0.18, 0.0, -0.3, t)
    assert ok


def test_mot_mau_sai_la_dem_lai():
    det = LandingDetector()
    _, t = chay(det, 15, 0.18, 0.0, -0.3)
    det.step(0.18, 0.2, -0.3, t)                     # nay len
    ok, _ = chay(det, 15, 0.18, 0.0, -0.3, t + DT)
    assert not ok


def test_bay_ngang_qua_hop_khong_phai_cham_dat():
    """Laser doc mat hop gan moc, vz = 0, nhung Pi khong ra lenh xuong."""
    det = LandingDetector()
    ok, _ = chay(det, 40, 0.18, 0.0, 0.0)
    assert not ok


def test_con_cao_khong_phai_cham_dat():
    det = LandingDetector()
    ok, _ = chay(det, 40, 0.40, 0.0, -0.3)
    assert not ok


def test_thieu_du_lieu_khong_phai_cham_dat():
    det = LandingDetector()
    assert not chay(det, 40, None, 0.0, -0.3)[0]
    assert not chay(det, 40, 0.18, None, -0.3)[0]
    assert not chay(det, 40, 0.18, 0.0, None)[0]


def test_moc_mac_dinh_khi_chua_hoc():
    assert LandingDetector().ground_ref_m == GROUND_REF_DEFAULT_M


def test_hoc_moc_luc_disarm_nam_yen():
    det = LandingDetector()
    for i in range(50):                               # 2,5 s
        det.update_ground_ref(False, 0.20, 0.0, i * DT)
    assert det.ground_ref_m == pytest.approx(0.20)
    det.update_ground_ref(True, 0.90, 0.5, 3.0)       # da arm: giu moc
    assert det.ground_ref_m == pytest.approx(0.20)


def test_moc_ngoai_khoang_hop_ly_dung_mac_dinh():
    """Arm luc cam tay / tren ban cao: laser 0,6 m -> khong tin, dung 0,17."""
    det = LandingDetector()
    for i in range(50):
        det.update_ground_ref(False, 0.60, 0.0, i * DT)
    assert det.ground_ref_m == GROUND_REF_DEFAULT_M


def test_dang_di_chuyen_khong_hoc_moc():
    det = LandingDetector()
    for i in range(50):
        det.update_ground_ref(False, 0.20, 0.3, i * DT)
    assert det.ground_ref_m == GROUND_REF_DEFAULT_M
