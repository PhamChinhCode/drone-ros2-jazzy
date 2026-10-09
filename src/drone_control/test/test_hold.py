"""Kiem giu cho mac dinh: ham roi moi chot, khong chot khi so khong tin, tha moc khi nhay bac."""

import math

from drone_control.hold import HeadingHold, PositionHold, YAW_BRAKE_TIMEOUT_S


def test_vi_tri_ham_xong_moi_chot_dung_cho_dung():
    h = PositionHold()
    assert h.update((0.0, 0.0), 0.8, True) is None          # con dang chay: chua chot
    assert h.update((0.1, 0.0), 0.5, True) is None
    assert h.update((0.15, 0.0), 0.1, True) == (0.15, 0.0)  # cham -> chot tai cho
    assert h.update((0.3, 0.1), 0.3, True) == (0.15, 0.0)   # troi di: moc khong doi


def test_vi_tri_khong_duoc_phep_thi_tha_moc():
    h = PositionHold()
    h.update((1.0, 2.0), 0.0, True)
    assert h.update((1.0, 2.0), 0.0, False) is None         # vd chua neo / nam dat / co lenh khac
    assert h.update((3.0, 2.0), 0.0, True) == (3.0, 2.0)    # duoc phep lai: chot cho moi


def test_vi_tri_ekf_nhay_bac_thi_chot_lai():
    h = PositionHold()
    h.update((0.0, 0.0), 0.0, True)
    assert h.update((0.8, 0.0), 0.0, True) == (0.8, 0.0)    # EKF sua theo tag 0,8 m
    assert h.update((0.85, 0.0), 0.0, True) == (0.8, 0.0)   # troi nho: van giu moc


def test_vi_tri_mat_odom_thi_tha():
    h = PositionHold()
    h.update((0.0, 0.0), 0.0, True)
    assert h.update(None, 0.0, True) is None


def test_huong_ham_roi_moi_chot():
    h = HeadingHold()
    assert h.update(0.0, math.radians(40), True, 0.05) is None
    assert h.update(0.3, math.radians(5), True, 0.05) == 0.3
    assert h.update(0.35, 0.0, True, 0.05) == 0.3


def test_huong_qua_han_ham_thi_chot_luon():
    h = HeadingHold()
    t, got = 0.0, None
    while got is None:
        got = h.update(0.0, math.radians(30), True, 0.05)    # quay mai khong cham lai
        t += 0.05
        assert t < YAW_BRAKE_TIMEOUT_S + 0.2
    assert got == 0.0


def test_huong_nhay_bac_thi_chot_lai_qua_180():
    h = HeadingHold()
    h.update(math.radians(179), 0.0, True, 0.05)
    assert h.update(math.radians(-179), 0.0, True, 0.05) == math.radians(179)   # chi 2 do
    assert h.update(math.radians(150), 0.0, True, 0.05) == math.radians(150)    # nhay 31 do


def test_huong_nam_dat_thi_khong_giu():
    h = HeadingHold()
    h.update(1.0, 0.0, True, 0.05)
    assert h.update(1.0, 0.0, False, 0.05) is None
