"""Kiem tu kep setpoint theo 9.6: khong bao gio cham tran FC."""

import math

import pytest

from drone_control.setpoint_limits import limit_velocity


def test_trong_gioi_han_giu_nguyen():
    assert limit_velocity(0.5, -0.5, 0.3, 0.2, 2.0) == pytest.approx((0.5, -0.5, 0.3, 0.2))


def test_kep_ngang_moi_truc_rieng_95_phan_tram():
    vx, vy, _, _ = limit_velocity(10.0, -10.0, 0.0, 0.0, 2.0)
    assert (vx, vy) == pytest.approx((1.9, -1.9))


def test_kep_len_va_yaw():
    _, _, vz, yr = limit_velocity(0.0, 0.0, 5.0, -10.0, 2.0)
    assert vz == pytest.approx(0.95)
    assert yr == pytest.approx(-math.radians(85.5))


def test_xuong_nhanh_khi_cao():
    assert limit_velocity(0, 0, -5.0, 0, 2.0)[2] == pytest.approx(-0.95)


def test_xuong_cham_duoi_1_2_m():
    assert limit_velocity(0, 0, -0.5, 0, 1.2)[2] == pytest.approx(-0.285)
    assert limit_velocity(0, 0, -0.2, 0, 0.5)[2] == pytest.approx(-0.2)


def test_mat_laser_coi_nhu_sat_dat():
    assert limit_velocity(0, 0, -0.9, 0, None)[2] == pytest.approx(-0.285)
