"""Kiem phan tinh toan lop uoc luong: loc van toc FC, hinh hoc marker, co suc khoe EKF."""

import math

import pytest

from drone_estimation.estimation_math import (HealthMonitor, drone_position_from_tag,
                                              fc_velocity_validity, parse_known_tags, rotate)


def cov(vx, vy, vz):
    c = [0.0] * 36
    c[0], c[7], c[14] = vx, vy, vz
    return c


def test_van_toc_fc_so_do_that_nam_ban():
    # Do 09-14 tren /mavros/odometry/in: flow chua hop le, vz hop le.
    assert fc_velocity_validity(cov(1e6, 1e6, 4.98e-4)) == (False, True)


def test_van_toc_fc_hop_le_va_khong_hop_le():
    assert fc_velocity_validity(cov(0.01, 0.01, 1e6)) == (True, False)
    assert fc_velocity_validity(cov(0.01, 1e6, 0.01)) == (False, True)
    assert fc_velocity_validity(cov(float('nan'), 0.01, float('inf'))) == (False, False)
    assert fc_velocity_validity(cov(-1.0, 0.01, 0.01)) == (False, True)


def yaw_q(deg):
    h = math.radians(deg) / 2.0
    return (0.0, 0.0, math.sin(h), math.cos(h))


def test_rotate_yaw_90():
    x, y, z = rotate(yaw_q(90), (1.0, 0.0, 0.0))
    assert (x, y, z) == pytest.approx((0.0, 1.0, 0.0), abs=1e-12)


def test_vi_tri_drone_tu_tag_ngay_duoi():
    # Drone treo 2 m ngay tren tag o goc odom, khong xoay: tag o z = -2 trong than may.
    assert drone_position_from_tag((0.0, 0.0, 0.0), (0.0, 0.0, -2.0), yaw_q(0)) == \
        pytest.approx((0.0, 0.0, 2.0))


def test_vi_tri_drone_co_yaw():
    # Mui drone huong +y odom (yaw 90), tag nam 1 m phia truoc mui, 2 m ben duoi.
    pos = drone_position_from_tag((10.0, 0.0, 0.0), (1.0, 0.0, -2.0), yaw_q(90))
    assert pos == pytest.approx((10.0, -1.0, 2.0), abs=1e-9)


def test_parse_known_tags():
    assert parse_known_tags([0.0, 0.0, 0.0, 0.0, 1.0, 10.0, 0.0, 0.0]) == \
        {0: (0.0, 0.0, 0.0), 1: (10.0, 0.0, 0.0)}
    with pytest.raises(ValueError):
        parse_known_tags([0.0, 1.0])


def test_health_im_lang_va_nan():
    h = HealthMonitor(2.0, 2.0, 1.0)
    assert h.evaluate(0.0, None, (0, 0, 0), True)[0] is False
    assert h.evaluate(5.0, 3.5, (0, 0, 0), True)[0] is False
    assert h.evaluate(5.0, 4.9, (0, 0, 0), False)[0] is False
    assert h.evaluate(5.0, 4.9, (0.1, 0.1, 0.1), True) == (True, '')


def test_health_phuong_sai_vuot_nguong_phai_lien_tuc():
    h = HealthMonitor(2.0, 2.0, 1.0)
    assert h.evaluate(0.0, 0.0, (3.0, 0, 0), True)[0] is True
    assert h.evaluate(1.9, 1.9, (3.0, 0, 0), True)[0] is True
    assert h.evaluate(2.0, 2.0, (1.0, 0, 0), True)[0] is True      # xuong duoi nguong: dem lai
    assert h.evaluate(3.0, 3.0, (3.0, 0, 0), True)[0] is True
    assert h.evaluate(5.0, 5.0, (3.0, 0, 0), True)[0] is False
