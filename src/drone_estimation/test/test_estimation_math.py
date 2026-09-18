"""Kiem phan tinh toan lop uoc luong: loc van toc FC, hinh hoc marker, co suc khoe EKF."""

import math

import pytest

from drone_estimation.estimation_math import (HealthMonitor, MarkerResetPolicy,
                                              drone_position_from_tag, fc_velocity_validity,
                                              parse_known_tags, rotate, tilt_cos_from_quaternion,
                                              vertical_range, zupt_active)


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


def roll_pitch_yaw_q(roll_deg, pitch_deg, yaw_deg):
    r, p, y = (math.radians(a) / 2.0 for a in (roll_deg, pitch_deg, yaw_deg))
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), \
        math.sin(y)
    return (sr * cp * cy - cr * sp * sy, cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy, cr * cp * cy + sr * sp * sy)


def test_tilt_cos_la_cos_roll_nhan_cos_pitch_khong_phu_thuoc_yaw():
    for yaw in (0, 90, -135):
        c = tilt_cos_from_quaternion(roll_pitch_yaw_q(10, 20, yaw))
        assert c == pytest.approx(math.cos(math.radians(10)) * math.cos(math.radians(20)))


def vr(r, tilt_deg, blocked=False):
    return vertical_range(r, 0.15, 8.0, math.cos(math.radians(tilt_deg)), blocked, 25.0, 3.0)


def test_do_cao_thang_dung_bu_nghieng():
    h, blocked = vr(2.0, 20.0)
    assert h == pytest.approx(2.0 * math.cos(math.radians(20.0)))
    assert not blocked


def test_nghieng_qua_nguong_thi_bo_va_co_tre():
    assert vr(2.0, 26.0) == (None, True)
    # Da bo: 24 do van chua du thap de nhan lai (nguong ve la 22 do).
    assert vr(2.0, 24.0, blocked=True) == (None, True)
    h, blocked = vr(2.0, 21.0, blocked=True)
    assert h is not None and not blocked
    # Chua bo: 24 do van nhan.
    assert vr(2.0, 24.0)[0] is not None


def test_so_do_laser_ngoai_dai_la_khong_hop_le():
    # FC hop dong 1.6 gui 0 khi mat laser; sim gui inf; NaN cung phai bi loai.
    for r in (0.0, float('inf'), float('nan'), 9.0):
        assert vr(r, 0.0)[0] is None


def test_zupt_chi_khi_chua_arm_va_trang_thai_con_moi():
    assert zupt_active(0.5, True, False, 2.5)
    assert not zupt_active(0.5, True, True, 2.5)        # da arm: de flow/tag lam viec
    assert not zupt_active(3.0, True, False, 2.5)       # trang thai qua han
    assert not zupt_active(0.5, False, False, 2.5)      # mat ket noi FC
    assert not zupt_active(None, True, False, 2.5)      # chua nhan /mavros/state lan nao


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


def reset_policy():
    return MarkerResetPolicy(max_error_m=1.0, disagree_after_s=1.0, marker_max_age_s=0.5,
                             cooldown_s=2.0)


def test_reset_lech_marker_phai_lien_tuc():
    # Tai hien 09-15: EKF bi day 20 m, marker dung o goc - van healthy vai giay dau.
    p = reset_policy()
    far, home = (20.0, 0.0, 0.3), (0.0, 0.0, 0.3)
    assert p.evaluate(0.0, True, True, far, home, 0.0) == ''
    assert p.evaluate(0.9, True, True, far, home, 0.9) == ''
    assert p.evaluate(1.0, True, True, home, home, 1.0) == ''        # khop lai: dem lai
    assert p.evaluate(1.5, True, True, far, home, 1.5) == ''
    assert 'lech marker' in p.evaluate(2.5, True, True, far, home, 2.5)


def test_reset_khi_khong_healthy_va_cooldown():
    p = reset_policy()
    home = (0.0, 0.0, 0.3)
    assert p.evaluate(0.0, True, False, home, home, 0.0) == 'EKF khong healthy'
    assert p.evaluate(1.0, True, False, home, home, 1.0) == ''       # trong cooldown
    assert p.evaluate(2.0, True, False, home, home, 2.0) == 'EKF khong healthy'


def test_reset_can_marker_moi_va_ekf_con_chay():
    p = reset_policy()
    far, home = (20.0, 0.0, 0.3), (0.0, 0.0, 0.3)
    assert p.evaluate(5.0, True, False, far, home, 4.0) == ''        # marker cu
    assert p.evaluate(5.0, True, False, far, home, None) == ''       # chua thay marker
    assert p.evaluate(5.0, False, False, far, home, 5.0) == ''       # EKF im lang: ep vo ich


def test_reset_ekf_nan():
    p = reset_policy()
    nan = (float('nan'), 0.0, 0.0)
    p.evaluate(0.0, True, True, nan, (0.0, 0.0, 0.3), 0.0)
    assert p.evaluate(1.0, True, True, nan, (0.0, 0.0, 0.3), 1.0) != ''
