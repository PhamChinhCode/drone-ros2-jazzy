"""Kiem bu troi: vong P + troi co dinh de lai lech; co bu thi lech ve 0, ke ca khi doi nguon/quay mui."""

import math

import pytest

from drone_control.drift_comp import (body_to_world, closing_speed, DriftCompensator,
                                      world_to_body)


def mo_phong(comp, drift=(0.06, -0.03), kp=0.5, t_end=20.0, dt=0.05, yaw_rate=0.0):
    """Vong P vi tri he than + troi he ban do; tra lech cuoi (m)."""
    x, y, yaw, t = 0.3, -0.2, 0.0, 0.0
    while t < t_end:
        err_w = (-x, -y)                                   # dich tai goc
        cmd_b = tuple(kp * e for e in world_to_body(err_w, yaw))
        if comp is not None:
            b = world_to_body(comp.update(err_w, dt), yaw)
            cmd_b = (cmd_b[0] + b[0], cmd_b[1] + b[1])
        vx, vy = body_to_world(cmd_b, yaw)
        x, y = x + (vx + drift[0]) * dt, y + (vy + drift[1]) * dt
        yaw += yaw_rate * dt
        t += dt
    return math.hypot(x, y)


def test_chi_khau_p_thi_lech_co_dinh_bang_troi_chia_kp():
    assert mo_phong(None) == pytest.approx(math.hypot(0.06, -0.03) / 0.5, rel=0.02)


def test_co_bu_thi_lech_ve_0():
    assert mo_phong(DriftCompensator()) < 0.01


def test_quay_mui_van_giu_uoc_luong():
    assert mo_phong(DriftCompensator(), yaw_rate=math.radians(20)) < 0.01


def test_xa_dich_thi_khong_tich():
    c = DriftCompensator(window_m=0.5)
    assert c.update((3.0, 0.0), 1.0) == (0.0, 0.0)
    assert c.update((0.2, 0.0), 1.0)[0] > 0.0


def test_kep_theo_do_lon():
    c = DriftCompensator(ki=10.0, limit_mps=0.3)
    bx, by = c.update((0.4, 0.3), 1.0)
    assert math.hypot(bx, by) == pytest.approx(0.3)
    assert by / bx == pytest.approx(0.3 / 0.4)


def test_doi_he_qua_lai():
    v = (0.3, -0.1)
    assert body_to_world(world_to_body(v, 1.1), 1.1) == pytest.approx(v)
    assert world_to_body((1.0, 0.0), math.pi / 2) == pytest.approx((0.0, -1.0))


def keo_ve_dich(gate, drift=0.0, kp=0.5, t_end=40.0, dt=0.05):
    """1 truc: keo tu 0,2 m sau dich ve dich (nhu PRECISION_LAND sau diem dung PAD_ALIGN).
    gate: chi hoc khi toc do tien ve dich <= 0,03 m/s (position_controller_node). Tra (lech, bias)."""
    c = DriftCompensator(ki=0.05, limit_mps=0.1)
    x, t = -0.2, 0.0
    while t < t_end:
        v = kp * -x + c.bias[0]
        e = (-x, 0.0)
        if not gate or closing_speed(e, (v + drift, 0.0)) <= 0.03:
            c.update(e, dt)
        x += (kp * -x + c.bias[0] + drift) * dt
        t += dt
    return x, c.bias[0]


def test_dang_tien_ve_dich_thi_khong_hoc_nham_thanh_troi():
    # Khong co troi: hoc ca luc keo thi bias day vuot qua dich ~2 cm sau 8 s; co cong thi < 0,5 cm.
    vuot_khong, _ = keo_ve_dich(gate=False, t_end=8.0)
    vuot_co, _ = keo_ve_dich(gate=True, t_end=8.0)
    assert vuot_khong > 0.015 and abs(vuot_co) < 0.3 * vuot_khong
    # Troi that (dung yen lech / bi day ra xa) van hoc het: lech ve ~0.
    lech, bias = keo_ve_dich(gate=True, drift=-0.04, t_end=120.0)
    assert abs(lech) < 0.01 and bias == pytest.approx(0.04, abs=0.005)


def test_toc_do_tien_ve_dich():
    assert closing_speed((0.2, 0.0), (0.1, 0.0)) == pytest.approx(0.1)
    assert closing_speed((0.2, 0.0), (-0.1, 0.05)) == pytest.approx(-0.1)
    assert closing_speed((0.01, 0.0), (0.5, 0.0)) == 0.0
