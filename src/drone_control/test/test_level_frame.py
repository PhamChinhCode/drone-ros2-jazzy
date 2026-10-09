"""Kiem he than phang: tag ngay duoi than dang nghieng khong duoc bao lech ngang."""

import math

import pytest

from drone_control.level_frame import rotate, to_level, yaw_of


def q_rpy(roll=0.0, pitch=0.0, yaw=0.0):
    cr, sr = math.cos(roll / 2), math.sin(roll / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
    return (sr * cp * cy - cr * sp * sy, cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy, cr * cp * cy + sr * sp * sy)


def body_vector(q_wb, v_world):
    """Vecto the gioi -> bieu dien trong base_link (nhu TF base_link -> tag tra ve)."""
    x, y, z, w = q_wb
    return rotate((-x, -y, -z, w), v_world)


@pytest.mark.parametrize('roll,pitch,yaw', [(0, 10, 0), (0, -10, 30), (8, 0, -120), (5, -7, 170)])
def test_tag_ngay_duoi_khong_lech_ngang_du_than_nghieng(roll, pitch, yaw):
    q = q_rpy(math.radians(roll), math.radians(pitch), math.radians(yaw))
    v_body = body_vector(q, (0.0, 0.0, -1.0))           # tag ngay duoi 1 m
    assert math.hypot(v_body[0], v_body[1]) > 0.08     # base_link (nghieng) bao lech gia
    p, _ = to_level(q, v_body, (0.0, 0.0, 0.0, 1.0))
    assert p == pytest.approx((0.0, 0.0, -1.0), abs=1e-9)


def test_lech_that_giu_nguyen_theo_huong_mui():
    """Tag lech 0,3 m ve phia mui (the gioi), than nghieng + mui huong 40 do."""
    yaw = math.radians(40)
    q = q_rpy(math.radians(6), math.radians(-9), yaw)
    v_world = (0.3 * math.cos(yaw), 0.3 * math.sin(yaw), -1.2)
    p, _ = to_level(q, body_vector(q, v_world), (0.0, 0.0, 0.0, 1.0))
    assert p == pytest.approx((0.3, 0.0, -1.2), abs=1e-9)    # truoc mui 0,3 m, khong lech ngang


def test_khong_nghieng_thi_khong_doi():
    q = q_rpy(0, 0, math.radians(75))
    v = (0.2, -0.1, -0.9)
    p, _ = to_level(q, v, (0.0, 0.0, 0.0, 1.0))
    assert p == pytest.approx(v, abs=1e-12)


def test_huong_tag_cung_bo_nghieng():
    q = q_rpy(math.radians(10), 0, math.radians(20))
    _, qt = to_level(q, (0, 0, -1), q_rpy(0, 0, math.radians(5)))
    assert math.degrees(yaw_of(qt)) == pytest.approx(5.0, abs=1.0)

