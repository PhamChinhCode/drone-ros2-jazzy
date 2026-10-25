import math

import pytest

from drone_sim.gripper_sim import (OPEN_RAD, box_in_jaws, four_bar, joint_targets, step_servo,
                                   SIDES, H, _rot, _sub)


def test_goc_0_la_tu_the_cad():
    for v in joint_targets(0.0).values():
        assert v == pytest.approx(0.0, abs=1e-9)


def test_mo_khop_gia_tri_urdf():
    # Gioi han joint trong drone_v2.urdf = loi giai chinh xac o -180 do (build_meshes.py); hang so
    # o day lam tron 0,01 mm nen lech toi 0,02 do.
    t = joint_targets(OPEN_RAD)
    assert t['gripper_jaw_left_joint'] == pytest.approx(0.97716, abs=1e-3)
    assert t['gripper_jaw_right_joint'] == pytest.approx(-0.97425, abs=1e-3)
    assert t['gripper_rod_left_joint'] == pytest.approx(3.41901, abs=1e-3)
    assert t['gripper_rod_right_joint'] == pytest.approx(2.85854, abs=1e-3)


@pytest.mark.parametrize('deg', [-180, -150, -120, -90, -45, -10])
def test_vong_kin_khep(deg):
    """Dau thanh (quay theo horn + thanh) trung tai ham (quay quanh chot)."""
    th = math.radians(deg)
    for side, (h0, e0, p) in SIDES.items():
        jaw, rod = four_bar(side, th)
        tai = tuple(p[i] + _rot(_sub(e0, p), jaw)[i] for i in range(2))
        h = tuple(H[i] + _rot(_sub(h0, H), th)[i] for i in range(2))
        dau = tuple(h[i] + _rot(_sub(e0, h0), th + rod)[i] for i in range(2))
        assert math.dist(tai, dau) < 1e-6


def test_ham_mo_ra_hai_ben():
    t = joint_targets(math.radians(-60))
    assert t['gripper_jaw_left_joint'] > 0 > t['gripper_jaw_right_joint']


def test_servo_chay_theo_toc_do():
    assert step_servo(0.0, OPEN_RAD, 0.1) == pytest.approx(-1.05)
    assert step_servo(-3.0, OPEN_RAD, 0.1) == pytest.approx(OPEN_RAD)


# Hop 12 cm: day hop 8,6 cm duoi day ngon (gap o ToF 0,27 m).
DAY = -0.1928 - 0.086


def test_hop_giua_ham_kep_duoc():
    assert box_in_jaws((0.01, -0.015, DAY), math.radians(88), 0.06, 0.12)


@pytest.mark.parametrize('box,yaw', [
    ((0.0, 0.065, DAY), 0.0),             # lech ngang qua khe mo
    ((0.05, 0.0, DAY), 0.0),              # ngon trot ra ngoai mat hop
    ((0.0, 0.0, DAY), math.radians(30)),  # hop xoay lech ham
    ((0.0, 0.0, -0.1928 + 0.005), 0.0),   # drone qua thap, ngon cham dat
    ((0.0, 0.0, -0.1928 - 0.115), 0.0),   # drone qua cao, ngon tren nap
])
def test_hop_lech_khong_kep(box, yaw):
    assert not box_in_jaws(box, yaw, 0.06, 0.12)
