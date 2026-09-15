"""Kiem FC gia lap theo giao uoc: setpoint, het han, ARM/DISARM, NAMED_VALUE_INT."""

import pytest

from drone_sim import sim_fc as f


def fc_nam_dat(**kw):
    fc = f.SimFc(**kw)
    fc.on_odometry(0.05)
    return fc


def sp(fc, t, vx=0.0, vz=0.0, frame=f.FRAME_BODY_NED, mask=f.TYPE_MASK_VELOCITY_YAWRATE):
    return fc.on_setpoint(t, frame, mask, vx, 0.0, vz, 0.0)


def test_laser_gia_theo_do_cao():
    fc = f.SimFc()
    assert fc.range_m() is None
    fc.on_odometry(0.05)
    assert fc.range_m() == pytest.approx(f.GROUND_RANGE_M)
    fc.on_odometry(2.05)
    assert fc.range_m() == pytest.approx(f.GROUND_RANGE_M + 2.0)
    fc.on_odometry(20.0)
    assert fc.range_m() is None


def test_chua_arm_khong_bat_dong_co():
    fc = fc_nam_dat()
    sp(fc, 0.0, vx=1.0)
    assert fc.step(0.1) == (False, (0.0, 0.0, 0.0, 0.0))


def test_loai_khung_sai_frame_hoac_mask():
    fc = fc_nam_dat(auto_arm=True)
    assert sp(fc, 0.0, vx=1.0, frame=1) is False
    assert sp(fc, 0.0, vx=1.0, mask=0x0FF8) is False
    assert fc.step(0.1) == (True, (0.0, 0.0, 0.0, 0.0))


def test_arm_chay_offboard_giu_nguyen_flu():
    fc = fc_nam_dat()
    assert fc.named_values()['OB_ARM_RDY'] == 1
    assert fc.command(400, 1.0, 0.0) == f.MAV_RESULT_ACCEPTED
    assert sp(fc, 1.0, vx=0.8, vz=-0.3)
    assert fc.step(1.1) == (True, (0.8, 0.0, -0.3, 0.0))
    assert fc.named_values()['OB_STATE'] == f.OB_STATE_DANG_CHAY


def test_het_han_phanh_va_mat_quyen_den_khi_disarm():
    fc = fc_nam_dat(auto_arm=True)
    sp(fc, 1.0, vx=1.0)
    fc.step(1.1)
    assert fc.step(1.6) == (True, (0.0, 0.0, 0.0, 0.0))
    nv = fc.named_values()
    assert nv['OB_STATE'] == f.OB_STATE_KHOA and nv['OB_AUTH'] == 0
    sp(fc, 1.7, vx=1.0)                                  # Pi song lai: KHONG tu vao lai
    assert fc.step(1.8) == (True, (0.0, 0.0, 0.0, 0.0))
    assert fc.command(400, 0.0, 0.0) == f.MAV_RESULT_DENIED
    assert fc.command(400, 0.0, f.DISARM_FORCE_MAGIC) == f.MAV_RESULT_ACCEPTED
    assert fc.named_values()['OB_AUTH'] == 1


def test_disarm_thuong_chi_khi_sat_dat():
    fc = fc_nam_dat(auto_arm=True)
    fc.on_odometry(1.05)
    assert fc.named_values()['OB_DIS_RDY'] == 0
    assert fc.command(400, 0.0, 0.0) == f.MAV_RESULT_TEMPORARILY_REJECTED
    fc.on_odometry(0.2)
    assert fc.named_values()['OB_DIS_RDY'] == 1
    assert fc.command(400, 0.0, 0.0) == f.MAV_RESULT_ACCEPTED and not fc.armed


def test_disarm_khi_khong_arm_va_lenh_khac():
    fc = fc_nam_dat()
    assert fc.command(400, 0.0, 0.0) == f.MAV_RESULT_ACCEPTED
    assert fc.command(22, 0.0, 0.0) == f.MAV_RESULT_UNSUPPORTED
    assert f.SimFc().command(400, 1.0, 0.0) == f.MAV_RESULT_TEMPORARILY_REJECTED   # chua co z


def test_hang_so_khop_fc_link():
    from drone_mission import fc_link
    assert f.MAV_CMD_COMPONENT_ARM_DISARM == fc_link.MAV_CMD_COMPONENT_ARM_DISARM
    assert f.DISARM_FORCE_MAGIC == fc_link.DISARM_FORCE_MAGIC
    assert (f.OB_STATE_KHOA, f.OB_STATE_TAT, f.OB_STATE_DANG_CHAY) == (
        fc_link.OB_STATE_KHOA, fc_link.OB_STATE_TAT, fc_link.OB_STATE_DANG_CHAY)
    assert f.FC_CTR_VER // 10000 == fc_link.KNOWN_CONTRACT_MAJOR
