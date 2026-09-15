"""Kiem quy tac failsafe: nguong, gia tri khong biet, muc leo thang."""

import pytest

from drone_safety import failsafe_rules as r

L = r.Limits()


def inp(**kw):
    base = dict(now_s=100.0, node_start_s=0.0, fc_heartbeat_s=99.5)
    base.update(kw)
    return r.Inputs(**base)


def test_binh_thuong_khong_su_co():
    assert r.evaluate(inp(battery_pct=80.0, ekf_healthy=True), L) == {}


def test_khong_biet_thi_khong_suy_ra_su_co():
    assert r.evaluate(inp(battery_pct=None, ekf_healthy=None, mission_state=None), L) == {}


@pytest.mark.parametrize('pct, esc', [(24.0, r.ESCALATE_RTH), (14.0, r.ESCALATE_EMERGENCY_LAND),
                                      (25.0, None)])
def test_pin(pct, esc):
    out = r.evaluate(inp(battery_pct=pct), L)
    assert out.get(r.FS_LOW_BATTERY, (None,))[0] == esc


def test_ekf_khong_healthy_loiter():
    assert r.evaluate(inp(ekf_healthy=False), L)[r.FS_EKF_UNHEALTHY][0] == r.ESCALATE_LOITER


def test_mat_fc_ke_ca_chua_tung_thay():
    assert r.FS_FC_COMM_LOST in r.evaluate(inp(fc_heartbeat_s=96.0), L)
    assert r.FS_FC_COMM_LOST not in r.evaluate(inp(fc_heartbeat_s=97.5), L)
    assert r.FS_FC_COMM_LOST in r.evaluate(inp(fc_heartbeat_s=None, node_start_s=89.0), L)
    assert r.FS_FC_COMM_LOST not in r.evaluate(inp(fc_heartbeat_s=None, node_start_s=95.0), L)


def test_mat_gcs_chi_khi_da_bao_false_du_lau():
    assert r.FS_LINK_LOST not in r.evaluate(inp(gcs_disconnected_since_s=None), L)
    assert r.FS_LINK_LOST not in r.evaluate(inp(gcs_disconnected_since_s=95.0), L)
    assert r.evaluate(inp(gcs_disconnected_since_s=85.0), L)[r.FS_LINK_LOST][0] == r.ESCALATE_RTH


def test_marker_timeout_va_gripper_chi_trong_dung_trang_thai():
    out = r.evaluate(inp(mission_state='MARKER_SEARCH', state_entered_s=70.0), L)
    assert out[r.FS_MARKER_TIMEOUT][0] == r.ESCALATE_RETRY_LOITER
    assert r.evaluate(inp(mission_state='PRECISION_LAND', state_entered_s=0.0), L) == {}
    out = r.evaluate(inp(mission_state='ACTUATE_GRIPPER', state_entered_s=90.0,
                         gripper_confirmed=False), L)
    assert r.FS_GRIP_CONFIRM_FAIL in out
    assert r.evaluate(inp(mission_state='ACTUATE_GRIPPER', state_entered_s=90.0,
                          gripper_confirmed=True), L) == {}


def test_hang_so_khop_failsafe_event():
    from drone_interfaces.msg import FailsafeEvent as F
    assert (r.FS_MARKER_TIMEOUT, r.FS_GRIP_CONFIRM_FAIL, r.FS_LINK_LOST, r.FS_LOW_BATTERY,
            r.FS_EKF_UNHEALTHY, r.FS_FC_COMM_LOST) == (
        F.FS_MARKER_TIMEOUT, F.FS_GRIP_CONFIRM_FAIL, F.FS_LINK_LOST, F.FS_LOW_BATTERY,
        F.FS_EKF_UNHEALTHY, F.FS_FC_COMM_LOST)
    assert (r.ESCALATE_LOITER, r.ESCALATE_RETRY_LOITER, r.ESCALATE_RTH,
            r.ESCALATE_EMERGENCY_LAND) == (F.ESCALATE_LOITER, F.ESCALATE_RETRY_LOITER,
                                           F.ESCALATE_RTH, F.ESCALATE_EMERGENCY_LAND)
