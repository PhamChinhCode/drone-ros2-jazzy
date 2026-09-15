"""Kiem PID: reset kem sai so hien tai thi lan update dau khong co cu giat D."""

import pytest

from drone_control.pid import PID, check_gain_param


def test_reset_kem_sai_so_khong_giat_d():
    pid = PID(kp=0.0, ki=0.0, kd=1.0, i_limit=1.0, out_limit=10.0)
    pid.update(0.0, 0.05)
    pid.reset(0.5)
    assert pid.update(0.5, 0.05) == pytest.approx(0.0)
    assert pid.integral == pytest.approx(0.5 * 0.05)


def pids():
    return {phase: {axis: PID(0.0, 0.0, 0.0, 1.0, 2.0) for axis in 'xyz'}
            for phase in ('cruise', 'landing')}


def test_check_gain_param():
    p = pids()
    pid, field, err = check_gain_param(p, 'cruise.x.kp', 0.8)
    assert pid is p['cruise']['x'] and field == 'kp' and err == ''
    assert check_gain_param(p, 'control_rate_hz', 20.0) == (None, None, '')
    assert check_gain_param(p, 'cruise.w.kp', 1.0) == (None, None, '')
    assert 'phai >= 0' in check_gain_param(p, 'landing.y.ki', -0.1)[2]
    assert 'huu han' in check_gain_param(p, 'cruise.z.kd', float('nan'))[2]
    assert 'huu han' in check_gain_param(p, 'cruise.z.kd', True)[2]
