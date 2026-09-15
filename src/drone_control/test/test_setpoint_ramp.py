"""Kiem chuyen nguon setpoint muot: khong nhay tuc thoi, het ramp khop nguon moi."""

import pytest

from drone_control.setpoint_ramp import SetpointRamp


def test_cung_nguon_khong_tron():
    r = SetpointRamp(0.8)
    assert r.apply(0.0, 'cruise', (0.0, 0.0)) == (0.0, 0.0)
    assert r.apply(0.05, 'cruise', (1.5, -1.0)) == (1.5, -1.0)   # doi gia tri, khong doi nguon


def test_doi_nguon_tron_tuyen_tinh_roi_khop():
    r = SetpointRamp(0.8)
    r.apply(0.0, 'velocity', (0.5, 0.0))
    assert r.apply(1.0, 'landing', (-0.3, 0.2)) == pytest.approx((0.5, 0.0))
    assert r.apply(1.4, 'landing', (-0.3, 0.2)) == pytest.approx((0.1, 0.1))
    assert r.apply(1.8, 'landing', (-0.3, 0.2)) == pytest.approx((-0.3, 0.2))
    assert r.apply(1.85, 'landing', (-0.4, 0.2)) == pytest.approx((-0.4, 0.2))


def test_nguon_moi_doi_trong_luc_tron_van_bam_theo():
    r = SetpointRamp(1.0)
    r.apply(0.0, 'a', (1.0,))
    r.apply(0.1, 'b', (0.0,))
    assert r.apply(0.6, 'b', (-2.0,)) == pytest.approx((1.0 + (-2.0 - 1.0) * 0.5,))


def test_doi_nguon_giua_ramp_tron_tu_gia_tri_dang_ra():
    r = SetpointRamp(1.0)
    r.apply(0.0, 'a', (1.0,))
    r.apply(0.1, 'b', (0.0,))
    mid = r.apply(0.6, 'b', (0.0,))[0]                         # 0,5
    assert r.apply(0.7, 'a', (1.0,)) == pytest.approx((mid,))  # khong nhay ve 1,0


def test_lan_dau_va_duration_0_khong_tron():
    assert SetpointRamp(0.8).apply(0.0, 'a', (0.7,)) == (0.7,)
    r = SetpointRamp(0.0)
    r.apply(0.0, 'a', (1.0,))
    assert r.apply(0.1, 'b', (0.0,)) == (0.0,)
