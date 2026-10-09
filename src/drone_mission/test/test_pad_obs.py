"""Kiem loc tu the bai do tu tag: hoi tu, yaw qua +-180 do, loai mau xa vi tri khai bao."""

import math

import pytest

from drone_mission.pad_obs import PadObsFilter


def test_chua_co_mau_thi_none():
    assert PadObsFilter().value() is None


def test_hoi_tu_ve_tu_the_do():
    f = PadObsFilter()
    for _ in range(30):
        assert f.update(10.1, 0.2, 0.0, math.radians(90), (10.0, 0.0))
    x, y, z, yaw = f.value()
    assert (x, y, z) == pytest.approx((10.1, 0.2, 0.0), abs=1e-6)
    assert yaw == pytest.approx(math.radians(90), abs=1e-6)


def test_yaw_trung_binh_qua_180_do_khong_ve_0():
    f = PadObsFilter(alpha=0.5)
    for yaw in (math.radians(179), math.radians(-179)) * 10:
        f.update(0.0, 0.0, 0.0, yaw, (0.0, 0.0))
    assert abs(abs(f.value()[3]) - math.pi) < math.radians(2)


def test_mau_xa_vi_tri_khai_bao_bi_loai():
    f = PadObsFilter(max_offset_m=3.0)
    assert not f.update(20.0, 0.0, 0.0, 0.0, (10.0, 0.0))
    assert f.value() is None
