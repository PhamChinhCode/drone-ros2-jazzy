"""Kiem vong P yaw: duong ngan qua +-180 do, kep toc do, dau theo FLU."""

import math

import pytest

from drone_control.yaw_control import yaw_rate_command

MAX = math.radians(30)


def test_quay_trai_khi_dich_lon_hon():
    assert yaw_rate_command(0.2, 0.0, 1.0, MAX) == pytest.approx(0.2)


def test_di_duong_ngan_qua_180_do():
    # 170 do -> -170 do: ngan nhat la +20 do (quay trai), khong phai -340 do.
    r = yaw_rate_command(math.radians(-170), math.radians(170), 1.0, MAX)
    assert r == pytest.approx(math.radians(20))


@pytest.mark.parametrize('err_deg', [90, -120, 179])
def test_kep_toc_do(err_deg):
    r = yaw_rate_command(math.radians(err_deg), 0.0, 1.0, MAX)
    assert abs(r) == pytest.approx(MAX)
    assert math.copysign(1, r) == math.copysign(1, err_deg)
