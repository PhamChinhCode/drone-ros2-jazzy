"""Kiem dieu kien khoi dong lai stack tu GCS (lenh 42101): chi khi chac chan dang nam yen."""

import pytest

from drone_comms.stack_restart import quyet_dinh


def test_nam_yen_idle_thi_cho_phep():
    assert quyet_dinh(False, 0, '1234') == ('ok', '')


@pytest.mark.parametrize('armed', [True, None])
def test_arm_hoac_khong_biet_thi_tu_choi(armed):
    kq, ly_do = quyet_dinh(armed, 0, '1234')
    assert kq == 'tu_choi' and ly_do


def test_dang_lam_nhiem_vu_thi_tu_choi():
    kq, ly_do = quyet_dinh(False, 2, '1234')
    assert kq == 'tu_choi' and 'IDLE' in ly_do


@pytest.mark.parametrize('pid', [None, '', 'abc'])
def test_chay_tay_khong_co_service_thi_khong_ho_tro(pid):
    assert quyet_dinh(False, 0, pid)[0] == 'khong_ho_tro'
