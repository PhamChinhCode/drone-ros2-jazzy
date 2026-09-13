"""Kiem doc trang thai OB_* va luat chan lenh phia Pi theo giao uoc 6.2, 6.3."""

from drone_mission.fc_link import FcStatus, local_refusal


def status_voi(now_s=10.0, **values):
    st = FcStatus()
    for name, value in values.items():
        st.update(name, value, now_s)
    return st


def test_chua_nhan_goi_la_khong_biet():
    st = FcStatus()
    assert st.get('OB_AUTH', 0.0) is None
    assert st.has_authority(0.0) is None


def test_gia_tri_qua_han_la_khong_biet():
    st = status_voi(OB_AUTH=1)
    assert st.has_authority(11.0) is True
    assert st.has_authority(11.6) is None


def test_arm_chi_khi_ob_arm_rdy():
    st = status_voi(OB_AUTH=1, OB_ARM_RDY=0, OB_ARM_BLK=0x1000)
    assert 'OB_ARM_RDY' in local_refusal(True, False, st, 10.0)
    st.update('OB_ARM_RDY', 1, 10.0)
    assert local_refusal(True, False, st, 10.0) == ''


def test_mat_quyen_khong_gui_ke_ca_disarm_khi_dang_arm():
    st = status_voi(OB_AUTH=0, OB_ARM_RDY=0)
    assert local_refusal(False, True, st, 10.0) != ''
    assert local_refusal(True, False, st, 10.0) != ''


def test_khong_biet_quyen_thi_khong_gui():
    assert local_refusal(False, True, FcStatus(), 10.0) != ''


def test_disarm_khi_dang_khong_arm_luon_duoc_gui():
    """FC tra ACCEPTED va khong lam gi (6.2) - ke ca khi mat quyen hay chua co OB_*."""
    assert local_refusal(False, False, FcStatus(), 10.0) == ''
    assert local_refusal(False, False, status_voi(OB_AUTH=0), 10.0) == ''


def test_disarm_khi_co_quyen_khong_can_ob_dis_rdy():
    """Cong 20 cm la cua FC; bridge gui va tra ket qua that (TEMPORARILY_REJECTED)."""
    st = status_voi(OB_AUTH=1, OB_DIS_RDY=0)
    assert local_refusal(False, True, st, 10.0) == ''


def test_armed_khong_biet_coi_nhu_dang_arm():
    st = status_voi(OB_AUTH=0)
    assert local_refusal(False, None, st, 10.0) != ''
