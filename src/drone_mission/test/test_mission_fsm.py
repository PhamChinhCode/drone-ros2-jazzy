"""Kiem lop FC cua MissionFsm (giao uoc 6.2, 6.3, P2/P10) bang Snapshot tu dung."""

from drone_mission import mission_fsm as m

CO_QUYEN = dict(fc_connected=True, ob_auth=True, ob_state=1)


def snap(t, **kw):
    return m.Snapshot(now_s=t, **{**CO_QUYEN, **kw})


def fsm_dang_ha_canh():
    fsm = m.MissionFsm(params=m.Params(takeoff_alt_m=1.0))
    fsm.request_start(0.0)
    fsm.step(snap(0.0, arm_ready=True))
    fsm.step(snap(0.5, armed=True))
    fsm.request_land()
    fsm.step(snap(1.0, armed=True, range_m=0.5))
    assert fsm.state == m.EMERGENCY_LAND
    return fsm


def test_khong_yeu_cau_thi_khong_arm():
    fsm = m.MissionFsm()
    act = fsm.step(snap(0.0, arm_ready=True))
    assert fsm.state == m.IDLE and act.fc_command == ''


def test_arm_chi_khi_ob_arm_rdy_va_thua_2_giay():
    fsm = m.MissionFsm()
    assert fsm.request_start(0.0) == ''
    assert fsm.step(snap(0.0, arm_ready=False)).fc_command == ''
    assert fsm.step(snap(0.5, arm_ready=True)).fc_command == 'arm'
    assert fsm.step(snap(1.0, arm_ready=True)).fc_command == ''
    assert fsm.step(snap(2.6, arm_ready=True)).fc_command == 'arm'


def test_khong_arm_khi_mat_hoac_khong_biet_quyen():
    for auth in (False, None):
        fsm = m.MissionFsm()
        fsm.request_start(0.0)
        act = fsm.step(snap(0.0, arm_ready=True, ob_auth=auth))
        assert act.fc_command == ''


def test_het_han_cho_arm_huy_yeu_cau():
    fsm = m.MissionFsm()
    fsm.request_start(0.0)
    fsm.step(snap(m.ARM_WAIT_S + 1, arm_ready=False))
    assert fsm.step(snap(m.ARM_WAIT_S + 2, arm_ready=True)).fc_command == ''


def test_da_arm_sang_takeoff_va_leo():
    fsm = m.MissionFsm(params=m.Params(takeoff_alt_m=1.0))
    fsm.request_start(0.0)
    fsm.step(snap(0.0, arm_ready=True))
    fsm.step(snap(0.5, armed=True))
    assert fsm.state == m.TAKEOFF
    assert fsm.step(snap(1.0, armed=True, range_m=0.2)).velocity_up_mps == m.TAKEOFF_CLIMB_MPS
    assert fsm.step(snap(1.1, armed=True, range_m=1.0)).velocity_up_mps == 0.0
    assert fsm.step(snap(1.2, armed=True, range_m=None)).velocity_up_mps == 0.0


def test_mat_quyen_khi_arm_sang_failsafe_khong_gui_gi():
    for kw in (dict(ob_auth=False), dict(ob_state=0)):
        fsm = fsm_dang_ha_canh()
        act = fsm.step(snap(2.0, armed=True, landed=True, disarm_ready=True, **kw))
        assert fsm.state == m.FAILSAFE
        assert act.fc_command == '' and act.velocity_up_mps is None
        act = fsm.step(snap(5.0, armed=True, landed=True, disarm_ready=True, **kw))
        assert act.fc_command == ''


def test_failsafe_disarm_ve_idle_lay_lai_quyen_thi_ha_canh():
    fsm = fsm_dang_ha_canh()
    fsm.step(snap(2.0, armed=True, ob_auth=False))
    fsm.step(snap(3.0, armed=True))
    assert fsm.state == m.EMERGENCY_LAND
    fsm.step(snap(4.0, armed=True, ob_auth=False))
    fsm.step(snap(5.0, armed=False, ob_auth=False))
    assert fsm.state == m.IDLE


def test_khong_biet_quyen_khi_arm_thi_giu_nguyen():
    fsm = fsm_dang_ha_canh()
    act = fsm.step(snap(2.0, armed=True, ob_auth=None, landed=True, disarm_ready=True))
    assert fsm.state == m.EMERGENCY_LAND and act.fc_command == ''


def test_ha_canh_disarm_khi_cham_dat_va_ob_dis_rdy_thu_lai_3_giay():
    fsm = fsm_dang_ha_canh()
    assert fsm.step(snap(2.0, armed=True)).velocity_up_mps == -m.LAND_DESCENT_MPS
    assert fsm.step(snap(2.1, armed=True, landed=True, disarm_ready=False)).fc_command == ''
    assert fsm.step(snap(2.2, armed=True, landed=True, disarm_ready=True)).fc_command == 'disarm'
    assert fsm.step(snap(3.0, armed=True, landed=True, disarm_ready=True)).fc_command == ''
    act = fsm.step(snap(5.3, armed=True, landed=True, disarm_ready=True))
    assert act.fc_command == 'disarm' and act.velocity_up_mps == -m.LAND_DESCENT_MPS
    fsm.step(snap(5.5, armed=False))
    assert fsm.state == m.MISSION_COMPLETE
    fsm.step(snap(5.6))
    assert fsm.state == m.IDLE


def test_log_cho_xac_nhan_khi_ob_toi_truoc_mavros_state():
    fsm = m.MissionFsm()
    fsm.request_start(0.0)
    fsm.step(snap(0.0, arm_ready=True))
    act = fsm.step(snap(0.2, arm_ready=False))           # OB_ARM_RDY ve 0, armed chua len
    assert act.fc_command == '' and 'cho FC xac nhan' in act.detail

    fsm = fsm_dang_ha_canh()
    assert fsm.step(snap(2.0, armed=True, landed=True, disarm_ready=True)).fc_command == 'disarm'
    act = fsm.step(snap(2.2, armed=True, landed=True, disarm_ready=False))
    assert act.fc_command == '' and 'cho FC xac nhan' in act.detail


def test_khong_bao_gio_tu_gui_lenh_khac_arm_disarm():
    fsm = fsm_dang_ha_canh()
    lenh = set()
    for i in range(200):
        lenh.add(fsm.step(snap(2.0 + i * 0.1, armed=i < 150, landed=True, disarm_ready=True))
                 .fc_command)
    assert lenh <= {'', 'disarm'}


def test_yeu_cau_ha_canh_khi_chua_arm_bi_bo():
    fsm = m.MissionFsm()
    fsm.request_land()
    fsm.step(snap(0.0))
    fsm.request_start(1.0)
    fsm.step(snap(1.0, arm_ready=True))
    fsm.step(snap(1.5, armed=True))
    fsm.step(snap(2.0, armed=True, range_m=0.2))
    assert fsm.state == m.TAKEOFF


def test_request_start_chi_khi_idle():
    fsm = fsm_dang_ha_canh()
    assert fsm.request_start(2.0) != ''
