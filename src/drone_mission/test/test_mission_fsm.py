"""Kiem lop FC cua MissionFsm (giao uoc 6.2, 6.3, P2/P10) bang Snapshot tu dung."""

import pytest

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
    # Giu MISSION_COMPLETE du lau cho telemetry 2 Hz lay duoc mau (giao uoc 11.5 P30).
    fsm.step(snap(5.6))
    assert fsm.state == m.MISSION_COMPLETE
    fsm.step(snap(5.5 + m.MISSION_COMPLETE_HOLD_S))
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


# ---- Buoc 1: nap ke hoach ----------------------------------------------------------------

TAGS = {0: (0.0, 0.0, 0.0), 1: (10.0, 0.0, 0.0)}


def wp(i, marker=0, **kw):
    d = dict(seq=i, marker_id=marker, action=m.ACTION_NONE, alt_m=0.5, acceptance_radius_m=0.3,
             max_vel_mps=0.5, loiter_s=1.0)
    d.update(kw)
    return d


def test_nap_ke_hoach_hop_le_vi_tri_suy_tu_tag():
    fsm = m.MissionFsm()
    assert fsm.load_plan(7, [wp(0, 1, alt_m=2.0), wp(1, 0, action=m.ACTION_DROPOFF)], 0, 0.0,
                         TAGS) == ''
    assert fsm.mission_id == 7 and len(fsm.waypoints) == 2 and fsm.current_wp_index == 0
    assert fsm.waypoints[0].target == (10.0, 0.0, 2.0)
    assert fsm.max_retries == fsm.params.max_retries
    assert fsm.search_timeout_s == fsm.params.search_timeout_s
    assert fsm.state == m.IDLE and fsm.start_requested_s is None      # khong tu cat canh


def test_ke_hoach_ghi_de_retry_va_timeout():
    fsm = m.MissionFsm()
    fsm.load_plan(1, [wp(0)], 5, 12.0, TAGS)
    assert fsm.max_retries == 5 and fsm.search_timeout_s == 12.0


@pytest.mark.parametrize('raw, loi', [
    ([], 'khong co waypoint'),
    ([wp(1)], 'seq'),
    ([wp(0, marker=9)], 'known_tags'),
    ([wp(0, action=5)], 'action'),
    ([wp(0, alt_m=0.0)], 'alt_m'),
    ([wp(0, acceptance_radius_m=-1.0)], 'acceptance_radius_m'),
    ([wp(0, max_vel_mps=2.5)], 'max_vel_mps'),
    ([wp(0, loiter_s=float('nan'))], 'nan'),
    ([wp(0), wp(1, marker=3)], 'waypoint 1'),
])
def test_tu_choi_ke_hoach_sai(raw, loi):
    fsm = m.MissionFsm()
    ly_do = fsm.load_plan(1, raw, 0, 0.0, TAGS)
    assert loi in ly_do and fsm.waypoints == []


def test_chi_nhan_ke_hoach_khi_idle_va_chua_cho_cat_canh():
    fsm = m.MissionFsm()
    fsm.request_start(0.0)
    assert 'cho cat canh' in fsm.load_plan(1, [wp(0)], 0, 0.0, TAGS)
    assert 'IDLE' in fsm_dang_ha_canh().load_plan(1, [wp(0)], 0, 0.0, TAGS)


def test_parse_known_tags():
    assert m.parse_known_tags([0.0, 1.0, 2.0, 3.0]) == {0: (1.0, 2.0, 3.0)}
    with pytest.raises(ValueError):
        m.parse_known_tags([0.0])


# ---- Buoc 2: ha canh chinh xac ------------------------------------------------------------

def fsm_dang_treo(waypoints=None):
    fsm = m.MissionFsm(params=m.Params(takeoff_alt_m=1.0))
    fsm.load_plan(1, waypoints or [wp(0)], 0, 0.0, TAGS)
    fsm.request_start(0.0)
    fsm.step(snap(0.0, arm_ready=True))
    fsm.step(snap(0.5, armed=True))
    assert fsm.state == m.TAKEOFF
    return fsm


def tren_tag(fsm, z=1.0):
    x, y, _ = fsm.current_waypoint().target
    return (x, y, z)


def fsm_dang_tim_tag(waypoints=None):
    """TAKEOFF -> ENROUTE (0,6) -> MARKER_SEARCH (0,7)."""
    fsm = fsm_dang_treo(waypoints)
    fsm.step(snap(0.6, armed=True, range_m=1.0))
    assert fsm.state == m.ENROUTE
    fsm.step(snap(0.7, armed=True, range_m=1.0, position=tren_tag(fsm)))
    assert fsm.state == m.MARKER_SEARCH
    return fsm


def fsm_dang_ha_chinh_xac(waypoints=None):
    """... -> PRECISION_LAND luc 1,0."""
    fsm = fsm_dang_tim_tag(waypoints)
    fsm.step(snap(1.0, armed=True, range_m=1.0, target_offset_m=0.2))
    assert fsm.state == m.PRECISION_LAND
    return fsm


def test_phat_expected_marker_id_va_cho_bat_tag_roi_ve_marker_search():
    fsm = fsm_dang_ha_chinh_xac([wp(0, marker=1)])
    act = fsm.step(snap(1.5, armed=True, range_m=1.0))
    assert act.expected_marker_id == 1 and act.velocity_up_mps == 0.0
    assert 'cho bat tag' in act.detail
    fsm.step(snap(2.1, armed=True, range_m=1.0))
    assert fsm.state == m.MARKER_SEARCH                 # khong ha mu


def test_chi_xuong_khi_vao_tam():
    fsm = fsm_dang_ha_chinh_xac()
    # do cao 1,0 m -> cho phep lech 0,10 + 0,25 = 0,35 m
    act = fsm.step(snap(1.2, armed=True, range_m=1.0, target_offset_m=0.40))
    assert act.velocity_up_mps == 0.0 and 'can tam' in act.detail
    act = fsm.step(snap(1.3, armed=True, range_m=1.0, target_offset_m=0.30))
    assert act.velocity_up_mps == -m.LAND_DESCENT_MPS


def test_mat_tag_giua_chung_thi_khong_ha_mu():
    fsm = fsm_dang_ha_chinh_xac()
    fsm.step(snap(1.2, armed=True, range_m=1.0, target_offset_m=0.1))
    fsm.step(snap(3.0, armed=True, range_m=0.8, target_offset_m=None))
    assert fsm.state == m.MARKER_SEARCH


def test_sat_dat_xuong_tiep_khong_can_tag_roi_disarm_va_hoan_thanh():
    fsm = fsm_dang_ha_chinh_xac()
    low = m.PRECISION_BLIND_BELOW_M - 0.05
    act = fsm.step(snap(3.0, armed=True, range_m=low, target_offset_m=None))
    assert fsm.state == m.PRECISION_LAND and act.velocity_up_mps == -m.LAND_DESCENT_MPS
    act = fsm.step(snap(4.0, armed=True, range_m=0.18, landed=True, disarm_ready=True))
    assert act.fc_command == 'disarm'
    fsm.step(snap(4.5, armed=False, range_m=0.18))
    assert fsm.state == m.MISSION_COMPLETE


def test_cham_dat_o_diem_co_hanh_dong_thi_sang_actuate_gripper_khong_disarm():
    fsm = fsm_dang_ha_chinh_xac([wp(0, action=m.ACTION_DROPOFF)])
    act = fsm.step(snap(3.0, armed=True, range_m=0.18, landed=True, disarm_ready=True))
    assert fsm.state == m.ACTUATE_GRIPPER and act.fc_command == ''


def test_bi_disarm_ngoai_y_muon_sang_failsafe():
    fsm = fsm_dang_ha_chinh_xac()
    fsm.step(snap(2.0, armed=False, range_m=1.0))
    assert fsm.state == m.FAILSAFE


def test_precision_land_khong_biet_quyen_giu_vz_0():
    fsm = fsm_dang_ha_chinh_xac()
    act = fsm.step(snap(1.2, armed=True, ob_auth=None, range_m=1.0, target_offset_m=0.0))
    assert act.velocity_up_mps == 0.0


def test_yeu_cau_ha_canh_khan_trong_precision_land():
    fsm = fsm_dang_ha_chinh_xac()
    fsm.request_land()
    fsm.step(snap(1.2, armed=True, range_m=1.0))
    assert fsm.state == m.EMERGENCY_LAND


# ---- Buoc 4: bay toi diem, tim tag, thu lai ------------------------------------------------

def test_cat_canh_khong_ke_hoach_thi_giu_co_ke_hoach_thi_bay_toi_diem():
    fsm = m.MissionFsm(params=m.Params(takeoff_alt_m=1.0))
    fsm.request_start(0.0)
    fsm.step(snap(0.0, arm_ready=True))
    fsm.step(snap(0.5, armed=True))
    fsm.step(snap(1.0, armed=True, range_m=1.0))
    assert fsm.state == m.TAKEOFF
    fsm = fsm_dang_treo([wp(0, marker=1, alt_m=2.0)])
    fsm.step(snap(0.6, armed=True, range_m=1.0))
    assert fsm.state == m.ENROUTE and fsm.retry_count == 0
    act = fsm.step(snap(0.8, armed=True, range_m=1.0, position=(0.0, 0.0, 1.0)))
    assert act.position_target == (10.0, 0.0, 2.0) and act.velocity_up_mps is None
    assert act.expected_marker_id == -1                 # khong bam tag khi dang bay


def test_toi_diem_chi_xet_khoang_cach_ngang():
    fsm = fsm_dang_treo([wp(0, marker=1, acceptance_radius_m=0.3)])
    fsm.step(snap(0.6, armed=True, range_m=1.0))
    fsm.step(snap(0.8, armed=True, range_m=1.0, position=None))
    assert fsm.state == m.ENROUTE                       # chua co EKF: khong tu coi la toi
    fsm.step(snap(1.0, armed=True, range_m=1.0, position=(9.6, 0.0, 0.5)))
    assert fsm.state == m.ENROUTE
    act = fsm.step(snap(1.2, armed=True, range_m=1.0, position=(9.8, 0.1, 7.0)))
    assert fsm.state == m.MARKER_SEARCH and act.expected_marker_id == 1


def test_tim_tag_giu_tai_diem_thay_dung_tag_thi_ha():
    fsm = fsm_dang_tim_tag([wp(0, marker=1)])
    act = fsm.step(snap(0.9, armed=True, range_m=1.0))
    assert fsm.state == m.MARKER_SEARCH
    assert act.expected_marker_id == 1 and act.position_target == fsm.current_waypoint().target
    act = fsm.step(snap(1.0, armed=True, range_m=1.0, target_offset_m=0.5))
    assert fsm.state == m.PRECISION_LAND and act.expected_marker_id == 1


def test_het_gio_tim_thu_lai_roi_het_luot_thi_ha_canh_tai_cho():
    fsm = fsm_dang_treo([wp(0)])
    fsm.max_retries, fsm.search_timeout_s = 2, 5.0
    fsm.step(snap(0.6, armed=True, range_m=1.0))
    fsm.step(snap(0.7, armed=True, range_m=1.0, position=tren_tag(fsm)))
    t = 0.7
    for lan in (1, 2):
        fsm.step(snap(t + 4.9, armed=True, range_m=1.0))
        assert fsm.state == m.MARKER_SEARCH
        t += 5.01
        fsm.step(snap(t, armed=True, range_m=1.0))
        assert fsm.state == m.RETRY_LOITER and fsm.retry_count == lan
        act = fsm.step(snap(t + 1.0, armed=True, range_m=1.0))
        assert act.position_target == fsm.current_waypoint().target
        assert act.expected_marker_id == -1
        t += m.RETRY_LOITER_S + 0.01
        act = fsm.step(snap(t, armed=True, range_m=1.0))
        assert fsm.state == m.MARKER_SEARCH and act.expected_marker_id == 0
        assert fsm.retry_count == lan                   # khong bi xoa khi tim lai
    act = fsm.step(snap(t + 5.01, armed=True, range_m=1.0))
    assert fsm.state == m.EMERGENCY_LAND and 'het 2 lan' in act.detail


def test_failsafe_retry_loiter_khi_tim_tag_la_mot_lan_that_bai():
    fsm = fsm_dang_tim_tag()
    fsm.step(snap(1.0, armed=True, range_m=1.0, failsafe_escalate_to=m.ESCALATE_RETRY_LOITER))
    assert fsm.state == m.RETRY_LOITER and fsm.retry_count == 1


def test_mat_tag_khi_ha_lap_lai_khong_vo_han():
    fsm = fsm_dang_ha_chinh_xac()                       # max_retries = 3 (params)
    t = 1.0
    for _ in range(3):
        fsm.step(snap(t + m.PRECISION_ACQUIRE_S + 0.1, armed=True, range_m=1.0))
        assert fsm.state == m.MARKER_SEARCH
        t += m.PRECISION_ACQUIRE_S + 0.2
        fsm.step(snap(t, armed=True, range_m=1.0, target_offset_m=0.1))
        assert fsm.state == m.PRECISION_LAND
    fsm.step(snap(t + m.PRECISION_ACQUIRE_S + 0.1, armed=True, range_m=1.0))
    assert fsm.state == m.EMERGENCY_LAND


def test_chang_moi_xoa_retry_count():
    fsm = fsm_dang_tim_tag()
    fsm.step(snap(1.0, armed=True, range_m=1.0, failsafe_escalate_to=m.ESCALATE_RETRY_LOITER))
    assert fsm.retry_count == 1
    fsm.state = m.ACTUATE_GRIPPER                       # gia lap xong diem (trang thai chua viet)
    fsm.transition(m.ENROUTE, 2.0)
    assert fsm.retry_count == 0


@pytest.mark.parametrize('trang_thai', ['dang_bay', 'dang_tim'])
def test_yeu_cau_ha_canh_va_khong_biet_quyen_khi_bay_toi_diem(trang_thai):
    fsm = fsm_dang_treo()
    fsm.step(snap(0.6, armed=True, range_m=1.0))
    if trang_thai == 'dang_tim':
        fsm.step(snap(0.7, armed=True, range_m=1.0, position=tren_tag(fsm)))
    act = fsm.step(snap(0.8, armed=True, ob_auth=None, range_m=1.0))
    assert act.velocity_up_mps == 0.0 and act.position_target is None
    fsm.request_land()
    fsm.step(snap(0.9, armed=True, range_m=1.0))
    assert fsm.state == m.EMERGENCY_LAND


# ---- Buoc 3: failsafe ----------------------------------------------------------------------

def test_failsafe_dang_bat_thi_khong_arm():
    fsm = m.MissionFsm()
    fsm.request_start(0.0)
    act = fsm.step(snap(0.0, arm_ready=True, failsafe_escalate_to=m.ESCALATE_LOITER))
    assert act.fc_command == '' and 'failsafe' in act.detail
    # het su co: KHONG tu arm tu yeu cau cu
    assert fsm.step(snap(1.0, arm_ready=True)).fc_command == ''


@pytest.mark.parametrize('esc', [m.ESCALATE_RTH, m.ESCALATE_EMERGENCY_LAND])
def test_rth_va_emergency_land_ha_canh_tai_cho(esc):
    fsm = fsm_dang_treo()
    fsm.step(snap(1.0, armed=True, range_m=1.0, failsafe_escalate_to=esc))
    assert fsm.state == m.EMERGENCY_LAND
    fsm = fsm_dang_ha_chinh_xac()
    fsm.step(snap(1.2, armed=True, range_m=1.0, failsafe_escalate_to=esc))
    assert fsm.state == m.EMERGENCY_LAND


def test_loiter_giu_vi_tri_het_su_co_thi_ha_canh():
    fsm = fsm_dang_ha_chinh_xac()
    fsm.step(snap(1.2, armed=True, range_m=1.0, failsafe_escalate_to=m.ESCALATE_LOITER))
    assert fsm.state == m.FAILSAFE
    act = fsm.step(snap(1.4, armed=True, range_m=1.0, failsafe_escalate_to=m.ESCALATE_LOITER))
    assert act.velocity_up_mps == 0.0 and act.fc_command == ''
    fsm.step(snap(1.6, armed=True, range_m=1.0, failsafe_escalate_to=m.ESCALATE_RTH))
    assert fsm.state == m.EMERGENCY_LAND                # leo thang tiep khi dang loiter
    fsm = fsm_dang_ha_chinh_xac()
    fsm.step(snap(1.2, armed=True, failsafe_escalate_to=m.ESCALATE_LOITER))
    fsm.step(snap(2.0, armed=True, failsafe_escalate_to=m.ESCALATE_NONE))
    assert fsm.state == m.EMERGENCY_LAND                # khong tu tiep tuc nhiem vu


def test_loiter_co_yeu_cau_ha_canh_thi_ha():
    fsm = fsm_dang_treo()
    fsm.step(snap(1.0, armed=True, failsafe_escalate_to=m.ESCALATE_LOITER))
    fsm.request_land()
    fsm.step(snap(1.2, armed=True, failsafe_escalate_to=m.ESCALATE_LOITER))
    assert fsm.state == m.EMERGENCY_LAND


def test_retry_loiter_khong_cuop_quyen_trang_thai():
    fsm = fsm_dang_ha_chinh_xac()
    fsm.step(snap(1.2, armed=True, range_m=1.0, target_offset_m=0.0,
                  failsafe_escalate_to=m.ESCALATE_RETRY_LOITER))
    assert fsm.state == m.PRECISION_LAND


def test_mat_quyen_uu_tien_hon_failsafe_khong_gui_gi():
    fsm = fsm_dang_treo()
    act = fsm.step(snap(1.0, armed=True, ob_auth=False, failsafe_escalate_to=m.ESCALATE_RTH))
    assert fsm.state == m.FAILSAFE and act.velocity_up_mps is None
    act = fsm.step(snap(1.5, armed=True, ob_auth=False, failsafe_escalate_to=m.ESCALATE_RTH))
    assert act.velocity_up_mps is None and act.fc_command == ''


def test_dang_ha_canh_khan_thi_bo_qua_failsafe_nhe_hon():
    fsm = fsm_dang_ha_canh()
    fsm.step(snap(2.0, armed=True, failsafe_escalate_to=m.ESCALATE_LOITER))
    assert fsm.state == m.EMERGENCY_LAND


def test_hang_so_escalate_khop_failsafe_event():
    from drone_interfaces.msg import FailsafeEvent as F
    assert (m.ESCALATE_NONE, m.ESCALATE_LOITER, m.ESCALATE_RETRY_LOITER, m.ESCALATE_RTH,
            m.ESCALATE_EMERGENCY_LAND) == (F.ESCALATE_NONE, F.ESCALATE_LOITER,
                                           F.ESCALATE_RETRY_LOITER, F.ESCALATE_RTH,
                                           F.ESCALATE_EMERGENCY_LAND)


# ---------------------------------------------------------------- RTH (ve nha)

def fsm_dang_bay_toi_diem():
    """TAKEOFF -> ENROUTE, da chot nha o (0, 0), dang o giua duong toi tag.

    pos_anchored=True la TIEN DE bat buoc: chua neo thi FSM co y khong chot nha (xem
    test_chua_neo_thi_khong_chot_nha).
    """
    fsm = fsm_dang_treo([wp(0, marker=1)])
    fsm.step(snap(0.5, armed=True, range_m=0.2, position=(0.0, 0.0, 0.2), pos_anchored=True))
    fsm.step(snap(0.6, armed=True, range_m=1.0, position=(0.0, 0.0, 1.0), pos_anchored=True))
    assert fsm.state == m.ENROUTE
    assert fsm.home == (0.0, 0.0, 0.2)
    return fsm


def test_muc_3_bay_ve_nha_chu_khong_ha_tai_cho():
    fsm = fsm_dang_bay_toi_diem()
    act = fsm.step(snap(1.0, armed=True, range_m=2.0, position=(3.0, 0.0, 2.0),
                        failsafe_escalate_to=m.ESCALATE_RTH))
    assert fsm.state == m.RTH
    assert fsm.rth_alt_m == 2.0
    # Diem den la NHA, giu do cao luc vao RTH.
    act = fsm.step(snap(1.2, armed=True, range_m=2.0, position=(3.0, 0.0, 2.0),
                        failsafe_escalate_to=m.ESCALATE_RTH))
    assert act.position_target == (0.0, 0.0, 2.0)


def test_muc_4_ha_ngay_khong_ve_nha():
    """Pin kiet / mat FC: moi giay bay them deu la rui ro - khong duoc bay ve nha."""
    fsm = fsm_dang_bay_toi_diem()
    fsm.step(snap(1.0, armed=True, range_m=2.0, position=(3.0, 0.0, 2.0),
                  failsafe_escalate_to=m.ESCALATE_EMERGENCY_LAND))
    assert fsm.state == m.EMERGENCY_LAND


def test_rth_toi_nha_thi_ha_canh():
    fsm = fsm_dang_bay_toi_diem()
    fsm.step(snap(1.0, armed=True, range_m=2.0, position=(3.0, 0.0, 2.0),
                  failsafe_escalate_to=m.ESCALATE_RTH))
    assert fsm.state == m.RTH
    fsm.step(snap(2.0, armed=True, range_m=2.0, position=(0.5, 0.0, 2.0),
                  failsafe_escalate_to=m.ESCALATE_RTH))
    assert fsm.state == m.EMERGENCY_LAND


def test_rth_qua_han_thi_ha_tai_cho():
    fsm = fsm_dang_bay_toi_diem()
    fsm.step(snap(1.0, armed=True, range_m=2.0, position=(3.0, 0.0, 2.0),
                  failsafe_escalate_to=m.ESCALATE_RTH))
    xa = (50.0, 0.0, 2.0)
    fsm.step(snap(1.0 + m.RTH_TIMEOUT_S + 0.1, armed=True, range_m=2.0, position=xa,
                  failsafe_escalate_to=m.ESCALATE_RTH))
    assert fsm.state == m.EMERGENCY_LAND


def test_mat_vi_tri_khi_rth_thi_ha_tai_cho():
    fsm = fsm_dang_bay_toi_diem()
    fsm.step(snap(1.0, armed=True, range_m=2.0, position=(3.0, 0.0, 2.0),
                  failsafe_escalate_to=m.ESCALATE_RTH))
    fsm.step(snap(1.5, armed=True, range_m=2.0, position=None,
                  failsafe_escalate_to=m.ESCALATE_RTH))
    assert fsm.state == m.EMERGENCY_LAND


def test_khong_biet_nha_thi_khong_rth():
    """Chua tung co vi tri EKF luc cat canh -> khong RTH duoc, ha canh tai cho."""
    fsm = fsm_dang_treo([wp(0, marker=1)])
    fsm.step(snap(0.6, armed=True, range_m=1.0, position=None))
    assert fsm.state == m.ENROUTE and fsm.home is None
    fsm.step(snap(1.0, armed=True, range_m=2.0, position=(3.0, 0.0, 2.0),
                  failsafe_escalate_to=m.ESCALATE_RTH))
    assert fsm.state == m.EMERGENCY_LAND


def test_ekf_hong_khi_dang_rth_thi_dung_lai_giu_vi_tri():
    """EKF khong healthy -> vi tri khong tin duoc -> khong bay tiep ve nha."""
    fsm = fsm_dang_bay_toi_diem()
    fsm.step(snap(1.0, armed=True, range_m=2.0, position=(3.0, 0.0, 2.0),
                  failsafe_escalate_to=m.ESCALATE_RTH))
    assert fsm.state == m.RTH
    fsm.step(snap(1.5, armed=True, range_m=2.0, position=(3.0, 0.0, 2.0),
                  failsafe_escalate_to=m.ESCALATE_LOITER))
    assert fsm.state == m.FAILSAFE


# ------------------------------------------------------- ACTUATE_GRIPPER (gap/tha)

def fsm_dang_gap(action=None, waypoints=None):
    """... -> cham dat tai diem co hanh dong -> ACTUATE_GRIPPER luc 3,0."""
    wps = waypoints or [wp(0, marker=1, action=action or m.ACTION_PICKUP)]
    fsm = fsm_dang_ha_chinh_xac(wps)
    fsm.step(snap(3.0, armed=True, range_m=0.18, landed=True, disarm_ready=True))
    assert fsm.state == m.ACTUATE_GRIPPER
    return fsm


def gap_snap(t, **kw):
    return snap(t, armed=True, range_m=0.18, landed=True, disarm_ready=True, **kw)


def test_giu_on_dinh_roi_moi_ra_lenh_gap():
    """Vua cham dat, khung may con rung - khong duoc cho co cau chay ngay."""
    fsm = fsm_dang_gap()
    act = fsm.step(gap_snap(3.5))           # pre_dropoff_settle_s mac dinh 2,0 s
    assert act.gripper_command == '' and 'on dinh' in act.detail
    act = fsm.step(gap_snap(5.5))
    assert act.gripper_command == 'close'


def test_pickup_cho_xac_nhan_cam_bien_khong_dung_timeout():
    """Nguyen tac 3: het gio KHONG duoc coi la da gap."""
    fsm = fsm_dang_gap(m.ACTION_PICKUP)
    fsm.step(gap_snap(5.5))
    for t in (7.0, 20.0, 60.0):
        fsm.step(gap_snap(t, gripper_sensor_confirmed=False))
        assert fsm.state == m.ACTUATE_GRIPPER
    fsm.step(gap_snap(61.0, gripper_sensor_confirmed=True))
    assert fsm.state != m.ACTUATE_GRIPPER


def test_dropoff_xong_khi_cam_bien_NHA_ra():
    """DROPOFF nguoc PICKUP: xong = sensor_confirmed ve False, khong phai True."""
    fsm = fsm_dang_gap(m.ACTION_DROPOFF)
    act = fsm.step(gap_snap(5.5, gripper_sensor_confirmed=True))
    assert act.gripper_command == 'open'
    fsm.step(gap_snap(6.0, gripper_sensor_confirmed=True))
    assert fsm.state == m.ACTUATE_GRIPPER           # con dang giu -> chua xong
    fsm.step(gap_snap(6.5, gripper_sensor_confirmed=False))
    assert fsm.state == m.EMERGENCY_LAND            # diem cuoi -> ha canh va disarm


def test_gian_nhip_phat_lai_lenh_gripper():
    fsm = fsm_dang_gap()
    assert fsm.step(gap_snap(5.5)).gripper_command == 'close'
    assert fsm.step(gap_snap(5.7)).gripper_command == ''      # chua du GRIPPER_RETRY_S
    assert fsm.step(gap_snap(5.5 + m.GRIPPER_RETRY_S)).gripper_command == 'close'


def test_gripper_khong_xac_nhan_thi_thu_lai_ca_chang():
    fsm = fsm_dang_gap()
    fsm.max_retries = 2
    fsm.step(gap_snap(5.5))
    fsm.step(gap_snap(6.0, failsafe_escalate_to=m.ESCALATE_RETRY_LOITER))
    assert fsm.state == m.RETRY_LOITER and fsm.retry_count == 1


def test_het_luot_thu_lai_gripper_thi_ha_canh():
    fsm = fsm_dang_gap()
    fsm.max_retries = 0
    fsm.step(gap_snap(5.5))
    fsm.step(gap_snap(6.0, failsafe_escalate_to=m.ESCALATE_RETRY_LOITER))
    assert fsm.state == m.EMERGENCY_LAND


def test_con_diem_sau_thi_cat_canh_lai():
    fsm = fsm_dang_gap(waypoints=[wp(0, marker=1, action=m.ACTION_PICKUP), wp(1, marker=0)])
    fsm.step(gap_snap(5.5))
    fsm.step(gap_snap(6.0, gripper_sensor_confirmed=True))
    assert fsm.state == m.TAKEOFF and fsm.current_wp_index == 1


def test_yeu_cau_ha_canh_thoat_duoc_actuate_gripper():
    """Thieu EMERGENCY_LAND trong bang chuyen thi ~/land bao thanh cong nhung khong lam gi."""
    fsm = fsm_dang_gap()
    fsm.request_land()
    fsm.step(gap_snap(5.5))
    assert fsm.state == m.EMERGENCY_LAND


def test_diem_khong_co_hanh_dong_khong_dinh_gripper():
    """ACTION_NONE o diem KHONG phai diem cuoi: ghe roi di tiep, tuyet doi khong lenh gripper.

    Truoc khi sua, ham chi phan hai nhanh (PICKUP / con lai) nen diem nay bi xu ly nhu DROPOFF:
    log ghi "tha hang" cho mot diem khong tha gi. Giao uoc GCS<->Pi 3.2b cau 3.
    """
    fsm = fsm_dang_gap(waypoints=[wp(0, marker=1, action=m.ACTION_NONE), wp(1, marker=0)])
    act = fsm.step(gap_snap(3.5))
    assert act.gripper_command == '' and 'khong co hanh dong' in act.detail
    act = fsm.step(gap_snap(5.5))
    assert act.gripper_command == ''
    assert 'tha' not in act.detail and 'gap' not in act.detail    # khong noi doi nua
    assert fsm.state == m.TAKEOFF and fsm.current_wp_index == 1


def test_chua_neo_thi_khong_chot_nha_va_rth_ha_tai_cho():
    """Truoc khi odom neo theo bang tag, toa do thuoc khung khac va se NHAY khi neo.

    Chot nha luc do thi sau khi neo con so ay tro sang mot cho vat ly khac: da tai hien duoc
    canh cat canh lech pad_home 3 m thi RTH ve pad_home chu khong ve diem cat canh.
    Nay khong chot -> HOME_VALID = 0 -> RTH tu roi ve ha canh tai cho. Giao uoc GCS<->Pi 5.2b y 4.
    """
    fsm = fsm_dang_treo([wp(0, marker=1)])
    fsm.step(snap(0.5, armed=True, range_m=0.2, position=(0.0, 0.0, 0.2)))   # pos_anchored=False
    fsm.step(snap(0.6, armed=True, range_m=1.0, position=(0.0, 0.0, 1.0)))
    assert fsm.state == m.ENROUTE
    assert fsm.home is None, 'khong duoc chot nha khi odom chua neo'
    fsm.step(snap(1.0, armed=True, range_m=2.0, position=(3.0, 0.0, 2.0),
                  failsafe_escalate_to=m.ESCALATE_RTH))
    assert fsm.state == m.EMERGENCY_LAND          # ha tai cho, KHONG bay ve mot diem sai


def test_neo_muon_trong_luc_leo_van_chot_duoc_nha():
    """Drone khong thay pad cua chinh no luc nam dat; neo xay ra ~0,6 s sau khi roi dat, luc
    x, y van la cua diem cat canh (leo thang dung)."""
    fsm = fsm_dang_treo([wp(0, marker=1)])
    fsm.step(snap(0.5, armed=True, range_m=0.1, position=(2.0, 1.0, 0.1)))   # chua neo
    assert fsm.home is None
    fsm.step(snap(0.9, armed=True, range_m=0.4, position=(2.0, 1.0, 0.4), pos_anchored=True))
    assert fsm.home == (2.0, 1.0, 0.4), 'neo trong luc leo thi van phai chot duoc nha'


# ------------------------------------------------- lenh tu GCS (giao uoc GCS muc 4.1)

def test_rth_tu_gcs_khi_chua_biet_nha_bi_tu_choi_SOM():
    """Tu choi som, khong nhan roi im lang ha canh tai cho: nguoi van hanh bam "ve nha" ma drone
    ha xuong cho la la kieu bat ngo te nhat."""
    fsm = fsm_dang_treo([wp(0, marker=1)])
    fsm.step(snap(0.6, armed=True, range_m=1.0, position=None))   # chua neo -> home = None
    assert fsm.home is None
    ly_do = fsm.request_rth()
    assert ly_do and 'chua biet nha' in ly_do
    assert fsm.rth_requested is False


def test_rth_tu_gcs_khi_da_biet_nha():
    fsm = fsm_dang_bay_toi_diem()
    assert fsm.request_rth() == ''
    fsm.step(snap(1.0, armed=True, range_m=2.0, position=(3.0, 0.0, 2.0)))
    assert fsm.state == m.RTH


def test_rth_khi_idle_bi_tu_choi():
    fsm = m.MissionFsm()
    assert 'IDLE' in fsm.request_rth()


def test_abort_khi_dang_bay_thi_bo_ke_hoach_roi_ha_canh():
    fsm = fsm_dang_bay_toi_diem()
    fsm.request_abort()
    fsm.step(snap(1.0, armed=True, range_m=2.0, position=(3.0, 0.0, 2.0)))
    assert fsm.state == m.EMERGENCY_LAND
    assert fsm.waypoints == [], 'phai bo ke hoach TRUOC khi ha, de cham dat khong di tiep diem nao'


def test_abort_khi_da_disarm_thi_ve_idle_qua_failsafe():
    """IDLE khong nam trong bang chuyen cua ENROUTE, va da disarm ma con o trang thai bay la
    BAT THUONG - FAILSAFE la duong danh cho bat thuong va tu ve IDLE khi thay khong armed."""
    fsm = fsm_dang_bay_toi_diem()
    fsm.request_abort()
    fsm.step(snap(1.0, armed=False, range_m=0.1, position=(3.0, 0.0, 0.1)))
    assert fsm.state == m.FAILSAFE and fsm.waypoints == []
    fsm.step(snap(1.2, armed=False, range_m=0.1, position=(3.0, 0.0, 0.1)))
    assert fsm.state == m.IDLE


def test_abort_khi_dang_idle_chi_bo_ke_hoach():
    fsm = m.MissionFsm()
    fsm.load_plan(1, [wp(0, marker=1)], 0, 0.0, TAGS)
    assert len(fsm.waypoints) == 1
    fsm.request_abort()
    fsm.step(snap(1.0, armed=False))
    assert fsm.state == m.IDLE and fsm.waypoints == []


def test_abort_khong_bao_gio_bi_tu_choi():
    """Huy phai luon di duoc, ke ca khi FSM dang o trang thai la."""
    fsm = m.MissionFsm()
    fsm.request_abort()
    assert fsm.abort_requested is True


def test_abort_xong_thi_bao_MISSION_COMPLETE_chu_khong_FAILSAFE():
    """Lenh huy hoan thanh dung phai hien MISSION_COMPLETE. Giu abort_requested sau khi da vao
    duong ha canh se chan duong do va bat GCS hien FAILSAFE oan."""
    fsm = fsm_dang_bay_toi_diem()
    fsm.request_abort()
    fsm.step(snap(1.0, armed=True, range_m=2.0, position=(3.0, 0.0, 2.0)))
    assert fsm.state == m.EMERGENCY_LAND and fsm.abort_requested is False
    fsm.step(snap(2.0, armed=False, range_m=0.1, landed=True, position=(3.0, 0.0, 0.1)))
    assert fsm.state == m.MISSION_COMPLETE


# --------------------------------------------------------------------------------------------
# P30 (giao uoc GCS 11.5): "nhiem vu da xong" phai co dau hieu doc duoc tren day.
# --------------------------------------------------------------------------------------------

def test_mission_complete_song_du_lau_cho_telemetry_2hz():
    """Vong FSM 5 Hz ma telemetry 2 Hz: trang thai chi song mot tick thi gan nhu khong bao gio
    len duoc day. GCS do hai chuyen bay that, khong bat duoc lan nao."""
    assert m.MISSION_COMPLETE_HOLD_S > 0.5, 'phai dai hon mot chu ky telemetry (0,5 s)'
    fsm = fsm_dang_bay_toi_diem()
    fsm.request_abort()
    fsm.step(snap(1.0, armed=True, range_m=2.0, position=(3.0, 0.0, 2.0)))
    fsm.step(snap(2.0, armed=False, range_m=0.1, landed=True, position=(3.0, 0.0, 0.1)))
    assert fsm.state == m.MISSION_COMPLETE
    # Ba mau telemetry lien tiep cach nhau 0,5 s deu phai thay trang thai 9.
    for t in (2.1, 2.6, 3.1):
        fsm.step(snap(t, armed=False, range_m=0.1, landed=True, position=(3.0, 0.0, 0.1)))
        assert fsm.state == m.MISSION_COMPLETE, f'mat trang thai 9 o t = {t}'


def test_huy_va_lam_xong_deu_toi_MISSION_COMPLETE_nen_can_mission_result():
    """Ly do ton tai cua mission_result: rieng trang thai 9 KHONG phan biet duoc hai chuyen nay."""
    huy = fsm_dang_bay_toi_diem()
    huy.request_abort()
    huy.step(snap(1.0, armed=True, range_m=2.0, position=(3.0, 0.0, 2.0)))
    huy.step(snap(2.0, armed=False, range_m=0.1, landed=True, position=(3.0, 0.0, 0.1)))

    xong = fsm_dang_bay_toi_diem()
    xong.waypoints = []          # het ke hoach
    xong.request_land()
    xong.step(snap(1.0, armed=True, range_m=2.0, position=(3.0, 0.0, 2.0)))
    xong.step(snap(2.0, armed=False, range_m=0.1, landed=True, position=(3.0, 0.0, 0.1)))

    assert huy.state == xong.state == m.MISSION_COMPLETE
    assert huy.mission_result == m.RESULT_ABORTED
    assert xong.mission_result == m.RESULT_LANDED_CMD
    assert huy.mission_result != xong.mission_result


def test_mission_result_chot_lai_sau_khi_da_ve_idle():
    """R5: GCS khoi dong lai bao lau sau van phai hoi lai duoc ket qua chuyen truoc."""
    fsm = fsm_dang_bay_toi_diem()
    fsm.request_abort()
    fsm.step(snap(1.0, armed=True, range_m=2.0, position=(3.0, 0.0, 2.0)))
    fsm.step(snap(2.0, armed=False, range_m=0.1, landed=True, position=(3.0, 0.0, 0.1)))
    fsm.step(snap(2.0 + m.MISSION_COMPLETE_HOLD_S, armed=False, landed=True))
    assert fsm.state == m.IDLE
    fsm.step(snap(600.0, armed=False, landed=True))       # muoi phut sau
    assert fsm.state == m.IDLE
    assert fsm.mission_result == m.RESULT_ABORTED, 'ket qua phai con nguyen khi dang dau'


def test_mission_result_xoa_khi_cat_canh_chuyen_moi():
    fsm = fsm_dang_bay_toi_diem()
    fsm.mission_result = m.RESULT_ABORTED
    fsm.state = m.IDLE
    fsm.load_plan(2, [wp(0, marker=1)], 0, 0.0, TAGS)
    fsm.transition(m.TAKEOFF, 10.0)
    assert fsm.mission_result == m.RESULT_UNKNOWN


def test_het_luot_thu_khong_duoc_bao_la_hoan_thanh():
    """Ha canh vi het luot tim tag ket thuc o MISSION_COMPLETE y het chuyen thanh cong."""
    fsm = fsm_dang_bay_toi_diem()
    fsm.max_retries = 0
    fsm._search_failed(1.0, m.RETRY_LOITER, 'thu')
    assert fsm.state == m.EMERGENCY_LAND
    fsm.step(snap(2.0, armed=False, range_m=0.1, landed=True, position=(3.0, 0.0, 0.1)))
    assert fsm.state == m.MISSION_COMPLETE
    assert fsm.mission_result == m.RESULT_RETRIES_EXHAUSTED


def test_ly_do_dau_tien_thang_khong_bi_abort_ghi_de_failsafe():
    fsm = fsm_dang_bay_toi_diem()
    fsm.step(snap(1.0, armed=True, range_m=2.0, position=(3.0, 0.0, 2.0),
                  failsafe_escalate_to=m.ESCALATE_EMERGENCY_LAND))
    assert fsm.mission_result == m.RESULT_FAILSAFE
    fsm.request_abort()
    fsm.step(snap(1.2, armed=True, range_m=2.0, position=(3.0, 0.0, 2.0)))
    assert fsm.mission_result == m.RESULT_FAILSAFE, 'ghi de se giau mat su co'


def test_abort_giua_luc_giu_MISSION_COMPLETE_khong_lam_fsm_ket():
    """TRANSITIONS[MISSION_COMPLETE] chi co IDLE - khong co nhanh FAILSAFE. Thieu duong ra rieng
    thi abort toi trong cua so giu 1,5 s se chan _step moi tick va FSM ket lai vinh vien."""
    fsm = fsm_dang_bay_toi_diem()
    fsm.waypoints = []
    fsm.request_land()
    fsm.step(snap(1.0, armed=True, range_m=2.0, position=(3.0, 0.0, 2.0)))
    fsm.step(snap(2.0, armed=False, range_m=0.1, landed=True, position=(3.0, 0.0, 0.1)))
    assert fsm.state == m.MISSION_COMPLETE
    fsm.request_abort()
    fsm.step(snap(2.2, armed=False, landed=True))
    assert fsm.state == m.IDLE and fsm.abort_requested is False
    fsm.step(snap(2.4, armed=False, landed=True))
    assert fsm.state == m.IDLE
