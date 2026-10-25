"""May trang thai nhiem vu, thuan Python - khong phu thuoc rclpy de pytest duoc moi nhanh loi
(mat marker, mat gripper confirm, het pin giua chung) truoc khi bay.

Ba nguyen tac bat buoc (muc 4.1 tai lieu huong dan):
  1. KHONG BAO GIO chuyen sang ha canh chi dua vao GPS/EKF - luon cho landing_target_bridge_node
     xac thuc dung ID marker truoc ("hai lop dinh vi bo tro");
  2. MARKER_SEARCH co retry_count/max_retries - het luot thi RETRY_LOITER roi bao failsafe,
     khong lap vo han;
  3. TUYET DOI khong dung timeout thay cho gripper_status.sensor_confirmed - hang co the chua
     gap chac du da du thoi gian.

Camera nghieng bang servo (2026-10-22, docs/ke_hoach_camera_servo_gap_hang.md): bay toi diem nhin
20 do ve mui, tim/ha theo tag nhin thang xuong, gap hang nghieng 20 do ve duoi. Gap/tha hang khi
TREO o ToF 0,20 m (co cau gap o tam drone), tag nho dan tren nap hop hang.
"""

import math
from dataclasses import dataclass, field

# Ten trang thai trung khop hang so cua drone_interfaces/MissionState.
IDLE = 'IDLE'
TAKEOFF = 'TAKEOFF'
ENROUTE = 'ENROUTE'
MARKER_SEARCH = 'MARKER_SEARCH'
PRECISION_LAND = 'PRECISION_LAND'
ACTUATE_GRIPPER = 'ACTUATE_GRIPPER'
RETRY_LOITER = 'RETRY_LOITER'
RTH = 'RTH'
EMERGENCY_LAND = 'EMERGENCY_LAND'
MISSION_COMPLETE = 'MISSION_COMPLETE'
FAILSAFE = 'FAILSAFE'
# Ban 0.8 cua giao uoc GCS da co san hai gia tri nay (MissionState 11, 13) - dung lai, khong doi enum.
ALIGN_HEADING = 'ALIGN_HEADING'     # treo tren tam bai, quay mui = huong bai roi moi ha
FINAL_APPROACH = 'FINAL_APPROACH'   # lay hang: tim tag tren hop, nghieng camera, ha toi do cao gap

# Bang chuyen trang thai hop le. Moi trang thai chi duoc di toi cac trang thai liet ke o day;
# nho vay them mot nhanh moi la sua bang nay chu khong phai doc het than ham.
TRANSITIONS = {
    IDLE:            [TAKEOFF, FAILSAFE],
    TAKEOFF:         [ENROUTE, FAILSAFE, EMERGENCY_LAND],
    ENROUTE:         [MARKER_SEARCH, RTH, FAILSAFE, EMERGENCY_LAND],
    # MARKER_SEARCH -> EMERGENCY_LAND: het luot tim. -> RTH: su co muc 3 khi dang treo tim tag,
    # ve nha tot hon la ha xuong cho la. -> ALIGN_HEADING: thay tag bai co huong (yaw_control).
    MARKER_SEARCH:   [PRECISION_LAND, ALIGN_HEADING, RETRY_LOITER, RTH, FAILSAFE, EMERGENCY_LAND],
    # ALIGN_HEADING -> FINAL_APPROACH / ACTUATE_GRIPPER: gap / tha tren bai co huong - da dung tren
    # diem hop tu luc quay mui, khong ha tren tam tag (user 10-25).
    ALIGN_HEADING:   [PRECISION_LAND, FINAL_APPROACH, ACTUATE_GRIPPER, MARKER_SEARCH, RTH, FAILSAFE,
                      EMERGENCY_LAND],
    PRECISION_LAND:  [ACTUATE_GRIPPER, FINAL_APPROACH, MARKER_SEARCH, FAILSAFE, MISSION_COMPLETE,
                      EMERGENCY_LAND],
    FINAL_APPROACH:  [ACTUATE_GRIPPER, RETRY_LOITER, RTH, FAILSAFE, EMERGENCY_LAND],
    # EMERGENCY_LAND PHAI co: thieu no thi ~/land tra 'thanh cong' nhung khong lam gi, va het
    # ke hoach cung khong ha canh duoc - drone nam dat nhung van arm.
    ACTUATE_GRIPPER: [TAKEOFF, ENROUTE, MISSION_COMPLETE, RETRY_LOITER, FAILSAFE,
                      EMERGENCY_LAND],
    RETRY_LOITER:    [MARKER_SEARCH, RTH, FAILSAFE, EMERGENCY_LAND],
    RTH:             [EMERGENCY_LAND, MISSION_COMPLETE, FAILSAFE],
    EMERGENCY_LAND:  [MISSION_COMPLETE, FAILSAFE],
    MISSION_COMPLETE: [IDLE],
    FAILSAFE:        [RTH, EMERGENCY_LAND, IDLE],
}


# Lop FC (GIAO_UOC_FC_ROS2.md 6.2, 6.3, 11.3 P2/P10).
ARM_RETRY_S = 2.0          # OB_ARM_RDY len day 2 Hz - khong gui ARM day hon muc do
ARM_WAIT_S = 30.0          # yeu cau cat canh het han neu nguoi lai khong trao quyen kip
DISARM_RETRY_S = 3.0       # P10: thu lai DISARM thuong moi 3 s khi da cham dat
TAKEOFF_CLIMB_MPS = 0.5
# Toc do ha (10-09): xa dat 0,25 m/s, gan dat (laser <= 1,2 m, trung moc xuong cham cua FC - hoac
# khong co laser) 0,10 m/s.
LAND_DESCENT_FAR_MPS = 0.25
LAND_DESCENT_NEAR_MPS = 0.10
LAND_NEAR_BELOW_M = 1.2
# Doan cuoi (10-10, yeu cau user): laser <= 0,25 m (cang con ~5-8 cm, laser doc ~0,17 m khi cham dat)
# thi ha NHANH. Sat dat khong nghieng sua vi tri duoc (de quet cang) va da can tam xong; xuong
# nhanh bot thoi gian gio day troi. 0,3 = tran xuong cham cua FC duoi 1,2 m (Pi tu kep 0,285).
LAND_FINAL_BELOW_M = 0.25
LAND_FINAL_MPS = 0.3
OB_STATE_KHOA = 0          # trung fc_link.OB_STATE_KHOA - FSM khong import ROS

# Giu MISSION_COMPLETE it nhat bay nhieu lau truoc khi ve IDLE. Vong FSM 5 Hz (200 ms) ma
# telemetry chi phat 2 Hz (500 ms), nen truoc day trang thai 9 song DUNG MOT tick va gan nhu
# khong bao gio len duoc day: GCS do hai chuyen bay that, khong bat duoc lan nao (giao uoc 11.5
# P30). 1,5 s = 3 goi telemetry, du ngay ca khi mat 2 goi.
MISSION_COMPLETE_HOLD_S = 1.5

# Ly do chuyen bay ket thuc - trung hang so drone_interfaces/msg/MissionState.RESULT_*.
# Ton tai vi MISSION_COMPLETE la dich chung cua ca huy lenh, RTH, NAV_LAND va het ke hoach.
RESULT_UNKNOWN = 0
RESULT_COMPLETED = 1
RESULT_ABORTED = 2
RESULT_RTH = 3
RESULT_LANDED_CMD = 4
RESULT_FAILSAFE = 5
RESULT_RETRIES_EXHAUSTED = 6

# Muc leo thang failsafe - trung drone_interfaces/FailsafeEvent.ESCALATE_*.
ESCALATE_NONE = 0
ESCALATE_LOITER = 1
ESCALATE_RETRY_LOITER = 2   # de trang thai nhiem vu tu xu ly (MARKER_SEARCH, ACTUATE_GRIPPER)
ESCALATE_RTH = 3            # bay ve diem cat canh roi ha (trang thai RTH)
ESCALATE_EMERGENCY_LAND = 4

# Ha canh chinh xac (PRECISION_LAND).
PRECISION_ACQUIRE_S = 1.0       # moi vao trang thai: cho bat tag toi da chung nay roi coi la mat
# Chi xuong khi tag lech ngang <= BASE + PER_M x do cao laser. 0,10 + 0,25 x h -> 0,04 + 0,10 x h
# (10-11): bay that ngoai troi gio day lech 10-13 cm ma o 0,35 m van cho lech 19 cm -> ha mu tu do,
# cham dat lech 10-15 cm huong ngau nhien. Nay o 0,35 m chi con 7,5 cm: chua dat thi dung lai can tam
# (dang thay tag) roi moi vao doan mu. Nguong nay cung chat hon gioi han lech TRUOC mui camera chiu
# duoc (tag nho con thay khi lech truoc 0,18 m o 0,5 m - test_approach_visibility).
PRECISION_ALIGN_BASE_M = 0.04
PRECISION_ALIGN_PER_M = 0.10
# Duoi do cao nay camera (thang xuong, truoc tam 9 cm) khong con thay tag to 25 cm tron ven (mep sau
# ra khoi khung tu ~0,29 m) - xuong tiep khong can tag. Tren muc nay mat tag la dung, KHONG ha mu.
PRECISION_BLIND_BELOW_M = 0.32
# Mat tag giua chung khi ha (10-10, yeu cau user): bong drone che tag, khung hinh hong... ma theo EKF
# drone van trong vung can tam (cung nguong PRECISION_ALIGN_*) thi HA TIEP toi da chung nay, truc
# ngang bam tam bai da biet (pad_obs) theo EKF. Troi ra ngoai (mat vi troi, khong phai vi nghieng
# xoay camera) thi dung ha, ve tam theo EKF cho tag; qua TAG_GAP_RECOVER_S moi bay len tim lai.
TAG_GAP_GRACE_S = 2.0
TAG_GAP_RECOVER_S = 4.0
LAND_EKF_MAX_VEL_MPS = 0.3      # tran ngang khi bam tam bai theo EKF luc mat tag

# Tim tag (MARKER_SEARCH / RETRY_LOITER).
# RETRY_LOITER giu tai diem chung nay roi tim lai. Dai hon chu ky failsafe_monitor_node (2 Hz) de
# su co FS_MARKER_TIMEOUT kip tat, khong bi dem them mot lan ngay khi vao lai MARKER_SEARCH.
RETRY_LOITER_S = 3.0
GRIPPER_RETRY_S = 1.0       # phat lai lenh gap/tha neu co cau chua doi trang thai
# RTH khong bao gio duoc keo dai vo han: su co kich RTH (pin yeu, mat GCS) deu la thu chi
# xau di theo thoi gian. Khong toi duoc nha trong ngan nay -> ha canh tai cho, con hon het pin
# giua duong.
RTH_TIMEOUT_S = 90.0
RTH_ACCEPT_M = 1.0          # toi nha trong ban kinh nay thi ha canh

# Camera nghieng bang servo: goc + = ve MUI, 0 = thang xuong (camera_tilt_node, /camera/tilt).
CAM_ENROUTE_DEG = 20.0          # bay toi diem: nhin ve phia truoc
CAM_DOWN_DEG = 0.0              # tim / can tam / ha theo tag
CAM_BOX_DEG = -20.0             # xuong theo tag hop: tag o sau camera (camera 9 cm truoc tam)
# Doan cuoi (ToF <= box_cam_switch_m): tag hop sat duoi, sau ong kinh ~6 cm -> nghieng them (user 10-22,
# do that: hop nang, mat tag ~1 cm duoi chan doc duoc 1150..1050 us, giua 1100 us = bang camera_tilt).
CAM_GRIP_DEG = -36.0
CAM_TOL_DEG = 0.5               # /camera/tilt lech goc mong muon duoi muc nay = da toi
CAM_SETTLE_TIMEOUT_S = 5.0      # camera khong toi goc (FC tu choi servo) -> mot lan that bai
# ENROUTE -> MARKER_SEARCH khi cach ngang <= he so nay x do cao (user dong y 10-22): camera thang
# xuong (nua FOV doc ~31 do) chi con thay tag to khi drone cach tag duoi ~0,5 h.
CAM_DOWN_DIST_PER_M = 0.5
# ENROUTE: mui huong theo duong bay khi con xa hon muc nay (gan thi giu mui, tranh quay vong).
HEADING_HOLD_NEAR_M = 1.0
# ALIGN_HEADING: treo tren tam bai bam tag to, quay mui = huong bai; lech duoi muc nay du lau thi ha.
ALIGN_HEADING_DONE_DEG = 5.0
ALIGN_HEADING_HOLD_S = 1.0
ALIGN_HEADING_TIMEOUT_S = 20.0  # qua han (gio xoay) -> ha voi mui dang co
# Gap / tha hang khi TREO: co cau gap o tam drone, ToF (10 cm sau tam) chieu xuong dat chu khong
# vao hop. User 10-22: gap khi nap hop (12 cm) NGANG CHAN: chan 0,155 - mat ToF 0,009 + hop 0,12 =
# 0,266 m (truoc: 0,20). Tha cung muc nay: day hop vua cham dat.
GRIP_HOVER_M = 0.27
GRIP_HOLD_KP = 1.0              # giu do cao: vz = kp x (dich - ToF), kep +-LAND_DESCENT_NEAR_MPS
# Doan cuoi toi do cao gap/tha ha cham: mo phong ToF theo tia 10-22, xuong 0,10 m/s thi FC tre lam lo
# xuong 0,15 m (drone ngoi len chan).
GRIP_SLOW_BELOW_M = 0.33
GRIP_FINAL_MPS = 0.05
GRIP_STOP_LEAD_M = 0.015        # dung ha som chung nay: FC tre con xuong them ~2 cm (mo phong 10-22)
# Chi kep khi EKF bao da vao tam hop: luc doi tu ha sang treo, luc day doi + trong tam lech day ngang
# ~3 cm roi moi ve (mo phong 10-22). Cho toi da GRIP_CENTER_WAIT_S sau thoi gian on dinh roi kep luon -
# tong (on dinh 2 + cho 2 + servo 0,5 s) phai < safety.yaml grip_confirm_timeout_s (5 s).
GRIP_CENTER_TOL_M = 0.015
GRIP_CENTER_WAIT_S = 2.0
# Tam hop chi lay tu cac lan do khi tag hop con TRON khung (ToF tren nguong doi camera + bien nay, hoac
# da o goc CAM_GRIP_DEG); lay trung vi BOX_OBS_N lan gan nhat. Lan do sat mep khung lech toi 5 cm (mo
# phong 10-22).
BOX_OBS_MARGIN_M = 0.05
BOX_OBS_N = 15
# FINAL_APPROACH (lay hang): ha theo tag to toi muc nay, lui ve vi tri hop khai bao, tim tag tren hop;
# khong thay trong BOX_SEARCH_TIMEOUT_S la mot lan that bai (user 10-22). 0,6 -> 0,80 (user 10-22,
# hinh hoc URDF Version2.0): dung tren hop, tag to phia mui con TRON khung (bien 6 do cho than nghieng
# khi phanh), tag hop ~50 px. O 0,6 m tag to da bi cat mep -> mat neo EKF dung luc tim hop.
BOX_SEARCH_ALT_M = 0.80
BOX_SEARCH_TIMEOUT_S = 10.0     # tinh tu luc thoi ha (toi BOX_SEARCH_ALT_M / thay hop)
# Bai co huong (user 10-25): thay tag to la vao THANG diem hop o do cao tim tag (ALIGN_HEADING), roi ha
# tu tu NGAY TAI DO toi khi thay tag hop (toi da xuong BOX_SEARCH_ALT_M). Chi ha khi EKF lech diem hop
# duoi muc nay - lech thi dung, keo ngang ve.
BOX_SEARCH_XY_TOL_M = 0.05
# Duoi ToF nay tag hop ra khoi khung o CAM_BOX_DEG: dung lai, nghieng camera CAM_GRIP_DEG roi bam tag
# tiep toi GRIP_HOVER_M (user 10-22, thay ha mu). Khung Version2.0: tag cach ong kinh ~11 cm theo phuong
# dung - bien ~5 do o -20, ~2 do o -36. Mac dinh cua Params.box_cam_switch_m.
BOX_CAM_SWITCH_M = 0.32
# Tha hang (user 10-25): KHONG ha canh, va KHONG ha tren tam tag to: hop treo duoi tam drone se che
# tag ngay duoi camera, cang xuong cang che (mat neo EKF dung luc can chinh xac nhat). Can tam + quay
# mui theo bai o do cao tim tag (2 m, hop che rat it), dich ngang ngay o do cao do ve diem hop (tam bai
# DO tu tag + box_forward_m theo huong bai = dung cho lay hop o bai khac) roi ha thang dung toi day hop
# cach dat ~5 cm moi mo kep. Camera nghieng ve mui: tag to o truoc tam 0,30 m, hop o SAU ong kinh -> o
# DROP_HOVER_M tag nam 16..50 do truoc, trong khung -14..54 do cua camera +20 do, khong bi hop che.
DROP_HOVER_M = GRIP_HOVER_M + 0.05
CAM_DROP_DEG = 20.0
DROP_ALIGN_TOL_M = 0.03         # chi ha khi EKF lech diem tha duoi muc nay
DROP_MOVE_TIMEOUT_S = 40.0      # qua han chua xuong toi (tu 2 m ~15 s) -> mot lan that bai
# Dau chang (10-11, yeu cau user): giu nguyen x, y (leo thang dung toi alt_m) va quay mui tai cho ve
# phia diem den, lech duoi muc nay moi bay ngang. Quay mui trong luc bay ngang lam flow troi them
# va camera chua nhin ve phia bai.
ENROUTE_TURN_DONE_DEG = 15.0

# Ke hoach nhiem vu. Hanh dong tai diem trung MissionWaypoint.ACTION_*.
ACTION_NONE, ACTION_PICKUP, ACTION_DROPOFF = 0, 1, 2
MAX_WAYPOINT_VEL_MPS = 1.9  # 95 % tran ngang FC (giao uoc 9.6, P9)


# Goc camera khi VAO trang thai (trang thai khac giu goc dang co). FINAL_APPROACH tu doi sang
# CAM_BOX_DEG khi thay tag hop.
CAM_FOR_STATE = {IDLE: CAM_DOWN_DEG, TAKEOFF: CAM_DOWN_DEG, ENROUTE: CAM_ENROUTE_DEG,
                 MARKER_SEARCH: CAM_DOWN_DEG, ALIGN_HEADING: CAM_DOWN_DEG,
                 PRECISION_LAND: CAM_DOWN_DEG, FINAL_APPROACH: CAM_DOWN_DEG,
                 RETRY_LOITER: CAM_DOWN_DEG, EMERGENCY_LAND: CAM_DOWN_DEG}


def wrap(a):
    """Goc ve (-pi, pi]."""
    return math.atan2(math.sin(a), math.cos(a))


def descent_mps(range_m):
    """Toc do ha (duong) theo do cao laser: xa nhanh, gan cham, sat dat nhanh; khong laser cham."""
    if range_m is None:
        return LAND_DESCENT_NEAR_MPS
    if range_m > LAND_NEAR_BELOW_M:
        return LAND_DESCENT_FAR_MPS
    if range_m <= LAND_FINAL_BELOW_M:
        return LAND_FINAL_MPS
    return LAND_DESCENT_NEAR_MPS


@dataclass
class Waypoint:
    """Mot diem da kiem tra. Khong co GPS nen vi tri SUY TU TAG: target = toa do tag
    expected_marker_id trong known_tags (khung odom, ENU) nang them alt_m theo z.
    lat/lon va pos_ned cua MissionWaypoint bi bo qua."""

    marker_id: int
    action: int
    alt_m: float
    acceptance_radius_m: float
    max_vel_mps: float
    loiter_s: float
    target: tuple
    # Huong "TREN" cua bai (yaw ENU, rad) - None = bai khong khai huong (khong co tag nho).
    pad_yaw: float = None
    pad: tuple = None           # tam bai (x, y, z) trong odom


def parse_known_tags(flat):
    """[id, x, y, z, ...] -> {id: (x, y, z)}. Cung dinh dang config/tags.yaml."""
    if len(flat) % 4:
        raise ValueError(f'known_tags phai co boi so cua 4 phan tu, dang co {len(flat)}')
    return {int(flat[i]): tuple(flat[i + 1:i + 4]) for i in range(0, len(flat), 4)}


def build_waypoints(raw, known_tags, pad_yaws=None):
    """raw: list dict (seq, marker_id, action, alt_m, acceptance_radius_m, max_vel_mps, loiter_s).
    pad_yaws: {id: yaw ENU rad} cua bai co huong.
    Tra (list Waypoint, '') hoac (None, ly do tu choi). Sai mot diem la tu choi ca ke hoach."""
    pad_yaws = pad_yaws or {}
    if not raw:
        return None, 'ke hoach khong co waypoint'
    out = []
    for i, w in enumerate(raw):
        where = f'waypoint {i}'
        if w['seq'] != i:
            return None, f'{where}: seq = {w["seq"]}, phai = {i} (dung thu tu)'
        if w['marker_id'] not in known_tags:
            return None, (f'{where}: expected_marker_id {w["marker_id"]} khong co trong known_tags '
                          f'{sorted(known_tags)} - khong co GPS, vi tri diem suy tu tag')
        if w['action'] not in (ACTION_NONE, ACTION_PICKUP, ACTION_DROPOFF):
            return None, f'{where}: action {w["action"]} khong hop le'
        nums = ('alt_m', 'acceptance_radius_m', 'max_vel_mps', 'loiter_s')
        if not all(math.isfinite(w[k]) for k in nums):
            return None, f'{where}: co gia tri nan/inf'
        if w['alt_m'] <= 0.0:
            return None, f'{where}: alt_m = {w["alt_m"]} phai > 0'
        if w['acceptance_radius_m'] <= 0.0:
            return None, f'{where}: acceptance_radius_m phai > 0'
        if not 0.0 < w['max_vel_mps'] <= MAX_WAYPOINT_VEL_MPS:
            return None, f'{where}: max_vel_mps phai trong (0, {MAX_WAYPOINT_VEL_MPS}]'
        if w['loiter_s'] < 0.0:
            return None, f'{where}: loiter_s phai >= 0'
        tx, ty, tz = known_tags[w['marker_id']]
        out.append(Waypoint(w['marker_id'], w['action'], w['alt_m'], w['acceptance_radius_m'],
                            w['max_vel_mps'], w['loiter_s'], (tx, ty, tz + w['alt_m']),
                            pad_yaws.get(w['marker_id']), (tx, ty, tz)))
    return out, ''


@dataclass
class Snapshot:
    """Anh chup toan bo dau vao mot chu ky - de FSM la ham thuan, khong tu doc topic."""

    now_s: float = 0.0
    armed: bool = False
    battery_pct: float = 100.0
    # Vi tri EKF (x, y, z) trong odom; None = chua co / qua han.
    position: tuple = None
    yaw: float = None            # yaw EKF (ENU, rad); None = chua co / qua han
    # Tu the bai DO tu tag cua diem hien tai (x, y, z, yaw ENU) trong odom - da loc; None = chua
    # thay tag bai nay. Co thi DUNG thay vi tri/huong khai bao: bai dat lech hay quay bat ky huong
    # nao van vao dung truc bai (10-09, yaw la ban gia dinh dung).
    pad_obs: tuple = None
    # odom da NEO theo tags.yaml chua (EkfHealth.anchored). Truoc khi neo, position thuoc mot
    # khung khac va se NHAY khi neo - nen khong duoc chot "nha" bang no.
    pos_anchored: bool = False
    landing_target_lost: bool = True
    landed: bool = False
    gripper_sensor_confirmed: bool = False
    failsafe_escalate_to: int = 0   # muc NANG NHAT trong cac su co dang bat
    fc_connected: bool = False
    # Kenh NAMED_VALUE_INT cua FC; None = chua nhan / qua han = KHONG BIET (6.3 dong cuoi).
    ob_auth: bool = None
    ob_state: int = None
    arm_ready: bool = False
    disarm_ready: bool = False
    range_m: float = None        # laser, None khi ngoai dai / qua han
    # Do lech ngang (m) cua tag mong doi so voi base_link; None = khong co pose tag con moi.
    # Chi landing_target_bridge_node phat pose nay, va chi khi thay DUNG ID mong doi - day la
    # xac thuc marker duy nhat FSM dung (nguyen tac 1).
    target_offset_m: float = None
    # Goc camera da on dinh (do, + ve mui) tu camera_tilt_node; None = servo dang quay / khong biet.
    camera_tilt_deg: float = CAM_DOWN_DEG
    # Tam hop (x, y) trong odom DO tu tag hop (mission_manager_node); None = khong thay.
    box_obs: tuple = None
    # Tag to cua bai dang toi vua duoc do (pad_odom con moi) - ALIGN_HEADING khi gap/tha khong bam
    # tam tag (dung tren diem hop) nen khong co target_offset_m, dung co nay de biet con thay tag.
    pad_seen: bool = False


@dataclass
class Params:
    search_timeout_s: float = 20.0
    max_retries: int = 3
    acceptance_radius_m: float = 1.5
    pre_dropoff_settle_s: float = 2.0
    takeoff_alt_m: float = 5.0
    anchor_margin_m: float = 0.5      # leo them sau khi neo truoc khi roi TAKEOFF, du bien an toan
    anchor_wait_s: float = 10.0       # cho toi da o tran takeoff_alt_m ma chua neo thi ha khan cap
    # Pi dieu khien huong mui (WP5): phat yaw mong muon, bat ALIGN_HEADING cho bai co huong. Tat thi
    # mui giu nguyen va bai nao cung ha thang dung, khong quay theo bai.
    yaw_control: bool = False
    # Tag nho tren hop hang (tags.yaml pad_small_tag_*): id = id_to + offset, tam hop cach tam tag to
    # box_forward_m theo huong bai (am = ve phia "duoi").
    small_id_offset: int = 10
    box_forward_m: float = -0.30
    box_cam_switch_m: float = BOX_CAM_SWITCH_M


@dataclass
class Action:
    """Viec FSM muon node ROS thuc hien sau buoc nay (node dich ra service/topic tuong ung)."""

    fc_command: str = ''       # '', 'arm', 'disarm' (thuong, KHONG BAO GIO 21196)
    gripper_command: str = ''  # '', 'open', 'close'
    expected_marker_id: int = -1
    velocity_up_mps: float = None   # lenh van toc len (FLU z); None = khong lenh van toc
    # Diem den (x, y, z) trong odom cho vong vi tri cruise; None = khong phat /mission/setpoint.
    # velocity_up_mps (neu co) uu tien hon trong position_controller_node.
    position_target: tuple = None
    # Tran toc do NGANG (m/s) cua waypoint dang bay toi; None = khong gioi han rieng, chi con
    # tran cua FC. Di kem position_target, khong co y nghia khi position_target = None.
    max_vel_mps: float = None
    # Yaw mong muon (ENU, rad) cho vong yaw cua position_controller_node; None = khong quay mui.
    yaw_target: float = None
    # Goc camera mong muon (do, + ve mui) cho camera_tilt_node; None = khong doi.
    camera_tilt_deg: float = None
    # Dang o pha hop hang (range_vertical_node bu laser khi vet sang tren nap hop).
    box_mode: bool = False
    detail: str = ''


@dataclass
class MissionFsm:
    params: Params = field(default_factory=Params)
    state: str = IDLE
    waypoints: list = field(default_factory=list)
    current_wp_index: int = 0
    retry_count: int = 0
    state_entered_s: float = 0.0
    last_fc_command_s: float = None
    start_requested_s: float = None   # thoi diem nhan yeu cau cat canh; None = khong co
    land_requested: bool = False
    rth_requested: bool = False       # GCS yeu cau ve nha (MAV_CMD 20)
    abort_requested: bool = False     # GCS huy nhiem vu (MAV_CMD 42100)
    # Ket qua chuyen bay gan nhat, CHOT lai qua ca luc ve IDLE cho toi khi cat canh chuyen moi.
    mission_result: int = RESULT_UNKNOWN
    mission_id: int = 0
    max_retries: int = None           # cua ke hoach dang nap; None = dung params
    # Vi tri (x, y, z) trong odom luc bat dau cat canh = "nha". Khong co GPS nen day chinh la
    # moc neo cua khung odom. None = chua tung cat canh -> khong RTH duoc.
    home: tuple = None
    rth_alt_m: float = None           # do cao giu khi bay ve, chot luc vao RTH
    # Do cao dich sau khi neo = min(do cao luc neo + anchor_margin_m, takeoff_alt_m); None = chua
    # neo trong chang TAKEOFF nay. Dat lai moi lan vao TAKEOFF (xem transition()).
    anchor_climb_target_m: float = None
    # Thoi diem bat dau lo lung o tran takeoff_alt_m ma chua neo; None = chua cham tran/da neo.
    ceiling_wait_since_s: float = None
    # Rieng cho nhip phat lai lenh gap/tha. KHONG dung chung last_fc_command_s: truong do
    # danh cho nhip thu lai DISARM trong _descend_and_disarm.
    last_gripper_command_s: float = None
    search_timeout_s: float = None
    last_snap: Snapshot = None
    cam_cmd: float = CAM_DOWN_DEG   # goc camera dang ra lenh (Action.camera_tilt_deg moi chu ky)
    heading_ok_since: float = None  # ALIGN_HEADING: luc mui bat dau nam trong ALIGN_HEADING_DONE_DEG
    box_phase: str = None           # FINAL_APPROACH: 'search' | 'tilt' | 'descend'
    box_phase_s: float = None
    # Vua gap hang o bai nay: TAKEOFF phai thay lai tag to cua no (neo lai EKF) moi bay tiep.
    reanchor_id: int = None
    box_xy: tuple = None            # tam hop DO duoc (trung vi) - FINAL_APPROACH -> ACTUATE_GRIPPER
    box_obs_list: list = field(default_factory=list)
    box_takeoff: bool = False       # TAKEOFF ngay sau khi gap: vet ToF con quet qua nap hop
    # Quay mui tai cho dau chang ENROUTE: diem x, y giu trong luc quay; da quay xong chua.
    enroute_hold: tuple = None
    enroute_turned: bool = False
    tag_lost_since: float = None    # PRECISION_LAND: thoi diem bat dau mat tag (None = dang thay)
    drop_ready_s: float = None      # ACTUATE_GRIPPER tha: luc xuong toi DROP_HOVER_M (None = chua)
    drop_hold_m: float = None       # ACTUATE_GRIPPER tha: ToF giu trong luc dich ve diem hop

    def load_plan(self, mission_id, raw_waypoints, max_retries, search_timeout_s, known_tags,
                  pad_yaws=None):
        """Nap ke hoach. Tra ly do tu choi, '' neu nhan. KHONG tu cat canh - van phai request_start.

        max_retries / search_timeout_s <= 0 nghia la dung gia tri trong params.
        """
        if self.state != IDLE:
            return f'dang {self.state}, chi nhan ke hoach khi IDLE'
        if self.start_requested_s is not None:
            return 'dang cho cat canh theo ke hoach cu - khong doi ke hoach luc nay'
        waypoints, err = build_waypoints(raw_waypoints, known_tags, pad_yaws)
        if err:
            return err
        self.mission_id = mission_id
        self.waypoints = waypoints
        self.current_wp_index = 0
        self.retry_count = 0
        self.max_retries = max_retries if max_retries > 0 else self.params.max_retries
        self.search_timeout_s = (search_timeout_s if search_timeout_s > 0
                                 else self.params.search_timeout_s)
        return ''

    def current_waypoint(self):
        if 0 <= self.current_wp_index < len(self.waypoints):
            return self.waypoints[self.current_wp_index]
        return None

    def clear_plan(self):
        """Xoa ke hoach dang nap (chua cat canh) khi ban do tag doi (giao uoc GCS 8.7 quy tac 3):
        ke hoach cu suy vi tri tu ban do cu, giu lai la giu mot ke hoach co nghia khac luc soan no.

        Cung dieu kien voi load_plan - chi tac dung khi IDLE va khong dang cho cat canh. Tra ly do
        tu choi, '' neu xoa duoc (ke ca khi khong co ke hoach nao dang nap - vo hai).
        """
        if self.state != IDLE:
            return f'dang {self.state}, chi xoa duoc khi IDLE'
        if self.start_requested_s is not None:
            return 'dang cho cat canh theo ke hoach cu - khong xoa luc nay'
        self.mission_id = 0
        self.waypoints = []
        self.current_wp_index = 0
        self.retry_count = 0
        return ''

    def request_start(self, now_s):
        """Yeu cau cat canh (GCS / thu tren ban). Tra ly do tu choi, '' neu nhan."""
        if self.state != IDLE:
            return f'dang {self.state}, chi nhan khi IDLE'
        self.start_requested_s = now_s
        return ''

    def request_land(self):
        self.land_requested = True

    def request_rth(self):
        """GCS yeu cau ve nha. Tra ly do tu choi, '' neu nhan.

        Tu choi SOM khi chua biet nha, thay vi nhan roi im lang ha canh tai cho: nguoi van hanh
        bam "ve nha" ma drone ha xuong cho la la kieu bat ngo te nhat. Nha chi co khi da cat canh
        VA odom da neo theo bang tag (giao uoc GCS 5.2b y 4).
        """
        if self.state == IDLE:
            return 'dang IDLE, khong co gi de ve'
        if self.home is None:
            return 'chua biet nha (odom chua neo luc cat canh) - dung ~/land de ha tai cho'
        self.rth_requested = True
        return ''

    def request_abort(self):
        """GCS huy nhiem vu: bo ke hoach, ha canh neu dang bay, ve IDLE.

        KHONG tu choi bao gio - huy phai luon di duoc, ke ca khi FSM dang o trang thai la.
        """
        self.abort_requested = True

    def _ghi_ket_qua(self, ma):
        """Ghi ly do chuyen bay ket thuc. LY DO DAU TIEN THANG.

        Vi du: dang ha canh vi failsafe thi GCS bam huy - ket qua phai van la FAILSAFE, vi do
        moi la thu khien chuyen bay dung. Ghi de se giau mat su co.
        """
        if self.mission_result == RESULT_UNKNOWN:
            self.mission_result = ma

    def transition(self, new_state, now_s, detail=''):
        """Chuyen trang thai co kiem tra bang TRANSITIONS - chan chuyen sai tu som."""
        if new_state not in TRANSITIONS[self.state]:
            raise ValueError(f'Chuyen trang thai khong hop le: {self.state} -> {new_state}')
        if self.state == IDLE and new_state == TAKEOFF:
            # Chuyen bay moi thuc su bat dau: xoa ket qua chuyen truoc. Xoa o day chu KHONG o
            # request_start() vi yeu cau cat canh co the bi huy truoc khi roi dat, va khi do
            # ket qua chuyen truoc van con gia tri voi GCS.
            self.mission_result = RESULT_UNKNOWN
        if new_state == TAKEOFF:
            # Chang TAKEOFF moi (tu IDLE hoac tu ACTUATE_GRIPPER giua chang) - dat lai muc tieu
            # leo them sau neo va bo dem cho tran, du da tung neo tu chang truoc (pos_anchored la
            # latch, khong tu False lai - snap.pos_anchored van True ngay tu buoc dau chang nay).
            self.anchor_climb_target_m = None
            self.ceiling_wait_since_s = None
        if new_state in (PRECISION_LAND, ALIGN_HEADING):
            self.tag_lost_since = now_s     # tinh tu luc vao cho toi khi thay tag
        if new_state == ALIGN_HEADING:
            self.heading_ok_since = None
        if new_state == FINAL_APPROACH:
            self.box_phase, self.box_phase_s = 'search', now_s
            self.box_xy = None
            self.box_obs_list = []
        if new_state in (ENROUTE, IDLE):
            self.box_takeoff = False
            self.tag_lost_since = None
        self.drop_ready_s = self.drop_hold_m = None
        self.cam_cmd = CAM_FOR_STATE.get(new_state, self.cam_cmd)
        if new_state == ENROUTE:
            self.enroute_hold = None
            self.enroute_turned = False
        if new_state == MISSION_COMPLETE:
            # Chua ly do nao khac duoc ghi = khong co gi cat ngang = lam het ke hoach.
            self._ghi_ket_qua(RESULT_COMPLETED)
        self.state = new_state
        self.state_entered_s = now_s
        # retry_count tinh theo TUNG DIEM: chi xoa khi bat dau chang moi (hoac ve IDLE), de vong
        # MARKER_SEARCH <-> RETRY_LOITER / PRECISION_LAND khong bao gio lap vo han.
        if new_state in (IDLE, ENROUTE):
            self.retry_count = 0
        if new_state == IDLE:
            self.reanchor_id = None
        self.last_fc_command_s = None
        self.last_gripper_command_s = None
        if new_state != IDLE:
            self.start_requested_s = None
        if new_state in (EMERGENCY_LAND, IDLE):
            self.land_requested = False
            self.rth_requested = False
        if new_state == IDLE:
            self.abort_requested = False
        return Action(detail=detail)

    def _box_mode(self):
        """Pha ma vet ToF co the nam tren nap hop: tim/ha toi hop, gap, bay len sau gap (cho toi khi
        thay lai tag to)."""
        wp = self.current_waypoint()
        if self.state == FINAL_APPROACH:
            return True
        if self.state == ACTUATE_GRIPPER:
            return wp is not None and wp.action == ACTION_PICKUP
        return self.state == TAKEOFF and self.box_takeoff

    def _cam_at(self, snap, deg):
        """Camera da on dinh o goc deg (khong dang quay)."""
        return snap.camera_tilt_deg is not None and abs(snap.camera_tilt_deg - deg) <= CAM_TOL_DEG

    def time_in_state(self, now_s):
        return now_s - self.state_entered_s

    def _fc_command_pending(self, now_s, period_s):
        """True neu lenh FC vua gui chua qua period_s - chua duoc gui lai."""
        return self.last_fc_command_s is not None and now_s - self.last_fc_command_s < period_s

    def step(self, snap):
        """Chay mot chu ky FSM (_step), kem goc camera dang ra lenh vao Action."""
        act = self._step(snap)
        act.camera_tilt_deg = self.cam_cmd
        act.box_mode = self._box_mode()
        return act

    def _step(self, snap):
        """Chay mot chu ky FSM, tra ve Action.

        Da hien thuc lop FC (giao uoc 6.2, 6.3, P2/P10):
          - dang arm ma mat quyen (OB_AUTH = 0) hoac KHOA -> FAILSAFE, KHONG gui lenh nao nua
            (ke ca DISARM); nguoi lai cat ch5 hoac gat ch8 xuong-len. OB_AUTH chua biet -> giu
            nguyen, khong gui lenh FC moi;
          - FAILSAFE: da disarm -> IDLE; con arm va lay lai quyen -> EMERGENCY_LAND;
          - IDLE: request_start() -> gui ARM chi khi OB_ARM_RDY = 1, thua 2 s; armed -> TAKEOFF;
          - TAKEOFF: leo toi khi neo (EkfHealth.anchored) roi leo them anchor_margin_m (khong vuot
            takeoff_alt_m) roi giu vz = 0; cham takeoff_alt_m ma chua neo -> lo lung cho toi
            anchor_wait_s roi EMERGENCY_LAND;
          - EMERGENCY_LAND: xuong; landed + OB_DIS_RDY = 1 -> DISARM thuong, thu lai 3 s;
            disarm xong -> MISSION_COMPLETE -> IDLE. KHONG tu goi 21196 (11.1 #12f).
          - request_land() -> EMERGENCY_LAND tu moi trang thai co nhanh do trong TRANSITIONS.

        Failsafe (/failsafe_event, muc nang nhat dang bat) - uu tien ngay sau kiem quyen:
          - RTH / EMERGENCY_LAND -> EMERGENCY_LAND (RTH tam thoi = ha canh tai cho);
          - LOITER -> FAILSAFE giu vz = 0; het su co -> ha canh (khong tu tiep tuc nhiem vu);
          - RETRY_LOITER -> chi MARKER_SEARCH dung (coi nhu het gio tim); trang thai khac bo qua;
          - dang bat bat ky muc nao -> IDLE khong arm.

        Trang thai nhiem vu (khong GPS, vi tri diem suy tu tag):
          - TAKEOFF: da neo VA laser >= anchor_climb_target_m -> ENROUTE neu da nap ke hoach,
            khong thi giu; chua neo khi cham takeoff_alt_m -> cho anchor_wait_s roi EMERGENCY_LAND;
          - ENROUTE: camera 20 do ve mui, bay toi tam bai (do tu tag neu da thay); cach ngang
            <= min(acceptance_radius_m, 0,5 x do cao) -> MARKER_SEARCH (camera thang xuong);
          - MARKER_SEARCH: giu tai diem, phat expected_marker_id; camera da xuong 0 do va
            landing_target_bridge_node xac thuc dung ID -> ALIGN_HEADING (bai co huong +
            yaw_control) / PRECISION_LAND; qua search_timeout_s (hoac failsafe RETRY_LOITER) ->
            mot lan that bai;
          - ALIGN_HEADING: treo bam tag to, quay mui = huong bai -> PRECISION_LAND;
          - PRECISION_LAND: ha thang dung bam tag to; mat tag tren PRECISION_BLIND_BELOW_M ->
            MARKER_SEARCH, cung la mot lan that bai (khong ha mu); DROPOFF can tam xong (khong ha
            tren tag) -> ACTUATE_GRIPPER (ve diem hop, ha, treo); PICKUP toi BOX_SEARCH_ALT_M ->
            FINAL_APPROACH; landed (diem khong hanh dong) -> ACTUATE_GRIPPER;
          - FINAL_APPROACH: tim tag hop, camera -20 do, ha bam tag hop; ToF box_cam_switch_m dung,
            camera -36 do, bam tiep toi GRIP_HOVER_M -> ACTUATE_GRIPPER; khong thay / mat tag hop
            -> mot lan that bai;
          - gap xong: TAKEOFF chi roi khi da thay lai tag to (neo lai EKF);
          - moi lan that bai retry_count += 1; qua max_retries -> EMERGENCY_LAND tai cho (RTH
            tam thoi); con luot -> RETRY_LOITER giu tai diem RETRY_LOITER_S roi tim lai.

        """
        now = snap.now_s
        self.last_snap = snap
        lost = snap.ob_auth is False or snap.ob_state == OB_STATE_KHOA
        if snap.armed and lost and self.state != FAILSAFE and FAILSAFE in TRANSITIONS[self.state]:
            self._ghi_ket_qua(RESULT_FAILSAFE)
            return self.transition(FAILSAFE, now, 'Pi mat quyen/KHOA khi dang arm - '
                                   'nguoi lai cat ch5 hoac ch8 xuong-len')

        esc = snap.failsafe_escalate_to
        if snap.armed and self.state not in (FAILSAFE, EMERGENCY_LAND):
            # Muc 4 va muc 3 KHONG con gop chung: muc 4 (pin kiet, mat FC) thi moi giay bay them
            # deu la rui ro -> ha ngay. Muc 3 (pin yeu, mat GCS) con du bien de ve nha.
            if esc >= ESCALATE_EMERGENCY_LAND:
                self._ghi_ket_qua(RESULT_FAILSAFE)
                target = EMERGENCY_LAND if EMERGENCY_LAND in TRANSITIONS[self.state] else FAILSAFE
                return self.transition(target, now, f'failsafe muc {esc} - ha canh tai cho')
            if esc == ESCALATE_RTH and self.state != RTH:
                self._ghi_ket_qua(RESULT_FAILSAFE)
                return self._vao_rth(snap, now, f'failsafe muc {esc}')
            if esc == ESCALATE_LOITER:
                self._ghi_ket_qua(RESULT_FAILSAFE)
                return self.transition(FAILSAFE, now, 'failsafe LOITER - giu vi tri')

        if self.state == FAILSAFE:
            if not snap.armed:
                return self.transition(IDLE, now, 'da disarm')
            if snap.ob_auth is not True or snap.ob_state == OB_STATE_KHOA:
                return Action(detail='mat quyen - cho nguoi lai')
            if esc >= ESCALATE_EMERGENCY_LAND:
                return self.transition(EMERGENCY_LAND, now, f'failsafe muc {esc} - ha canh tai cho')
            if esc == ESCALATE_RTH:
                return self._vao_rth(snap, now, f'failsafe muc {esc}')
            if esc == ESCALATE_LOITER and not self.land_requested:
                return Action(velocity_up_mps=0.0,
                              detail='failsafe LOITER - giu vi tri, cho het su co')
            # Het su co hoac lay lai quyen: KHONG tu tiep tuc nhiem vu - ha canh.
            return self.transition(EMERGENCY_LAND, now, 'het su co / lay lai quyen - ha canh')

        # Huy nhiem vu manh hon ha canh: bo ke hoach TRUOC roi moi ha, de khi cham dat FSM
        # khong tu di tiep diem nao. Dat truoc land_requested vi abort bao gom ca ha canh.
        if self.abort_requested:
            self._ghi_ket_qua(RESULT_ABORTED)
            self.waypoints = []
            self.current_wp_index = 0
            self.start_requested_s = None
            if not snap.armed:
                if self.state == IDLE:
                    self.abort_requested = False
                    return Action(detail='huy nhiem vu - da bo ke hoach')
                if self.state == MISSION_COMPLETE:
                    # Duong ra duy nhat cua MISSION_COMPLETE la IDLE - khong co nhanh FAILSAFE.
                    # Thieu nhanh nay thi abort toi trong lue giu 1,5 s se lam nhanh duoi tra ve
                    # Action moi tick, khong bao gio toi duoc _step, va FSM KET LAI O DAY VINH VIEN.
                    return self.transition(IDLE, now, 'huy nhiem vu - chuyen bay da ket thuc')
                # Da disarm ma con o trang thai bay la BAT THUONG, va FAILSAFE la duong danh cho
                # bat thuong: no tu ve IDLE ngay chu ky sau khi thay khong armed. IDLE khong nam
                # trong bang chuyen cua ENROUTE/TAKEOFF nen khong di thang duoc.
                if FAILSAFE in TRANSITIONS[self.state]:
                    return self.transition(FAILSAFE, now, 'huy nhiem vu khi da disarm')
                return Action(detail='huy nhiem vu - da bo ke hoach')
            if self.state != EMERGENCY_LAND and EMERGENCY_LAND in TRANSITIONS[self.state]:
                # Xoa co NGAY khi da vao duong ha canh: ke hoach da bo nen khong con gi de huy,
                # va giu co lai se chan duong EMERGENCY_LAND -> MISSION_COMPLETE binh thuong,
                # bat GCS hien FAILSAFE cho mot lenh huy hoan thanh dung.
                self.abort_requested = False
                return self.transition(EMERGENCY_LAND, now, 'huy nhiem vu - ha canh')

        if (self.rth_requested and snap.armed and self.state not in (RTH, EMERGENCY_LAND)):
            self._ghi_ket_qua(RESULT_RTH)
            return self._vao_rth(snap, now, 'GCS yeu cau ve nha')

        if (self.land_requested and snap.armed and self.state != EMERGENCY_LAND
                and EMERGENCY_LAND in TRANSITIONS[self.state]):
            self._ghi_ket_qua(RESULT_LANDED_CMD)
            return self.transition(EMERGENCY_LAND, now, 'yeu cau ha canh')

        if self.state == IDLE:
            return self._step_idle(snap)
        if self.state == TAKEOFF:
            return self._step_takeoff(snap)
        if (self.state in (ENROUTE, MARKER_SEARCH, ALIGN_HEADING, RETRY_LOITER)
                and snap.ob_auth is not True):
            return Action(velocity_up_mps=0.0, detail='khong biet quyen - giu vz = 0')
        if self.state == ENROUTE:
            return self._step_enroute(snap)
        if self.state == ALIGN_HEADING:
            return self._step_align_heading(snap)
        if self.state == FINAL_APPROACH:
            return self._step_final_approach(snap)
        if self.state == MARKER_SEARCH:
            return self._step_marker_search(snap)
        if self.state == RETRY_LOITER:
            return self._step_retry_loiter(snap)
        if self.state == RTH:
            return self._step_rth(snap)
        if self.state == EMERGENCY_LAND:
            return self._step_land(snap)
        if self.state == PRECISION_LAND:
            return self._step_precision_land(snap)
        if self.state == ACTUATE_GRIPPER:
            return self._step_actuate_gripper(snap)
        if self.state == MISSION_COMPLETE:
            if self.time_in_state(now) < MISSION_COMPLETE_HOLD_S:
                # Khong phat lenh nao: da disarm, nam dat. Chi de telemetry 2 Hz kip lay mau.
                return Action(detail='giu MISSION_COMPLETE cho GCS doc duoc')
            return self.transition(IDLE, now, 'xong')
        return Action(detail=f'{self.state}: chua hien thuc')

    def _step_idle(self, snap):
        now = snap.now_s
        self.land_requested = False
        if self.start_requested_s is None:
            return Action()
        if snap.failsafe_escalate_to != ESCALATE_NONE and not snap.armed:
            # HUY chu khong treo yeu cau: neu giu, het su co la tu ARM tu yeu cau cu.
            self.start_requested_s = None
            return Action(detail=f'huy yeu cau cat canh: failsafe dang bat '
                                 f'(muc {snap.failsafe_escalate_to})')
        if snap.armed:
            return self.transition(TAKEOFF, now, 'da arm')
        if now - self.start_requested_s > ARM_WAIT_S:
            self.start_requested_s = None
            return Action(detail=f'huy yeu cau cat canh: khong arm duoc sau {ARM_WAIT_S:.0f} s')
        if self._fc_command_pending(now, ARM_RETRY_S):
            # OB_* toi truoc /mavros/state: OB_ARM_RDY da ve 0 ma armed chua len.
            return Action(detail='da gui ARM, cho FC xac nhan')
        if not snap.fc_connected or snap.ob_auth is not True:
            return Action(detail='cho quyen: nguoi lai gat ch8 xuong-len')
        if not snap.arm_ready:
            return Action(detail='cho OB_ARM_RDY = 1 (ch5 len, ch6 POSHOLD, ga giua)')
        self.last_fc_command_s = now
        return Action(fc_command='arm', detail='gui ARM')

    def _vao_rth(self, snap, now, ly_do):
        """Vao RTH neu bay ve duoc; thieu dieu kien thi ha canh tai cho (an toan hon la treo)."""
        if RTH not in TRANSITIONS[self.state] or self.home is None or snap.position is None:
            target = EMERGENCY_LAND if EMERGENCY_LAND in TRANSITIONS[self.state] else FAILSAFE
            return self.transition(target, now, f'{ly_do} - khong RTH duoc, ha canh tai cho')
        self.rth_alt_m = snap.position[2]
        return self.transition(RTH, now, f'{ly_do} - bay ve nha')

    def _step_rth(self, snap):
        """Bay ve nha o do cao dang co roi ha canh. Khong ha chinh xac theo tag: RTH la duong
        thoat hiem, khong phai pha nghiep vu - tag o nha co the khong thay duoc."""
        if snap.position is None or self.home is None:
            return self.transition(EMERGENCY_LAND, snap.now_s,
                                   'RTH: khong co vi tri / chua biet nha - ha canh tai cho')
        if self.time_in_state(snap.now_s) >= RTH_TIMEOUT_S:
            return self.transition(EMERGENCY_LAND, snap.now_s,
                                   f'RTH qua {RTH_TIMEOUT_S:.0f} s chua ve toi - ha canh tai cho')
        dist = math.hypot(snap.position[0] - self.home[0], snap.position[1] - self.home[1])
        if dist <= RTH_ACCEPT_M:
            return self.transition(EMERGENCY_LAND, snap.now_s,
                                   f've toi nha (cach {dist:.2f} m) - ha canh')
        target = (self.home[0], self.home[1], self.rth_alt_m)
        # Bo max_vel_mps cua waypoint, dung tran cho phep: RTH la duong thoat hiem, ve cang nhanh
        # cang it thoi gian tren khong. Gioi han cua waypoint la rang buoc nghiep vu (hang hoa),
        # khong con y nghia khi da bo nhiem vu.
        return Action(position_target=target, max_vel_mps=MAX_WAYPOINT_VEL_MPS,
                      detail=f'RTH: ve nha, con {dist:.1f} m')

    def _step_takeoff(self, snap):
        act = self._step_takeoff_core(snap)
        if self.reanchor_id is not None and self.state == TAKEOFF:
            act.expected_marker_id = self.reanchor_id      # cho bridge bao khi thay lai tag to
        return act

    def _step_takeoff_core(self, snap):
        # Chot "nha" trong luc dang leo: drone leo THANG DUNG nen x, y van la cua nha, va RTH
        # chi dung x, y (do cao ve nha lay theo luc vao RTH). Ghi o day thay vi ngay luc chuyen
        # trang thai vi luc do co the chua co vi tri EKF.
        #
        # BAT BUOC doi pos_anchored: truoc khi odom neo theo bang tag, goc odom la cho EKF khoi
        # dong. Chot home luc do thi sau khi neo con so ay tro sang MOT CHO VAT LY KHAC va RTH bay
        # sai cho - da tai hien: cat canh lech pad_home 3 m thi RTH ve pad_home chu khong ve diem
        # cat canh. Giao uoc GCS<->Pi 5.2b y 4. Drone khong thay duoc pad cua chinh no luc dau
        # (camera chech 20 do, tag duoi min_range) nen neo xay ra ~0,6 s sau khi roi dat, luc x, y
        # van la cua diem cat canh.
        if self.home is None and snap.position is not None and snap.pos_anchored:
            self.home = snap.position
        if snap.ob_auth is not True:
            return Action(velocity_up_mps=0.0, detail='khong biet quyen - giu vz = 0')
        if snap.range_m is None:
            return Action(velocity_up_mps=0.0, detail='mat laser - giu vz = 0')

        if snap.pos_anchored and self.anchor_climb_target_m is None:
            # Vua neo xong: leo them mot bien an toan (anchor_margin_m) roi moi roi TAKEOFF, thay
            # vi cho het takeoff_alt_m nhu truoc - neo som (vd camera goc rong) thi roi som theo,
            # khong vuot tran takeoff_alt_m du neo tre.
            self.anchor_climb_target_m = min(snap.range_m + self.params.anchor_margin_m,
                                             self.params.takeoff_alt_m)

        if (self.reanchor_id is not None and snap.target_offset_m is not None
                and self._cam_at(snap, CAM_DOWN_DEG)):
            self.reanchor_id = None             # thay lai tag to sau khi gap: EKF da neo lai

        if self.anchor_climb_target_m is not None:
            if snap.range_m >= self.anchor_climb_target_m and self.reanchor_id is not None:
                # User 10-22: gap bang tag nho xong phai neo lai EKF bang tag to roi moi bay tiep.
                if self.ceiling_wait_since_s is None:
                    self.ceiling_wait_since_s = snap.now_s
                elif snap.now_s - self.ceiling_wait_since_s >= self.params.anchor_wait_s:
                    return self.transition(
                        EMERGENCY_LAND, snap.now_s,
                        f'khong thay lai tag to {self.reanchor_id} sau '
                        f'{self.params.anchor_wait_s:g} s - ha khan cap')
                return Action(velocity_up_mps=0.0,
                              detail=f'cho thay lai tag to {self.reanchor_id} de neo lai EKF')
            if snap.range_m >= self.anchor_climb_target_m:
                wp = self.current_waypoint()
                if wp is None:
                    return Action(velocity_up_mps=0.0, detail='du do cao - giu (chua nap ke hoach)')
                return self.transition(ENROUTE, snap.now_s,
                                       f'du do cao sau khi neo - bay toi diem {self.current_wp_index} '
                                       f'(tag {wp.marker_id})')
            return Action(velocity_up_mps=TAKEOFF_CLIMB_MPS, detail='leo them sau khi neo')

        # Chua neo. Cham tran takeoff_alt_m ma van chua neo thi lo lung cho toi han anchor_wait_s
        # roi ha khan cap - KHONG duoc bay tiep bang vi tri EKF chua dang tin (xem module docstring
        # ve pos_anchored: truoc khi neo, vi tri thuoc mot khung khac va se NHAY khi neo xong).
        if snap.range_m >= self.params.takeoff_alt_m:
            if self.ceiling_wait_since_s is None:
                self.ceiling_wait_since_s = snap.now_s
            elif snap.now_s - self.ceiling_wait_since_s >= self.params.anchor_wait_s:
                return self.transition(
                    EMERGENCY_LAND, snap.now_s,
                    f'chua neo sau {self.params.anchor_wait_s:g} s cho o tran '
                    f'{self.params.takeoff_alt_m:g} m - ha khan cap')
            return Action(velocity_up_mps=0.0, detail='cham tran, chua neo - lo lung cho')
        return Action(velocity_up_mps=TAKEOFF_CLIMB_MPS, detail='leo')

    def _step_enroute(self, snap):
        wp = self.current_waypoint()
        # Tam bai DO tu tag neu da thay (thay tu xa van bay ve dung tam tag), khong thi khai bao.
        pad = self._pad_pose(wp)[0]
        target = (pad[0], pad[1], wp.target[2])
        act = Action(position_target=target, max_vel_mps=wp.max_vel_mps)
        if snap.position is None:
            act.detail = f'bay toi tag {wp.marker_id}: chua co vi tri EKF'
            return act
        # Chi so khoang cach NGANG: z cua EKF chua co moc tuyet doi khi khong thay tag.
        dx, dy = target[0] - snap.position[0], target[1] - snap.position[1]
        dist = math.hypot(dx, dy)
        if (self.params.yaw_control and not self.enroute_turned and snap.yaw is not None
                and dist > HEADING_HOLD_NEAR_M):
            bearing = math.atan2(dy, dx)
            err = abs(wrap(bearing - snap.yaw))
            if err > math.radians(ENROUTE_TURN_DONE_DEG):
                if self.enroute_hold is None:
                    self.enroute_hold = snap.position[:2]
                act.position_target = (*self.enroute_hold, target[2])
                act.yaw_target = bearing
                act.detail = (f'quay mui tai cho ve tag {wp.marker_id}: con lech '
                              f'{math.degrees(err):.0f} do')
                return act
        self.enroute_turned = True
        h = snap.range_m if snap.range_m is not None else wp.alt_m
        near = min(wp.acceptance_radius_m, CAM_DOWN_DIST_PER_M * h)
        if dist <= near:
            return self._enter_search(snap.now_s, MARKER_SEARCH,
                                      f'toi diem {self.current_wp_index} (cach {dist:.2f} m) - '
                                      f'camera thang xuong, tim tag {wp.marker_id}')
        if self.params.yaw_control and dist > HEADING_HOLD_NEAR_M:
            act.yaw_target = math.atan2(dy, dx)        # mui theo duong bay: camera nhin phia truoc
        act.detail = f'bay toi tag {wp.marker_id}'
        return act

    def _pad_pose(self, wp):
        """(tam bai, huong bai) - DO tu tag neu da thay, khong thi khai bao (huong co the None)."""
        obs = self.last_snap.pad_obs if self.last_snap is not None else None
        if obs is not None:
            return obs[:3], obs[3]
        return wp.pad, wp.pad_yaw

    def _search_point(self, wp):
        """Diem treo khi tim tag: tren tam bai (do neu da thay), o do cao alt_m."""
        pad = self._pad_pose(wp)[0]
        return (pad[0], pad[1], wp.target[2])

    def _hold_yaw(self, wp):
        """Yaw giu khi tim/ha tren bai co huong: mui = huong bai (hop hang sau duoi)."""
        return self._pad_pose(wp)[1] if self.params.yaw_control else None

    def _step_marker_search(self, snap):
        now = snap.now_s
        wp = self.current_waypoint()
        act = Action(position_target=self._search_point(wp), max_vel_mps=wp.max_vel_mps,
                     expected_marker_id=wp.marker_id, yaw_target=self._hold_yaw(wp),
                     detail=f'tim tag {wp.marker_id}')
        if snap.target_offset_m is not None:
            if not self._cam_at(snap, CAM_DOWN_DEG):
                act.detail = f'thay tag {wp.marker_id} - cho camera thang xuong'
                return act
            if self._hold_yaw(wp) is not None:
                return self._enter_search(now, ALIGN_HEADING,
                                          f'thay dung tag {wp.marker_id} - quay mui theo bai')
            return self._enter_search(now, PRECISION_LAND,
                                      f'thay dung tag {wp.marker_id} - ha chinh xac')
        timeout = self.search_timeout_s or self.params.search_timeout_s
        waited = self.time_in_state(now)
        if waited >= timeout or snap.failsafe_escalate_to == ESCALATE_RETRY_LOITER:
            return self._search_failed(now, RETRY_LOITER,
                                       f'khong thay tag {wp.marker_id} sau {waited:.0f} s')
        return act

    def _step_align_heading(self, snap):
        """Treo tren tam bai (bam tag to), quay mui = huong bai roi moi ha thang dung.

        Gap / tha (user 10-25): vao THANG diem hop (tam bai DO + box_forward_m) va quay mui o do, giu
        theo EKF da neo tag to (khong bam tam tag), xong thi ha ngay tai diem hop: gap -> FINAL_APPROACH,
        tha -> ACTUATE_GRIPPER."""
        now = snap.now_s
        wp = self.current_waypoint()
        pad, pad_yaw = self._pad_pose(wp)
        hover = wp.action in (ACTION_PICKUP, ACTION_DROPOFF)
        xy = self._box_point(wp) if hover else pad[:2]
        act = Action(expected_marker_id=-1 if hover else wp.marker_id, velocity_up_mps=0.0,
                     yaw_target=pad_yaw, position_target=(xy[0], xy[1], wp.target[2]),
                     max_vel_mps=LAND_EKF_MAX_VEL_MPS)
        if not (snap.pad_seen if hover else snap.target_offset_m is not None):
            if self.tag_lost_since is None:
                self.tag_lost_since = now
            if now - self.tag_lost_since > TAG_GAP_RECOVER_S:
                return self._search_failed(now, MARKER_SEARCH,
                                           f'mat tag {wp.marker_id} khi quay mui')
        else:
            self.tag_lost_since = None
        if self.time_in_state(now) >= ALIGN_HEADING_TIMEOUT_S:
            return self._after_align(snap, wp, f'qua {ALIGN_HEADING_TIMEOUT_S:.0f} s chua quay xong')
        if snap.yaw is None or pad_yaw is None:
            act.detail = 'quay mui theo bai: chua co yaw - giu'
            return act
        err = math.degrees(abs(wrap(pad_yaw - snap.yaw)))
        if err > ALIGN_HEADING_DONE_DEG:
            self.heading_ok_since = None
            act.detail = f'quay mui theo bai: con lech {err:.0f} do'
            return act
        if self.heading_ok_since is None:
            self.heading_ok_since = now
        if now - self.heading_ok_since >= ALIGN_HEADING_HOLD_S:
            return self._after_align(snap, wp, f'mui da theo bai (lech {err:.1f} do)')
        act.detail = f'quay mui theo bai: lech {err:.1f} do, giu {ALIGN_HEADING_HOLD_S:.0f} s'
        return act

    def _after_align(self, snap, wp, ly_do):
        """Xong ALIGN_HEADING: gap -> ha tim hop, tha -> ha toi diem tha (ca hai tai diem hop),
        diem khong hanh dong -> ha chinh xac tren tam bai."""
        now = snap.now_s
        if wp.action == ACTION_PICKUP:
            act = self.transition(FINAL_APPROACH, now, f'{ly_do} - ha tai diem hop, tim tag hop')
            act.expected_marker_id = wp.marker_id + self.params.small_id_offset
            return act
        if wp.action == ACTION_DROPOFF:
            act = self.transition(ACTUATE_GRIPPER, now, f'{ly_do} - ha tai diem tha')
            self.cam_cmd = CAM_DROP_DEG
            self.drop_hold_m = snap.range_m
            return act
        return self._enter_search(now, PRECISION_LAND, f'{ly_do} - ha thang dung')

    def _step_retry_loiter(self, snap):
        wp = self.current_waypoint()
        if self.time_in_state(snap.now_s) >= RETRY_LOITER_S:
            return self._enter_search(snap.now_s, MARKER_SEARCH,
                                      f'tim lai tag {wp.marker_id} (lan thu lai {self.retry_count})')
        return Action(position_target=self._search_point(wp), max_vel_mps=wp.max_vel_mps,
                      yaw_target=self._hold_yaw(wp),
                      detail=f've diem {self.current_wp_index}, giu {RETRY_LOITER_S:.0f} s roi tim lai')

    def _enter_search(self, now_s, new_state, detail):
        """Chuyen sang trang thai can tag, GIU expected_marker_id ngay chu ky chuyen: phat -1 mot
        chu ky la landing_target_bridge_node quen tag dang bam."""
        act = self.transition(new_state, now_s, detail)
        if new_state in (MARKER_SEARCH, ALIGN_HEADING, PRECISION_LAND):
            act.expected_marker_id = self.current_waypoint().marker_id
        act.yaw_target = self._hold_yaw(self.current_waypoint())
        return act

    def _search_failed(self, now_s, retry_state, reason):
        """Mot lan tim/ha theo tag that bai. Qua max_retries -> ha canh tai cho, khong lap them."""
        limit = self.max_retries if self.max_retries is not None else self.params.max_retries
        self.retry_count += 1
        if self.retry_count > limit:
            # Ha canh tai cho vi KHONG lam duoc viec. Khong ghi day thi duong nay ket thuc o
            # MISSION_COMPLETE giong het chuyen thanh cong, va GCS bao "xong" cho mot chuyen
            # chua bao gio tim thay tag.
            self._ghi_ket_qua(RESULT_RETRIES_EXHAUSTED)
            return self.transition(EMERGENCY_LAND, now_s,
                                   f'{reason} - het {limit} lan thu lai, ha canh tai cho')
        return self._enter_search(now_s, retry_state,
                                  f'{reason} - thu lai {self.retry_count}/{limit}')

    def _step_actuate_gripper(self, snap):
        """Gap hoac tha hang. PICKUP: drone TREO o ToF GRIP_HOVER_M (giu do cao theo ToF); DROPOFF:
        dich ve diem hop roi xuong treo o DROP_HOVER_M (_drop_descend); diem khong hanh dong: drone
        NAM DAT va VAN ARM.

        Nguyen tac 3 (dau file): chi roi trang thai khi gripper_sensor_confirmed doi DUNG CHIEU -
        khong bao gio dung timeout thay xac nhan. Timeout chi de failsafe PHAT HIEN su co, va su
        co do vao day duoi dang ESCALATE_RETRY_LOITER.

        PICKUP xong = da gap chac (sensor_confirmed True). DROPOFF xong = da nha ra
        (sensor_confirmed False) - dieu kien NGUOC nhau, khong phai cung mot co.
        """
        now = snap.now_s
        wp = self.current_waypoint()
        if wp is None:
            return self.transition(EMERGENCY_LAND, now, 'khong con diem nao - ha canh')
        # ACTION_NONE phai xet RIENG. Truoc day ham chi phan hai nhanh (PICKUP / con lai) nen diem
        # khong co hanh dong bi xu ly nhu DROPOFF: log ghi "tha hang" cho mot diem khong tha gi.
        # Khong lenh gripper nao duoc phat (dieu kien xong cua dropoff dung ngay khi khong giu gi)
        # nen khong gay hai, nhung log noi doi. Giao uoc GCS<->Pi 3.2b cau 3.
        if wp.action == ACTION_NONE:
            if self.time_in_state(now) < self.params.pre_dropoff_settle_s:
                return Action(velocity_up_mps=0.0, detail='giu on dinh (diem khong co hanh dong)')
            return self._xong_hanh_dong(now, wp, 'ghe')

        dong = wp.action == ACTION_PICKUP
        xong = snap.gripper_sensor_confirmed if dong else not snap.gripper_sensor_confirmed
        viec = 'gap' if dong else 'tha'
        vz = self._hold_range(snap.range_m, GRIP_HOVER_M if dong else DROP_HOVER_M)
        # Giu xy tai tam hop (lay) / diem hop cua bai (tha): de tu do thi drone troi (mo phong 10-22:
        # lech 4 cm, vet ToF len nap hop -> "mat laser" dung luc gap).
        xy = (self.box_xy or self._box_point(wp)) if dong else self._box_point(wp)
        giu = dict(position_target=(xy[0], xy[1], wp.target[2]), max_vel_mps=LAND_EKF_MAX_VEL_MPS)

        if not dong and self.drop_ready_s is None:
            return self._drop_descend(snap, wp, xy, giu)

        # Vua toi do cao gap, may con dao dong: giu yen roi moi cho co cau chay.
        moc = self.state_entered_s if dong else self.drop_ready_s
        if now - moc < self.params.pre_dropoff_settle_s:
            return Action(velocity_up_mps=vz, detail=f'giu on dinh truoc khi {viec}', **giu)

        if xong:
            return self._xong_hanh_dong(now, wp, viec)

        # Gripper khong xac nhan qua lau -> mot lan that bai cua CA chang, thu lai tu dau.
        if snap.failsafe_escalate_to == ESCALATE_RETRY_LOITER:
            return self._search_failed(now, RETRY_LOITER,
                                       f'gripper khong xac nhan khi {viec} tai tag {wp.marker_id}')

        if dong and self.last_gripper_command_s is None and snap.position is not None:
            lech = math.hypot(snap.position[0] - xy[0], snap.position[1] - xy[1])
            if (lech > GRIP_CENTER_TOL_M and self.time_in_state(now)
                    < self.params.pre_dropoff_settle_s + GRIP_CENTER_WAIT_S):
                return Action(velocity_up_mps=vz, detail=f'cho vao tam hop: lech {lech * 100:.1f} cm',
                              **giu)

        act = Action(velocity_up_mps=vz, detail=f'{viec} hang tai tag {wp.marker_id}', **giu)
        if (self.last_gripper_command_s is None
                or now - self.last_gripper_command_s >= GRIPPER_RETRY_S):
            act.gripper_command = 'close' if dong else 'open'
            self.last_gripper_command_s = now
        return act

    def _drop_descend(self, snap, wp, xy, giu):
        """Tha: GIU do cao luc vao trang thai (theo ToF - chim thi leo lai, user 10-25: hop khong duoc
        cham dat truoc khi vao diem) va dich ngang ve diem hop; vao diem thi ha cham toi DROP_HOVER_M
        (day hop cach dat ~5 cm) roi moi tinh thoi gian on dinh. Chua vao diem thi KHONG bao gio
        coi la toi do cao tha, du ToF da thap."""
        now = snap.now_s
        if self.time_in_state(now) >= DROP_MOVE_TIMEOUT_S:
            return self._search_failed(now, RETRY_LOITER,
                                       f'qua {DROP_MOVE_TIMEOUT_S:.0f} s chua toi diem tha tag '
                                       f'{wp.marker_id}')
        act = Action(velocity_up_mps=0.0, **giu)
        if snap.position is None or snap.range_m is None:
            act.detail = 'tha: mat EKF / laser - giu'
            return act
        lech = math.hypot(snap.position[0] - xy[0], snap.position[1] - xy[1])
        if lech > DROP_ALIGN_TOL_M or not self._cam_at(snap, CAM_DROP_DEG):
            act.velocity_up_mps = self._hold_range(snap.range_m, self.drop_hold_m or BOX_SEARCH_ALT_M)
            act.detail = f've diem tha: lech {lech * 100:.1f} cm'
            return act
        if snap.range_m <= DROP_HOVER_M + GRIP_STOP_LEAD_M:
            self.drop_ready_s = now
            # Thao tac kep bat dau tu day: failsafe_monitor tinh grip_confirm_timeout_s tu luc vao
            # trang thai, ma dich + ha tu 2 m mat ~15 s -> mo kep xong (sensor_confirmed False = tha
            # thanh cong) se bi bao "gripper chua xac nhan" gia (mo phong 10-25).
            self.state_entered_s = now
            act.velocity_up_mps = self._hold_range(snap.range_m, DROP_HOVER_M)
            act.detail = f'treo {snap.range_m:.2f} m tren diem tha - giu on dinh'
            return act
        slow = snap.range_m <= DROP_HOVER_M + GRIP_SLOW_BELOW_M - GRIP_HOVER_M
        act.velocity_up_mps = -(GRIP_FINAL_MPS if slow else descent_mps(snap.range_m))
        act.detail = f'xuong diem tha, lech {lech * 100:.1f} cm'
        return act

    def _grip_descent(self, range_m):
        """Toc do xuong (duong) khi toi do cao gap/tha: cham han o doan cuoi."""
        return GRIP_FINAL_MPS if range_m <= GRIP_SLOW_BELOW_M else LAND_DESCENT_NEAR_MPS

    def _hold_range(self, range_m, target_m):
        """vz giu ToF quanh target_m (treo gap/tha); mat laser thi giu vz = 0."""
        if range_m is None:
            return 0.0
        vz = GRIP_HOLD_KP * (target_m - range_m)
        return max(-LAND_DESCENT_NEAR_MPS, min(LAND_DESCENT_NEAR_MPS, vz))

    def _xong_hanh_dong(self, now, wp, viec):
        """Xong viec tai diem: con diem sau thi cat canh lai, het thi ha canh va disarm."""
        if wp.action == ACTION_PICKUP:
            self.reanchor_id = wp.marker_id     # tag hop KHONG neo EKF: phai thay lai tag to
            self.box_takeoff = True
        self.current_wp_index += 1
        if self.current_wp_index >= len(self.waypoints):
            # Dang nam dat: _step_land thay landed se DISARM ngay, khong xuong them.
            return self.transition(EMERGENCY_LAND, now,
                                   f'xong {viec} tai tag {wp.marker_id} - het ke hoach, disarm')
        return self.transition(TAKEOFF, now, f'xong {viec} tai tag {wp.marker_id} - '
                                             f'cat canh toi diem {self.current_wp_index}')

    def _step_land(self, snap):
        now = snap.now_s
        if not snap.armed:
            return self.transition(MISSION_COMPLETE, now, 'da cham dat va disarm')
        act = Action(velocity_up_mps=-descent_mps(snap.range_m), detail='xuong')
        return self._descend_and_disarm(snap, act)

    def _step_precision_land(self, snap):
        """Xuong theo tag cua waypoint hien tai - chi xuong khi da vao tam.

        position_controller_node can tam (vx, vy) theo pose tag; mat tag thi bam TAM BAI DA BIET
        (position_target, theo EKF). FSM quyet dinh vz: mat tag ngan ma con gan tam -> ha tiep;
        troi xa -> dung, ve tam; mat qua lau -> MARKER_SEARCH (khong ha mu).
        """
        now = snap.now_s
        wp = self.current_waypoint()
        final = self.current_wp_index == len(self.waypoints) - 1 and wp.action == ACTION_NONE
        if not snap.armed:
            if final and self.last_fc_command_s is not None:
                return self.transition(MISSION_COMPLETE, now,
                                       f'da ha xuong tag {wp.marker_id} va disarm')
            self._ghi_ket_qua(RESULT_FAILSAFE)
            return self.transition(FAILSAFE, now, 'bi disarm khi dang ha chinh xac')
        if snap.landed and not final:
            return self.transition(ACTUATE_GRIPPER, now, f'cham dat tai tag {wp.marker_id}')
        hover = wp.action in (ACTION_PICKUP, ACTION_DROPOFF)
        if hover and snap.range_m is not None and snap.ob_auth is True:
            if wp.action == ACTION_PICKUP and snap.range_m <= BOX_SEARCH_ALT_M:
                act = self.transition(FINAL_APPROACH, now,
                                      f'{snap.range_m:.2f} m tren bai {wp.marker_id} - tim hop')
                act.expected_marker_id = wp.marker_id + self.params.small_id_offset
                return act
            if (wp.action == ACTION_DROPOFF and snap.target_offset_m is not None
                    and snap.target_offset_m <= PRECISION_ALIGN_BASE_M
                    + PRECISION_ALIGN_PER_M * snap.range_m):
                act = self.transition(ACTUATE_GRIPPER, now,
                                      f'da can tam bai {wp.marker_id} o {snap.range_m:.2f} m - '
                                      f've diem tha')
                self.cam_cmd = CAM_DROP_DEG
                self.drop_hold_m = snap.range_m
                return act
        pad = self._pad_pose(wp)[0]
        act = Action(expected_marker_id=wp.marker_id, velocity_up_mps=0.0,
                     yaw_target=self._hold_yaw(wp), position_target=tuple(pad),
                     max_vel_mps=LAND_EKF_MAX_VEL_MPS)
        if snap.ob_auth is not True:
            act.detail = 'khong biet quyen - giu vz = 0'
            return act
        low = snap.range_m is not None and snap.range_m <= PRECISION_BLIND_BELOW_M
        if snap.landed or low:
            # Tha hang khi treo: doan mu xuong cham, khong lao nhanh nhu luc cham dat.
            act.velocity_up_mps = -(self._grip_descent(snap.range_m) if hover
                                    else descent_mps(snap.range_m))
            act.detail = 'sat dat - xuong tiep'
            return self._descend_and_disarm(snap, act) if final else act
        if snap.target_offset_m is None:
            if self.tag_lost_since is None:
                self.tag_lost_since = now
            lost = now - self.tag_lost_since
            if snap.position is None or snap.range_m is None:
                # Khong co EKF / laser thi khong biet troi hay khong: giu cach cu.
                if self.time_in_state(now) < PRECISION_ACQUIRE_S:
                    act.detail = f'cho bat tag {wp.marker_id}'
                    return act
                return self._search_failed(now, MARKER_SEARCH,
                                           f'mat tag {wp.marker_id} - khong ha mu')
            dist = math.hypot(snap.position[0] - pad[0], snap.position[1] - pad[1])
            allowed = PRECISION_ALIGN_BASE_M + PRECISION_ALIGN_PER_M * snap.range_m
            if lost <= TAG_GAP_GRACE_S and dist <= allowed:
                act.velocity_up_mps = -descent_mps(snap.range_m)
                act.detail = (f'mat tag {lost:.1f} s, EKF lech tam {dist:.2f} m - ha tiep, '
                              f'giu tam bai')
                return act
            if lost <= TAG_GAP_RECOVER_S:
                act.detail = (f'mat tag {lost:.1f} s, EKF lech tam {dist:.2f} m > {allowed:.2f} - '
                              f'dung ha, ve tam bai')
                return act
            return self._search_failed(now, MARKER_SEARCH, f'mat tag {wp.marker_id} - khong ha mu')
        self.tag_lost_since = None
        if snap.range_m is None:
            act.detail = 'mat laser - giu vz = 0'
            return act
        allowed = PRECISION_ALIGN_BASE_M + PRECISION_ALIGN_PER_M * snap.range_m
        if snap.target_offset_m > allowed:
            act.detail = f'cho can tam: lech {snap.target_offset_m:.2f} m > {allowed:.2f} m'
            return act
        act.velocity_up_mps = -descent_mps(snap.range_m)
        act.detail = f'xuong theo tag, lech {snap.target_offset_m:.2f} m'
        return act

    def _box_point(self, wp):
        """(x, y) tam hop hang khai bao: tam bai + box_forward_m theo huong bai (do neu da thay)."""
        pad, yaw = self._pad_pose(wp)
        if yaw is None:
            # Bai khong khai huong, chua do duoc: mui drone dang theo huong ha (gan dung).
            snap = self.last_snap
            yaw = snap.yaw if snap is not None and snap.yaw is not None else 0.0
        f = self.params.box_forward_m
        return pad[0] + f * math.cos(yaw), pad[1] + f * math.sin(yaw)

    def _step_final_approach(self, snap):
        """Lay hang (docs/ke_hoach_camera_servo_gap_hang.md muc 3), nam pha:

        'search'  - tai vi tri hop (do tu tag to): cao hon BOX_SEARCH_ALT_M (vao tu ALIGN_HEADING) thi
                    ha tu tu theo z EKF (neo bang tag to - KHONG theo ToF: vet sang quet qua nap hop),
                    toi muc do thi giu; camera thang xuong, thay tag hop (id_to + offset) -> 'tilt';
        'tilt'    - nghieng camera CAM_BOX_DEG (ve duoi), giu nguyen cho toi khi on dinh -> 'descend';
        'descend' - bam tag hop (controller dat TAM DRONE = co cau gap len tag), ha cham; ToF <=
                    box_cam_switch_m -> 'grip_tilt';
        'grip_tilt' - dung (vz 0, giu tam hop), nghieng camera CAM_GRIP_DEG -> 'final';
        'final'   - bam tag hop nhu 'descend'; ToF <= GRIP_HOVER_M -> ACTUATE_GRIPPER.
        Khong thay / mat tag hop / camera khong toi goc -> mot lan that bai (RETRY_LOITER).
        """
        now = snap.now_s
        wp = self.current_waypoint()
        box_id = wp.marker_id + self.params.small_id_offset
        if (snap.box_obs is not None and snap.range_m is not None
                and (snap.range_m > self.params.box_cam_switch_m + BOX_OBS_MARGIN_M
                     or self.box_phase == 'final')):
            self.box_obs_list = (self.box_obs_list + [snap.box_obs[:2]])[-BOX_OBS_N:]
            xs = sorted(o[0] for o in self.box_obs_list)
            ys = sorted(o[1] for o in self.box_obs_list)
            self.box_xy = (xs[len(xs) // 2], ys[len(ys) // 2])
        bx, by = self.box_xy or self._box_point(wp)
        # Cao hon BOX_SEARCH_ALT_M (vao tu ALIGN_HEADING o do cao tim tag): giu do cao dang co, khong
        # de vong z EKF keo tut xuong trong luc chua thay hop / dang nghieng camera.
        h = None if snap.position is None else snap.position[2] - wp.pad[2]
        z = wp.pad[2] + (BOX_SEARCH_ALT_M if h is None else max(BOX_SEARCH_ALT_M, h))
        act = Action(expected_marker_id=box_id, velocity_up_mps=0.0, yaw_target=self._hold_yaw(wp),
                     position_target=(bx, by, z), max_vel_mps=LAND_EKF_MAX_VEL_MPS)
        if snap.ob_auth is not True:
            act.detail = 'khong biet quyen - giu vz = 0'
            return act
        if snap.range_m is None:
            act.detail = 'mat laser - giu vz = 0'
            return act
        seen = snap.target_offset_m is not None
        if self.box_phase == 'search':
            act.velocity_up_mps = None          # z theo position_target (EKF), xem docstring
            if seen and self._cam_at(snap, CAM_DOWN_DEG):
                self.box_phase, self.box_phase_s = 'tilt', now
                self.cam_cmd = CAM_BOX_DEG
                act.detail = f'thay tag hop {box_id} - nghieng camera {CAM_BOX_DEG:+.0f} do'
                return act
            if h is not None and h > BOX_SEARCH_ALT_M + 0.02:
                # Ha tu tu tai diem hop toi khi thay tag hop; lech thi dung, keo ve.
                self.box_phase_s = now          # dong ho het gio tinh tu luc thoi ha
                lech = math.hypot(snap.position[0] - bx, snap.position[1] - by)
                if lech > BOX_SEARCH_XY_TOL_M:
                    act.velocity_up_mps = 0.0
                    act.detail = f'tim tag hop: ve diem hop, lech {lech * 100:.0f} cm'
                else:
                    act.velocity_up_mps = -descent_mps(h)
                    act.detail = f'ha tai diem hop tim tag hop {box_id}'
                return act
            if now - self.box_phase_s >= BOX_SEARCH_TIMEOUT_S:
                return self._search_failed(now, RETRY_LOITER,
                                           f'khong thay tag hop {box_id} sau '
                                           f'{BOX_SEARCH_TIMEOUT_S:.0f} s')
            act.detail = f'tim tag hop {box_id}'
            return act
        if self.box_phase == 'tilt':
            act.velocity_up_mps = None
            if self._cam_at(snap, CAM_BOX_DEG):
                self.box_phase, self.box_phase_s = 'descend', now
                self.tag_lost_since = None
                act.detail = 'camera da nghieng - ha toi hop'
                return act
            if now - self.box_phase_s >= CAM_SETTLE_TIMEOUT_S:
                return self._search_failed(now, RETRY_LOITER,
                                           f'camera khong toi {CAM_BOX_DEG:+.0f} do sau '
                                           f'{CAM_SETTLE_TIMEOUT_S:.0f} s')
            act.detail = f'cho camera nghieng {CAM_BOX_DEG:+.0f} do'
            return act
        if self.box_phase == 'descend' and snap.range_m <= self.params.box_cam_switch_m:
            self.box_phase, self.box_phase_s = 'grip_tilt', now
            self.cam_cmd = CAM_GRIP_DEG
            act.detail = f'sat hop - dung, nghieng camera {CAM_GRIP_DEG:+.0f} do'
            return act
        if self.box_phase == 'grip_tilt':
            if self._cam_at(snap, CAM_GRIP_DEG):
                self.box_phase, self.box_phase_s = 'final', now
                self.tag_lost_since = None
                act.detail = f'camera {CAM_GRIP_DEG:+.0f} do - ha toi do cao gap'
                return act
            if now - self.box_phase_s >= CAM_SETTLE_TIMEOUT_S:
                return self._search_failed(now, RETRY_LOITER,
                                           f'camera khong toi {CAM_GRIP_DEG:+.0f} do sau '
                                           f'{CAM_SETTLE_TIMEOUT_S:.0f} s')
            act.detail = f'cho camera nghieng {CAM_GRIP_DEG:+.0f} do'
            return act
        if snap.range_m <= GRIP_HOVER_M + GRIP_STOP_LEAD_M:
            return self.transition(ACTUATE_GRIPPER, now,
                                   f'treo {snap.range_m:.2f} m tren hop {box_id} - gap')
        if not seen:
            if self.tag_lost_since is None:
                self.tag_lost_since = now
            lost = now - self.tag_lost_since
            if lost > TAG_GAP_RECOVER_S:
                return self._search_failed(now, RETRY_LOITER, f'mat tag hop {box_id}')
            act.detail = f'mat tag hop {lost:.1f} s - dung ha, giu vi tri hop'
            return act
        self.tag_lost_since = None
        allowed = PRECISION_ALIGN_BASE_M + PRECISION_ALIGN_PER_M * snap.range_m
        if snap.target_offset_m > allowed:
            act.detail = f'can tam hop: lech {snap.target_offset_m:.2f} m > {allowed:.2f} m'
            return act
        act.velocity_up_mps = -self._grip_descent(snap.range_m)
        act.detail = f'xuong theo tag hop, lech {snap.target_offset_m:.2f} m'
        return act

    def _descend_and_disarm(self, snap, act):
        """Giu lenh xuong (tieu chi cham dat can no); landed + OB_DIS_RDY -> DISARM thuong."""
        now = snap.now_s
        if snap.landed and self._fc_command_pending(now, DISARM_RETRY_S):
            # OB_DIS_RDY ve 0 truoc khi /mavros/state bao disarm.
            act.detail = 'da gui DISARM, cho FC xac nhan'
        elif snap.landed and snap.disarm_ready and snap.ob_auth is True:
            act.detail = 'cham dat - DISARM'
            act.fc_command = 'disarm'
            self.last_fc_command_s = now
        elif snap.landed:
            act.detail = 'cham dat, cho OB_DIS_RDY = 1'
        return act
