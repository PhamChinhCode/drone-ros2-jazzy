"""May trang thai nhiem vu, thuan Python - khong phu thuoc rclpy de pytest duoc moi nhanh loi
(mat marker, mat gripper confirm, het pin giua chung) truoc khi bay.

Ba nguyen tac bat buoc (muc 4.1 tai lieu huong dan):
  1. KHONG BAO GIO chuyen sang ha canh chi dua vao GPS/EKF - luon cho landing_target_bridge_node
     xac thuc dung ID marker truoc ("hai lop dinh vi bo tro");
  2. MARKER_SEARCH co retry_count/max_retries - het luot thi RETRY_LOITER roi bao failsafe,
     khong lap vo han;
  3. TUYET DOI khong dung timeout thay cho gripper_status.sensor_confirmed - hang co the chua
     gap chac du da du thoi gian.
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

# Bang chuyen trang thai hop le. Moi trang thai chi duoc di toi cac trang thai liet ke o day;
# nho vay them mot nhanh moi la sua bang nay chu khong phai doc het than ham.
TRANSITIONS = {
    IDLE:            [TAKEOFF, FAILSAFE],
    TAKEOFF:         [ENROUTE, FAILSAFE, EMERGENCY_LAND],
    ENROUTE:         [MARKER_SEARCH, RTH, FAILSAFE, EMERGENCY_LAND],
    # MARKER_SEARCH -> EMERGENCY_LAND: het luot tim. -> RTH: su co muc 3 khi dang treo tim tag,
    # ve nha tot hon la ha xuong cho la.
    MARKER_SEARCH:   [PRECISION_LAND, RETRY_LOITER, RTH, FAILSAFE, EMERGENCY_LAND],
    PRECISION_LAND:  [ACTUATE_GRIPPER, MARKER_SEARCH, FAILSAFE, MISSION_COMPLETE, EMERGENCY_LAND],
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
LAND_DESCENT_MPS = 0.5     # position_controller_node tu kep 0,285 m/s khi laser <= 1,2 m
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
PRECISION_ALIGN_BASE_M = 0.10   # chi xuong khi tag lech ngang <= BASE + PER_M x do cao laser
PRECISION_ALIGN_PER_M = 0.25
# Duoi do cao nay camera (nghieng 20 do, truoc tam 6 cm) khong con thay tag tron ven - xuong
# tiep khong can tag. Tren muc nay mat tag la dung, KHONG ha mu.
PRECISION_BLIND_BELOW_M = 0.35

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

# Ke hoach nhiem vu. Hanh dong tai diem trung MissionWaypoint.ACTION_*.
ACTION_NONE, ACTION_PICKUP, ACTION_DROPOFF = 0, 1, 2
MAX_WAYPOINT_VEL_MPS = 1.9  # 95 % tran ngang FC (giao uoc 9.6, P9)


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


def parse_known_tags(flat):
    """[id, x, y, z, ...] -> {id: (x, y, z)}. Cung dinh dang config/tags.yaml."""
    if len(flat) % 4:
        raise ValueError(f'known_tags phai co boi so cua 4 phan tu, dang co {len(flat)}')
    return {int(flat[i]): tuple(flat[i + 1:i + 4]) for i in range(0, len(flat), 4)}


def build_waypoints(raw, known_tags):
    """raw: list dict (seq, marker_id, action, alt_m, acceptance_radius_m, max_vel_mps, loiter_s).
    Tra (list Waypoint, '') hoac (None, ly do tu choi). Sai mot diem la tu choi ca ke hoach."""
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
                            w['max_vel_mps'], w['loiter_s'], (tx, ty, tz + w['alt_m'])))
    return out, ''


@dataclass
class Snapshot:
    """Anh chup toan bo dau vao mot chu ky - de FSM la ham thuan, khong tu doc topic."""

    now_s: float = 0.0
    armed: bool = False
    battery_pct: float = 100.0
    # Vi tri EKF (x, y, z) trong odom; None = chua co / qua han.
    position: tuple = None
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


@dataclass
class Params:
    search_timeout_s: float = 20.0
    max_retries: int = 3
    acceptance_radius_m: float = 1.5
    pre_dropoff_settle_s: float = 2.0
    takeoff_alt_m: float = 5.0


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
    # Rieng cho nhip phat lai lenh gap/tha. KHONG dung chung last_fc_command_s: truong do
    # danh cho nhip thu lai DISARM trong _descend_and_disarm.
    last_gripper_command_s: float = None
    search_timeout_s: float = None

    def load_plan(self, mission_id, raw_waypoints, max_retries, search_timeout_s, known_tags):
        """Nap ke hoach. Tra ly do tu choi, '' neu nhan. KHONG tu cat canh - van phai request_start.

        max_retries / search_timeout_s <= 0 nghia la dung gia tri trong params.
        """
        if self.state != IDLE:
            return f'dang {self.state}, chi nhan ke hoach khi IDLE'
        if self.start_requested_s is not None:
            return 'dang cho cat canh theo ke hoach cu - khong doi ke hoach luc nay'
        waypoints, err = build_waypoints(raw_waypoints, known_tags)
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
        if new_state == MISSION_COMPLETE:
            # Chua ly do nao khac duoc ghi = khong co gi cat ngang = lam het ke hoach.
            self._ghi_ket_qua(RESULT_COMPLETED)
        self.state = new_state
        self.state_entered_s = now_s
        # retry_count tinh theo TUNG DIEM: chi xoa khi bat dau chang moi (hoac ve IDLE), de vong
        # MARKER_SEARCH <-> RETRY_LOITER / PRECISION_LAND khong bao gio lap vo han.
        if new_state in (IDLE, ENROUTE):
            self.retry_count = 0
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

    def time_in_state(self, now_s):
        return now_s - self.state_entered_s

    def _fc_command_pending(self, now_s, period_s):
        """True neu lenh FC vua gui chua qua period_s - chua duoc gui lai."""
        return self.last_fc_command_s is not None and now_s - self.last_fc_command_s < period_s

    def step(self, snap):
        """Chay mot chu ky FSM, tra ve Action.

        Da hien thuc lop FC (giao uoc 6.2, 6.3, P2/P10):
          - dang arm ma mat quyen (OB_AUTH = 0) hoac KHOA -> FAILSAFE, KHONG gui lenh nao nua
            (ke ca DISARM); nguoi lai cat ch5 hoac gat ch8 xuong-len. OB_AUTH chua biet -> giu
            nguyen, khong gui lenh FC moi;
          - FAILSAFE: da disarm -> IDLE; con arm va lay lai quyen -> EMERGENCY_LAND;
          - IDLE: request_start() -> gui ARM chi khi OB_ARM_RDY = 1, thua 2 s; armed -> TAKEOFF;
          - TAKEOFF: leo toi laser >= takeoff_alt_m roi giu vz = 0;
          - EMERGENCY_LAND: xuong; landed + OB_DIS_RDY = 1 -> DISARM thuong, thu lai 3 s;
            disarm xong -> MISSION_COMPLETE -> IDLE. KHONG tu goi 21196 (11.1 #12f).
          - request_land() -> EMERGENCY_LAND tu moi trang thai co nhanh do trong TRANSITIONS.

        Failsafe (/failsafe_event, muc nang nhat dang bat) - uu tien ngay sau kiem quyen:
          - RTH / EMERGENCY_LAND -> EMERGENCY_LAND (RTH tam thoi = ha canh tai cho);
          - LOITER -> FAILSAFE giu vz = 0; het su co -> ha canh (khong tu tiep tuc nhiem vu);
          - RETRY_LOITER -> chi MARKER_SEARCH dung (coi nhu het gio tim); trang thai khac bo qua;
          - dang bat bat ky muc nao -> IDLE khong arm.

        Trang thai nhiem vu (khong GPS, vi tri diem suy tu tag):
          - TAKEOFF: laser >= takeoff_alt_m -> ENROUTE neu da nap ke hoach, khong thi giu;
          - ENROUTE: phat diem den; cach ngang <= acceptance_radius_m cua diem -> MARKER_SEARCH;
          - MARKER_SEARCH: giu tai diem, phat expected_marker_id; landing_target_bridge_node xac
            thuc dung ID -> PRECISION_LAND; qua search_timeout_s (hoac failsafe RETRY_LOITER) ->
            mot lan that bai;
          - PRECISION_LAND: mat tag tren PRECISION_BLIND_BELOW_M -> MARKER_SEARCH, cung la mot lan
            that bai (khong ha mu, khong nhay qua lai vo han); landed -> ACTUATE_GRIPPER;
          - moi lan that bai retry_count += 1; qua max_retries -> EMERGENCY_LAND tai cho (RTH
            tam thoi); con luot -> RETRY_LOITER giu tai diem RETRY_LOITER_S roi tim lai.

        """
        now = snap.now_s
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
        if self.state in (ENROUTE, MARKER_SEARCH, RETRY_LOITER) and snap.ob_auth is not True:
            return Action(velocity_up_mps=0.0, detail='khong biet quyen - giu vz = 0')
        if self.state == ENROUTE:
            return self._step_enroute(snap)
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
        if snap.range_m >= self.params.takeoff_alt_m:
            wp = self.current_waypoint()
            if wp is None:
                return Action(velocity_up_mps=0.0, detail='du do cao - giu (chua nap ke hoach)')
            return self.transition(ENROUTE, snap.now_s,
                                   f'du do cao - bay toi diem {self.current_wp_index} '
                                   f'(tag {wp.marker_id})')
        return Action(velocity_up_mps=TAKEOFF_CLIMB_MPS, detail='leo')

    def _step_enroute(self, snap):
        wp = self.current_waypoint()
        act = Action(position_target=wp.target, max_vel_mps=wp.max_vel_mps)
        if snap.position is None:
            act.detail = f'bay toi tag {wp.marker_id}: chua co vi tri EKF'
            return act
        # Chi so khoang cach NGANG: z cua EKF chua co moc tuyet doi khi khong thay tag.
        dist = math.hypot(snap.position[0] - wp.target[0], snap.position[1] - wp.target[1])
        if dist <= wp.acceptance_radius_m:
            return self._enter_search(snap.now_s, MARKER_SEARCH,
                                      f'toi diem {self.current_wp_index} (cach {dist:.2f} m) - '
                                      f'tim tag {wp.marker_id}')
        act.detail = f'bay toi tag {wp.marker_id}'
        return act

    def _step_marker_search(self, snap):
        now = snap.now_s
        wp = self.current_waypoint()
        if snap.target_offset_m is not None:
            return self._enter_search(now, PRECISION_LAND,
                                      f'thay dung tag {wp.marker_id} - ha chinh xac')
        timeout = self.search_timeout_s or self.params.search_timeout_s
        waited = self.time_in_state(now)
        if waited >= timeout or snap.failsafe_escalate_to == ESCALATE_RETRY_LOITER:
            return self._search_failed(now, RETRY_LOITER,
                                       f'khong thay tag {wp.marker_id} sau {waited:.0f} s')
        return Action(position_target=wp.target, max_vel_mps=wp.max_vel_mps,
                      expected_marker_id=wp.marker_id, detail=f'tim tag {wp.marker_id}')

    def _step_retry_loiter(self, snap):
        wp = self.current_waypoint()
        if self.time_in_state(snap.now_s) >= RETRY_LOITER_S:
            return self._enter_search(snap.now_s, MARKER_SEARCH,
                                      f'tim lai tag {wp.marker_id} (lan thu lai {self.retry_count})')
        return Action(position_target=wp.target, max_vel_mps=wp.max_vel_mps,
                      detail=f've diem {self.current_wp_index}, giu {RETRY_LOITER_S:.0f} s roi tim lai')

    def _enter_search(self, now_s, new_state, detail):
        """Chuyen sang trang thai can tag, GIU expected_marker_id ngay chu ky chuyen: phat -1 mot
        chu ky la landing_target_bridge_node quen tag dang bam."""
        act = self.transition(new_state, now_s, detail)
        if new_state in (MARKER_SEARCH, PRECISION_LAND):
            act.expected_marker_id = self.current_waypoint().marker_id
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
        """Gap hoac tha hang tai diem da ha canh. Drone dang NAM DAT va VAN ARM.

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

        # Vua cham dat, khung may con rung: giu yen roi moi cho co cau chay.
        if self.time_in_state(now) < self.params.pre_dropoff_settle_s:
            return Action(velocity_up_mps=0.0, detail=f'giu on dinh truoc khi {viec}')

        if xong:
            return self._xong_hanh_dong(now, wp, viec)

        # Gripper khong xac nhan qua lau -> mot lan that bai cua CA chang, thu lai tu dau.
        if snap.failsafe_escalate_to == ESCALATE_RETRY_LOITER:
            return self._search_failed(now, RETRY_LOITER,
                                       f'gripper khong xac nhan khi {viec} tai tag {wp.marker_id}')

        act = Action(velocity_up_mps=0.0, detail=f'{viec} hang tai tag {wp.marker_id}')
        if (self.last_gripper_command_s is None
                or now - self.last_gripper_command_s >= GRIPPER_RETRY_S):
            act.gripper_command = 'close' if dong else 'open'
            self.last_gripper_command_s = now
        return act

    def _xong_hanh_dong(self, now, wp, viec):
        """Xong viec tai diem: con diem sau thi cat canh lai, het thi ha canh va disarm."""
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
        act = Action(velocity_up_mps=-LAND_DESCENT_MPS, detail='xuong')
        return self._descend_and_disarm(snap, act)

    def _step_precision_land(self, snap):
        """Xuong theo tag cua waypoint hien tai - chi xuong khi da vao tam, mat tag thi dung.

        position_controller_node tu can tam (vx, vy) theo pose tag; FSM chi quyet dinh vz.
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
        act = Action(expected_marker_id=wp.marker_id, velocity_up_mps=0.0)
        if snap.ob_auth is not True:
            act.detail = 'khong biet quyen - giu vz = 0'
            return act
        low = snap.range_m is not None and snap.range_m <= PRECISION_BLIND_BELOW_M
        if snap.landed or low:
            act.velocity_up_mps = -LAND_DESCENT_MPS
            act.detail = 'sat dat - xuong tiep'
            return self._descend_and_disarm(snap, act) if final else act
        if snap.target_offset_m is None:
            if self.time_in_state(now) < PRECISION_ACQUIRE_S:
                act.detail = f'cho bat tag {wp.marker_id}'
                return act
            return self._search_failed(now, MARKER_SEARCH, f'mat tag {wp.marker_id} - khong ha mu')
        if snap.range_m is None:
            act.detail = 'mat laser - giu vz = 0'
            return act
        allowed = PRECISION_ALIGN_BASE_M + PRECISION_ALIGN_PER_M * snap.range_m
        if snap.target_offset_m > allowed:
            act.detail = f'cho can tam: lech {snap.target_offset_m:.2f} m > {allowed:.2f} m'
            return act
        act.velocity_up_mps = -LAND_DESCENT_MPS
        act.detail = f'xuong theo tag, lech {snap.target_offset_m:.2f} m'
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
