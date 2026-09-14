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
    ENROUTE:         [MARKER_SEARCH, RTH, FAILSAFE],
    MARKER_SEARCH:   [PRECISION_LAND, RETRY_LOITER, FAILSAFE],
    PRECISION_LAND:  [ACTUATE_GRIPPER, MARKER_SEARCH, FAILSAFE],
    ACTUATE_GRIPPER: [TAKEOFF, ENROUTE, MISSION_COMPLETE, RETRY_LOITER, FAILSAFE],
    RETRY_LOITER:    [MARKER_SEARCH, RTH, FAILSAFE],
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


@dataclass
class Snapshot:
    """Anh chup toan bo dau vao mot chu ky - de FSM la ham thuan, khong tu doc topic."""

    now_s: float = 0.0
    armed: bool = False
    battery_pct: float = 100.0
    at_waypoint: bool = False
    marker_confirmed: bool = False
    landing_target_lost: bool = True
    landed: bool = False
    gripper_sensor_confirmed: bool = False
    failsafe_escalate_to: int = 0
    fc_connected: bool = False
    # Kenh NAMED_VALUE_INT cua FC; None = chua nhan / qua han = KHONG BIET (6.3 dong cuoi).
    ob_auth: bool = None
    ob_state: int = None
    arm_ready: bool = False
    disarm_ready: bool = False
    range_m: float = None        # laser, None khi ngoai dai / qua han


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

    def request_start(self, now_s):
        """Yeu cau cat canh (GCS / thu tren ban). Tra ly do tu choi, '' neu nhan."""
        if self.state != IDLE:
            return f'dang {self.state}, chi nhan khi IDLE'
        self.start_requested_s = now_s
        return ''

    def request_land(self):
        self.land_requested = True

    def transition(self, new_state, now_s, detail=''):
        """Chuyen trang thai co kiem tra bang TRANSITIONS - chan chuyen sai tu som."""
        if new_state not in TRANSITIONS[self.state]:
            raise ValueError(f'Chuyen trang thai khong hop le: {self.state} -> {new_state}')
        self.state = new_state
        self.state_entered_s = now_s
        self.retry_count = 0 if new_state != RETRY_LOITER else self.retry_count
        self.last_fc_command_s = None
        if new_state != IDLE:
            self.start_requested_s = None
        if new_state in (EMERGENCY_LAND, IDLE):
            self.land_requested = False
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

        TODO: than cac trang thai nhiem vu con lai theo dung ba nguyen tac o dau file:
          - failsafe_escalate_to != ESCALATE_NONE -> uu tien tuyet doi, sang FAILSAFE truoc moi thu;
          - IDLE: co waypoints -> bat dau nhu start_requested;
          - TAKEOFF: dat takeoff_alt_m -> ENROUTE, phat goto waypoint hien tai;
          - ENROUTE: at_waypoint -> MARKER_SEARCH, phat expected_marker_id cua waypoint;
          - MARKER_SEARCH: marker_confirmed -> PRECISION_LAND;
            qua search_timeout_s -> RETRY_LOITER, retry_count += 1;
            retry_count > max_retries -> bao failsafe (khong tu lap them);
          - PRECISION_LAND: landing_target_lost -> quay lai MARKER_SEARCH (khong ha mu);
            landed -> ACTUATE_GRIPPER;
          - ACTUATE_GRIPPER: phat gripper open/close theo waypoint.action, CHO
            gripper_sensor_confirmed moi cho roi diem; con waypoint -> TAKEOFF, het -> MISSION_COMPLETE.
        """
        now = snap.now_s
        lost = snap.ob_auth is False or snap.ob_state == OB_STATE_KHOA
        if snap.armed and lost and self.state != FAILSAFE and FAILSAFE in TRANSITIONS[self.state]:
            return self.transition(FAILSAFE, now, 'Pi mat quyen/KHOA khi dang arm - '
                                   'nguoi lai cat ch5 hoac ch8 xuong-len')

        if self.state == FAILSAFE:
            if not snap.armed:
                return self.transition(IDLE, now, 'da disarm')
            if snap.ob_auth is True and snap.ob_state != OB_STATE_KHOA:
                return self.transition(EMERGENCY_LAND, now, 'lay lai quyen khi con arm - ha canh')
            return Action(detail='mat quyen - cho nguoi lai')

        if (self.land_requested and snap.armed and self.state != EMERGENCY_LAND
                and EMERGENCY_LAND in TRANSITIONS[self.state]):
            return self.transition(EMERGENCY_LAND, now, 'yeu cau ha canh')

        if self.state == IDLE:
            return self._step_idle(snap)
        if self.state == TAKEOFF:
            return self._step_takeoff(snap)
        if self.state == EMERGENCY_LAND:
            return self._step_land(snap)
        if self.state == MISSION_COMPLETE:
            return self.transition(IDLE, now, 'xong')
        return Action(detail=f'{self.state}: chua hien thuc')

    def _step_idle(self, snap):
        now = snap.now_s
        self.land_requested = False
        if self.start_requested_s is None:
            return Action()
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

    def _step_takeoff(self, snap):
        if snap.ob_auth is not True:
            return Action(velocity_up_mps=0.0, detail='khong biet quyen - giu vz = 0')
        if snap.range_m is None:
            return Action(velocity_up_mps=0.0, detail='mat laser - giu vz = 0')
        if snap.range_m >= self.params.takeoff_alt_m:
            return Action(velocity_up_mps=0.0, detail='du do cao - giu')
        return Action(velocity_up_mps=TAKEOFF_CLIMB_MPS, detail='leo')

    def _step_land(self, snap):
        now = snap.now_s
        if not snap.armed:
            return self.transition(MISSION_COMPLETE, now, 'da cham dat va disarm')
        act = Action(velocity_up_mps=-LAND_DESCENT_MPS, detail='xuong')
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
