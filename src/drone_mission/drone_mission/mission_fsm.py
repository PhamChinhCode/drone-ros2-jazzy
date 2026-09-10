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

    fc_command: str = ''       # '', 'arm', 'takeoff', 'goto', 'hold', 'precision_land', 'rth'...
    gripper_command: str = ''  # '', 'open', 'close'
    expected_marker_id: int = -1
    detail: str = ''


@dataclass
class MissionFsm:
    params: Params = field(default_factory=Params)
    state: str = IDLE
    waypoints: list = field(default_factory=list)
    current_wp_index: int = 0
    retry_count: int = 0
    state_entered_s: float = 0.0

    def transition(self, new_state, now_s, detail=''):
        """Chuyen trang thai co kiem tra bang TRANSITIONS - chan chuyen sai tu som."""
        if new_state not in TRANSITIONS[self.state]:
            raise ValueError(f'Chuyen trang thai khong hop le: {self.state} -> {new_state}')
        self.state = new_state
        self.state_entered_s = now_s
        self.retry_count = 0 if new_state != RETRY_LOITER else self.retry_count
        return Action(detail=detail)

    def time_in_state(self, now_s):
        return now_s - self.state_entered_s

    def step(self, snap):
        """Chay mot chu ky FSM, tra ve Action.

        TODO: hien thuc than tung trang thai theo dung ba nguyen tac o dau file:
          - failsafe_escalate_to != ESCALATE_NONE -> uu tien tuyet doi, sang FAILSAFE truoc moi thu;
          - IDLE: co waypoints + fc_connected -> arm roi TAKEOFF;
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
        raise NotImplementedError('than FSM chua duoc hien thuc')
