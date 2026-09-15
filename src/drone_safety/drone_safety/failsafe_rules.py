"""Quy tac failsafe, thuan Python de pytest duoc: dau vao hien tai -> tap su co dang bat.

Hang so trung drone_interfaces/FailsafeEvent (test kiem khop). Nguong PHAI khop system_config
phia GCS.
Gia tri None = khong biet -> KHONG tu suy ra su co (vd pin -1 % tu FC la "khong biet", 4.3).
"""

from dataclasses import dataclass

FS_MARKER_TIMEOUT = 1
FS_GRIP_CONFIRM_FAIL = 2
FS_LINK_LOST = 3
FS_LOW_BATTERY = 4
FS_EKF_UNHEALTHY = 5
FS_FC_COMM_LOST = 6

ESCALATE_NONE = 0
ESCALATE_LOITER = 1
ESCALATE_RETRY_LOITER = 2
ESCALATE_RTH = 3
ESCALATE_EMERGENCY_LAND = 4

# Chua tung nhan /mavros/state: cho DDS noi va MAVROS phat (1 Hz) truoc khi coi la mat FC.
# Do 09-14: khong co an han thi moi lan khoi dong bao gia "mat FC 3,1 s" roi het sau 1 s.
FC_STARTUP_GRACE_S = 10.0


@dataclass
class Limits:
    low_battery_pct: float = 25.0
    critical_battery_pct: float = 15.0
    link_lost_timeout_s: float = 10.0
    marker_search_timeout_s: float = 20.0
    fc_comm_timeout_s: float = 3.0
    grip_confirm_timeout_s: float = 5.0


@dataclass
class Inputs:
    now_s: float
    node_start_s: float
    battery_pct: float = None
    ekf_healthy: bool = None
    fc_heartbeat_s: float = None      # lan cuoi /mavros/state connected = true
    gcs_disconnected_since_s: float = None
    mission_state: str = None         # ten trang thai MissionState, None = chua nhan
    state_entered_s: float = None
    gripper_confirmed: bool = None


def evaluate(inp, lim):
    """Tra {fs_type: (escalate_to, detail)} cua cac su co dang dung o thoi diem inp.now_s."""
    out = {}
    now = inp.now_s

    if inp.battery_pct is not None:
        if inp.battery_pct < lim.critical_battery_pct:
            detail = f'pin {inp.battery_pct:.0f} % < {lim.critical_battery_pct:.0f} %'
            out[FS_LOW_BATTERY] = (ESCALATE_EMERGENCY_LAND, detail)
        elif inp.battery_pct < lim.low_battery_pct:
            out[FS_LOW_BATTERY] = (ESCALATE_RTH,
                                   f'pin {inp.battery_pct:.0f} % < {lim.low_battery_pct:.0f} %')

    if inp.ekf_healthy is False:
        out[FS_EKF_UNHEALTHY] = (ESCALATE_LOITER, 'EKF khong healthy')

    # Chua tung thay FC thi tinh tu luc node khoi dong + an han: FC khong len la khong bay duoc.
    if inp.fc_heartbeat_s is not None:
        silent = now - inp.fc_heartbeat_s
        limit = lim.fc_comm_timeout_s
    else:
        silent = now - inp.node_start_s
        limit = max(lim.fc_comm_timeout_s, FC_STARTUP_GRACE_S)
    if silent > limit:
        out[FS_FC_COMM_LOST] = (ESCALATE_EMERGENCY_LAND, f'mat FC {silent:.1f} s')

    if (inp.gcs_disconnected_since_s is not None
            and now - inp.gcs_disconnected_since_s > lim.link_lost_timeout_s):
        out[FS_LINK_LOST] = (ESCALATE_RTH,
                             f'mat GCS {now - inp.gcs_disconnected_since_s:.0f} s')

    in_state = None if inp.state_entered_s is None else now - inp.state_entered_s
    if inp.mission_state == 'MARKER_SEARCH' and in_state > lim.marker_search_timeout_s:
        out[FS_MARKER_TIMEOUT] = (ESCALATE_RETRY_LOITER, f'tim marker {in_state:.0f} s')
    # Timeout o day chi de PHAT HIEN su co, khong thay xac nhan cam bien (nguyen tac 3 FSM).
    if (inp.mission_state == 'ACTUATE_GRIPPER' and inp.gripper_confirmed is not True
            and in_state > lim.grip_confirm_timeout_s):
        out[FS_GRIP_CONFIRM_FAIL] = (ESCALATE_RETRY_LOITER,
                                     f'gripper chua xac nhan sau {in_state:.0f} s')
    return out
