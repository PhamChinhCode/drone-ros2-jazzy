"""Lenh GCS 42101 DRONE_RESTART_STACK (giao uoc GCS <-> Pi muc 4.1, bo sung 2026-10-09).

Quyet dinh thuan (pytest duoc): chi khoi dong lai khi CHAC drone dang nam yen - KHONG arm va FSM o
IDLE. Khong biet trang thai arm (mat duong FC) cung tu choi: khoi dong lai stack giua khong trung
la cat moi lenh Pi gui FC, FC het han 500 ms roi tu phanh - chap nhan duoc khi treo, khong chap nhan
khi khong biet.

Cach khoi dong lai khong can sudo: scripts/drone_startup.sh xuat DRONE_STACK_PID (pid cua chinh no)
cho moi node, nhan SIGUSR1 thi tat ros2 launch sach (bag dong file) roi thoat ma 75 ->
drone-startup.service (Restart=on-failure) chay lai sau RestartSec.
"""

MISSION_IDLE = 0


def quyet_dinh(armed, mission_state, stack_pid):
    """armed: True/False/None (khong biet). stack_pid: chuoi bien moi truong DRONE_STACK_PID.

    Tra ('ok' | 'tu_choi' | 'khong_ho_tro', ly do).
    """
    if not stack_pid or not stack_pid.isdigit():
        return 'khong_ho_tro', 'stack khong chay duoi drone-startup.service (chay tay)'
    if armed is None:
        return 'tu_choi', 'chua biet trang thai arm (mat duong FC)'
    if armed:
        return 'tu_choi', 'drone dang ARM'
    if mission_state != MISSION_IDLE:
        return 'tu_choi', f'mission_state {mission_state}, chi khoi dong lai khi IDLE'
    return 'ok', ''
