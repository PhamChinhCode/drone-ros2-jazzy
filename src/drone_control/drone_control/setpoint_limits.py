"""Tu kep setpoint van toc TRUOC khi gui, thuan Python - GIAO_UOC_FC_ROS2.md 9.6, no P9.

FC kep 20 setpoint lien tiep la KHOA (OB_EXIT = 6, Pi mat quyen). Pi kep o ~95 % tran de khong
bao gio cham co kep toc do. Xuong cham sat dat (laser <= 1,2 m -> FC kep 0,3 m/s) la kep bao,
khong vao KEP_DAI, nhung Pi van tu kep de OB_RX_CLP chi tang khi co loi that.

Tat ca theo FLU (x toi, y trai, z len, yaw_rate duong = quay trai), rad/s cho yaw.
"""

import math

from drone_control.pid import clamp

FC_MAX_VEL_MPS = 2.0            # offboard_max_vel_mps
FC_MAX_CLIMB_MPS = 1.0          # offboard_max_climb_mps
FC_MAX_YAW_DPS = 90.0           # offboard_max_yaw_dps
FC_SLOW_DESCENT_MPS = 0.3       # xuong cham sat dat
FC_SLOW_DESCENT_BELOW_M = 1.2   # do cao uoc luong (laser) bat dau xuong cham
MARGIN = 0.95


def limit_velocity(vx, vy, vz, yaw_rate, range_m):
    """Kep (vx, vy, vz, yaw_rate). range_m = None (khong co laser) -> coi nhu sat dat."""
    h = FC_MAX_VEL_MPS * MARGIN
    down = FC_MAX_CLIMB_MPS
    if range_m is None or range_m <= FC_SLOW_DESCENT_BELOW_M:
        down = FC_SLOW_DESCENT_MPS
    yaw = math.radians(FC_MAX_YAW_DPS) * MARGIN
    return (clamp(vx, -h, h),
            clamp(vy, -h, h),
            clamp(vz, -down * MARGIN, FC_MAX_CLIMB_MPS * MARGIN),
            clamp(yaw_rate, -yaw, yaw))
