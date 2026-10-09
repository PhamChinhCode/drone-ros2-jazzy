"""Giu cho mac dinh tren Pi khi FSM khong ra lenh cho truc ngang / yaw - thuan Python, pytest duoc.

Khi Pi dieu khien (OFFBOARD), FC tat ca hai moc cua minh (ctrl_poshold.c: moc vi tri BAM THEO cho
hien tai; ctrl_angle.c: giu huong la ban chi khi !offb) - FC chi bam VAN TOC. Pi gui vx = vy = 0,
yaw_rate = 0 thi may bay TROI theo sai so van toc cua flow va gyro, khong ai keo ve. Hai lop duoi
day lam dung viec POSHOLD / giu huong cua FC lam khi nguoi lai tha can: ham truoc, du cham moi chot
moc, roi giu moc do. Ham truoc roi moi chot de khong vot qua moc roi bo nguoc ve.

Chi chot khi so do dang tin; mat dieu kien la THA moc (ve lenh 0 nhu cu) - khong bam mot con so co
the dang sai. So do nhay bac (EKF sua theo tag) cung tha moc: moc cu nam o mot cho vat ly khac.
"""

import math

POS_LOCK_SPEED_MPS = 0.2        # khop POSHOLD_LOCK_SPEED_MPS cua FC
POS_JUMP_M = 0.5                # vi tri EKF nhay qua muc nay trong 1 chu ky -> tha moc
YAW_LOCK_RATE = math.radians(10.0)    # khop HEADING_LOCK_RATE_DPS
YAW_BRAKE_TIMEOUT_S = 2.0             # khop HEADING_BRAKE_TIMEOUT_S
YAW_JUMP = math.radians(10.0)         # khop HEADING_JUMP_DEG
MIN_AIRBORNE_RANGE_M = 0.3      # laser duoi muc nay = dang nam dat (17 cm khi dap): khong giu


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


class PositionHold:
    """Moc vi tri ngang (x, y) trong khung odom."""

    def __init__(self):
        self.target = None
        self.prev = None

    def reset(self):
        self.target = None
        self.prev = None

    def update(self, pos_xy, speed_mps, allowed):
        """Tra moc (x, y) dang giu, hoac None (chua chot / khong duoc giu -> lenh van toc 0).

        pos_xy: vi tri EKF; speed_mps: toc do ngang; allowed: odom moi + da neo + dang bay + khong
        nguon nao khac ra lenh truc ngang.
        """
        if not allowed or pos_xy is None:
            self.reset()
            return None
        if self.prev is not None and math.dist(pos_xy, self.prev) > POS_JUMP_M:
            self.target = None
        self.prev = pos_xy
        if self.target is None and speed_mps < POS_LOCK_SPEED_MPS:
            self.target = pos_xy
        return self.target


class HeadingHold:
    """Moc huong (yaw ENU, rad)."""

    def __init__(self):
        self.target = None
        self.prev = None
        self.brake_s = 0.0

    def reset(self):
        self.target = None
        self.prev = None
        self.brake_s = 0.0

    def update(self, yaw, yaw_rate, allowed, dt):
        """Tra yaw dang giu, hoac None (dang ham / khong duoc giu -> yaw_rate 0)."""
        if not allowed or yaw is None:
            self.reset()
            return None
        if self.prev is not None and abs(wrap(yaw - self.prev)) > YAW_JUMP:
            self.target = None
            self.brake_s = 0.0
        self.prev = yaw
        if self.target is None:
            self.brake_s += dt
            if abs(yaw_rate) < YAW_LOCK_RATE or self.brake_s > YAW_BRAKE_TIMEOUT_S:
                self.target = yaw
        return self.target
