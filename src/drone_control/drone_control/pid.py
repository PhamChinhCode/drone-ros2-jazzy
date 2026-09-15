"""Vong PID thuan, tach khoi ROS de pytest duoc.

Ten truong dung dung quy uoc pid_gains_t phia FC de de doi chieu khi tune hai tang.
"""

import math

GAIN_FIELDS = ('kp', 'ki', 'kd', 'i_limit', 'out_limit')


def clamp(value, low, high):
    return max(low, min(high, value))


class PID:

    def __init__(self, kp, ki, kd, i_limit, out_limit):
        self.kp, self.ki, self.kd = kp, ki, kd
        self.i_limit, self.out_limit = i_limit, out_limit
        self.integral = 0.0
        self.prev_error = 0.0

    def reset(self, error=0.0):
        """Xoa tich phan; prev_error = sai so hien tai de lan update dau khong co cu giat D."""
        self.integral = 0.0
        self.prev_error = error

    def update(self, error, dt):
        self.integral = clamp(self.integral + error * dt, -self.i_limit, self.i_limit)  # anti-windup
        derivative = (error - self.prev_error) / dt if dt > 0 else 0.0
        self.prev_error = error
        output = self.kp * error + self.ki * self.integral + self.kd * derivative
        return clamp(output, -self.out_limit, self.out_limit)


def check_gain_param(pids, name, value):
    """Kiem tham so '<pha>.<truc>.<truong>' (vd cruise.x.kp). Tra (pid, truong, '') neu hop le,
    (None, None, '') neu khong phai tham so gain, hoac (None, None, ly do) neu gia tri sai."""
    parts = name.split('.')
    if len(parts) != 3 or parts[0] not in pids or parts[1] not in pids[parts[0]] \
            or parts[2] not in GAIN_FIELDS:
        return None, None, ''
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
        return None, None, f'{name} phai la so thuc huu han'
    if value < 0.0:
        return None, None, f'{name} = {value} phai >= 0'
    return pids[parts[0]][parts[1]], parts[2], ''
