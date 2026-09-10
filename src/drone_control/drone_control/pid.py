"""Vong PID thuan, tach khoi ROS de pytest duoc.

Ten truong dung dung quy uoc pid_gains_t phia FC de de doi chieu khi tune hai tang.
"""


def clamp(value, low, high):
    return max(low, min(high, value))


class PID:

    def __init__(self, kp, ki, kd, i_limit, out_limit):
        self.kp, self.ki, self.kd = kp, ki, kd
        self.i_limit, self.out_limit = i_limit, out_limit
        self.integral = 0.0
        self.prev_error = 0.0

    def reset(self):
        self.integral = 0.0
        self.prev_error = 0.0

    def update(self, error, dt):
        self.integral = clamp(self.integral + error * dt, -self.i_limit, self.i_limit)  # anti-windup
        derivative = (error - self.prev_error) / dt if dt > 0 else 0.0
        self.prev_error = error
        output = self.kp * error + self.ki * self.integral + self.kd * derivative
        return clamp(output, -self.out_limit, self.out_limit)
