"""Bu troi ngang (khau I dung chung) - thuan Python, pytest duoc.

Moi vong vi tri cua Pi (cruise, giu cho, bam tag) chi co khau P. Gio hoac sai so van toc cua FC
(flow lech, FC tuong da bam dung lenh) day may bay deu mot huong -> khau P de lai mot do lech
CO DINH = toc_do_troi / kp: troi 6 cm/s voi landing kp 0,5 -> ha lech ~12 cm (Gazebo 10-09).

Khau I rieng tung PID khong lam duoc viec nay: tich phan bi xoa moi lan doi nguon (MARKER_SEARCH ->
PRECISION_LAND, mat tag roi thay lai), va doan bam tag chi ~5 s - khong kip tich. Ma troi la tinh
chat cua MAY BAY + MOI TRUONG, khong phu thuoc dang bam gi. Nen: MOT uoc luong van toc troi, tich tu
sai so vi tri cua BAT KY nguon nao dang lai truc ngang, KHONG xoa khi doi nguon, cong vao lenh.

Luu trong he ban do (ENU): gio co huong co dinh trong the gioi, quay mui khong lam mat uoc luong.
Chi tich khi da gan dich (|sai so| <= window_m) - dang bay chang dai sai so lon la do chua toi noi,
khong phai do troi. Xoa khi nam dat (moi lan cat canh tu dau).
"""

import math


class DriftCompensator:

    def __init__(self, ki=0.15, limit_mps=0.3, window_m=0.5):
        self.ki, self.limit_mps, self.window_m = ki, limit_mps, window_m
        self.bias = (0.0, 0.0)          # van toc bu, he ban do (m/s)

    def reset(self):
        self.bias = (0.0, 0.0)

    def update(self, err_world, dt):
        """err_world: sai so vi tri (dich - hien tai) he ban do, m. Tra bias moi (m/s, he ban do)."""
        if math.hypot(*err_world) <= self.window_m:
            bx = self.bias[0] + self.ki * err_world[0] * dt
            by = self.bias[1] + self.ki * err_world[1] * dt
            n = math.hypot(bx, by)
            if n > self.limit_mps:                  # kep theo DO LON, khong tung truc
                bx, by = bx * self.limit_mps / n, by * self.limit_mps / n
            self.bias = (bx, by)
        return self.bias


def closing_speed(err, vel):
    """Toc do tien VE dich (m/s, > 0 = dang lai gan): hinh chieu van toc len huong sai so.

    err, vel cung mot he. Sai so qua nho (< 2 cm) thi huong vo nghia - tra 0.
    """
    n = math.hypot(*err)
    return (err[0] * vel[0] + err[1] * vel[1]) / n if n > 0.02 else 0.0


def world_to_body(v, yaw):
    """(x, y) he ban do -> (toi, trai) he than theo yaw."""
    c, s = math.cos(yaw), math.sin(yaw)
    return (c * v[0] + s * v[1], -s * v[0] + c * v[1])


def body_to_world(v, yaw):
    c, s = math.cos(yaw), math.sin(yaw)
    return (c * v[0] - s * v[1], s * v[0] + c * v[1])
