"""He "than phang" (base_level): goc va huong mui cua base_link, KHONG nghieng theo roll/pitch.

Camera gan cung vao than: than nghieng (tang toc, ham, sua trai phai) thi vecto toi tag bieu dien
trong base_link xoay theo, nen tag NGAY DUOI cung bi coi la lech ngang - h * sin(nghieng): 1 m cao,
nghieng 5 do -> 9 cm, 10 do -> 17 cm (do 2026-10-08). Bo dieu khien ha canh sua theo do lech gia do
-> lai nghieng -> vong lap xau. Bieu dien trong he chi xoay theo yaw thi do lech ngang la that.
"""

import math


def quat_mul(a, b):
    """Tich quaternion (x, y, z, w)."""
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw,
            aw * bw - ax * bx - ay * by - az * bz)


def rotate(q, v):
    """Xoay vecto v bang quaternion q (x, y, z, w)."""
    x, y, z, w = quat_mul(quat_mul(q, (v[0], v[1], v[2], 0.0)), (-q[0], -q[1], -q[2], q[3]))
    return (x, y, z)


def yaw_of(q):
    x, y, z, w = q
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def tilt_only(q_wb):
    """Phan roll/pitch cua tu the than: q_level_body = Rz(-yaw) * q_wb."""
    h = -0.5 * yaw_of(q_wb)
    return quat_mul((0.0, 0.0, math.sin(h), math.cos(h)), q_wb)


def to_level(q_wb, pos_body, quat_body):
    """(vi tri, huong) bieu dien trong base_link -> trong base_level.

    q_wb: tu the base_link trong khung ban do (EKF, tu IMU) - chi dung roll/pitch cua no.
    """
    q = tilt_only(q_wb)
    return rotate(q, pos_body), quat_mul(q, quat_body)


def small_to_pad_center(pos_small, yaw_body, yaw_pad, forward_m):
    """Vi tri tag NHO trong base_level -> vi tri TAM BAI (tag to) trong base_level.

    Tag nho nam forward_m ve phia "tren" cua bai (yaw_pad, ENU) - tren than phang phia do lech
    yaw_pad - yaw_body so voi mui. Dung huong bai khai bao + yaw EKF (khong dung huong tag uoc
    luong tu anh): sai yaw 10 do chi lech tam ~4 cm voi forward 0,22 m.
    """
    d = yaw_pad - yaw_body
    return (pos_small[0] - forward_m * math.cos(d), pos_small[1] - forward_m * math.sin(d),
            pos_small[2])
