"""Tia ToF MTF-01 trong mo phong: cat mat dat va nap hop hang - thuan Python, pytest duoc.

Laser gia cu (sim_fc.range_m) chi lay do cao than + hang so nen luon "thay dat". Hop hang 12 cm nam
ngay duoi duoi drone luc gap, ma ToF lap 0,10 m sau tam (URDF drone_v2) - can biet laser doc gi
khi vet sang quet qua nap hop.

Chum 6 do (MTF-01_USER_MANUAL: Distance FOV 6 deg) lay mau bang 1 tia tam + N_RIM tia tren vien non
3 do; moi tia lay mat gan nhat (dat z = 0 hoac nap hop). Ket qua = TRUNG VI cac tia: hon nua vet sang
tren nap hop thi so doc nhay sang khoang cach toi nap - gan dung ToF mot vung bao dinh tin hieu manh
nhat. Bo qua mat ben hop (chum hep, gan thang dung).
"""

import math
import statistics

CONE_HALF_DEG = 3.0
N_RIM = 8


def quat_to_mat(x, y, z, w):
    return ((1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)),
            (2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)),
            (2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)))


def _mul(r, v):
    return tuple(sum(r[i][k] * v[k] for k in range(3)) for i in range(3))


def _body_rays():
    a = math.radians(CONE_HALF_DEG)
    rays = [(0.0, 0.0, -1.0)]
    for k in range(N_RIM):
        phi = 2 * math.pi * k / N_RIM
        rays.append((math.sin(a) * math.cos(phi), math.sin(a) * math.sin(phi), -math.cos(a)))
    return rays


RAYS = _body_rays()


def tof_range(p_base, r_wb, tof_xyz, boxes):
    """Khoang cach ToF doc duoc (m), None neu tia khong cham mat nao.

    p_base: vi tri base_link trong the gioi; r_wb: ma tran xoay than -> the gioi; tof_xyz: ToF
    trong base_link; boxes: [(x, y, yaw, half, height)] - tam hop tren dat, nua canh, cao nap.
    """
    o = tuple(p_base[i] + _mul(r_wb, tof_xyz)[i] for i in range(3))
    dist = []
    for ray in RAYS:
        d = _mul(r_wb, ray)
        if d[2] >= -1e-6:
            continue                                  # tia khong huong xuong
        best = o[2] / -d[2]                           # mat dat z = 0
        for bx, by, yaw, half, h in boxes:
            t = (o[2] - h) / -d[2]
            if t <= 0 or t >= best:
                continue
            hx, hy = o[0] + t * d[0] - bx, o[1] + t * d[1] - by
            c, s = math.cos(yaw), math.sin(yaw)
            if abs(c * hx + s * hy) <= half and abs(-s * hx + c * hy) <= half:
                best = t
        dist.append(best)
    return statistics.median(dist) if dist else None
