"""Mo phong tiep can co QUAN TINH + THAN NGHIENG theo gia toc, kiem tag con trong khung camera.

Camera gan cung than (90 mm truoc tam, nghieng 20 do): tang toc toi thi chuc mui (thay ra sau nhieu
hon), ham thi ngoc mui (thay ra sau it di - ngoc 5 do la tag to mat tu 1,08 m thay vi 0,74 m), sua
trai phai thi nghieng ngang. Mo phong dong hoc o test_approach_guidance bo qua viec nay; o day moi
buoc chieu 4 goc tag to va tag nho vao anh 640x400 that (K tu camera_info, TF lap dat).
"""

import math

import pytest

from drone_mission.approach_guidance import ApproachParams, guide, wrap

P = ApproachParams()
PAD = (5.0, 3.0, 0.0)
G = 9.81
FX, FY, CX, CY, W, H = 299.94, 298.75, 321.58, 180.21, 640, 400
P_CAM_B = (0.09, 0.0, 0.0)
BIG, SMALL, SMALL_FWD = 0.25, 0.10, 0.21
MIN_PX = 20.0               # canh tag toi thieu de apriltag doc chac (decimate 2)
MARGIN_PX = 4.0


def mat_mul(a, b):
    return [[sum(a[i][k] * b[k][j] for k in range(3)) for j in range(3)] for i in range(3)]


def rot(yaw=0.0, pitch=0.0, roll=0.0):
    cy, sy, cp, sp, cr, sr = (math.cos(yaw), math.sin(yaw), math.cos(pitch), math.sin(pitch),
                              math.cos(roll), math.sin(roll))
    rz = [[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]]
    ry = [[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]]
    rx = [[1, 0, 0], [0, cr, -sr], [0, sr, cr]]
    return mat_mul(mat_mul(rz, ry), rx)


R_BO = mat_mul(rot(pitch=1.2217), rot(yaw=-math.pi / 2, roll=-math.pi / 2))   # = estimation.launch


def apply(r, v):
    return [sum(r[i][k] * v[k] for k in range(3)) for i in range(3)]


def apply_t(r, v):
    return [sum(r[k][i] * v[k] for k in range(3)) for i in range(3)]


def tag_visible(pos, r_wb, center, size, yaw_pad):
    """Ca 4 goc tag nam trong anh (co bien) va canh >= MIN_PX."""
    cam = [pos[i] + apply(r_wb, P_CAM_B)[i] for i in range(3)]
    r_wo = mat_mul(r_wb, R_BO)
    ux, uy = math.cos(yaw_pad), math.sin(yaw_pad)
    px = []
    for a, b in ((1, 1), (1, -1), (-1, -1), (-1, 1)):
        c = (center[0] + size / 2 * (a * ux - b * uy), center[1] + size / 2 * (a * uy + b * ux),
             center[2])
        x = apply_t(r_wo, [c[i] - cam[i] for i in range(3)])
        if x[2] <= 0.05:
            return False
        u, v = CX + FX * x[0] / x[2], CY + FY * x[1] / x[2]
        if not (MARGIN_PX <= u <= W - MARGIN_PX and MARGIN_PX <= v <= H - MARGIN_PX):
            return False
        px.append((u, v))
    return math.hypot(px[0][0] - px[1][0], px[0][1] - px[1][1]) >= MIN_PX


def mo_phong(start, yaw0, yaw_pad, vmax=0.5, kp=1.0, tau=0.4, amax=2.0,
             rate=math.radians(30), dt=0.02, t_max=150.0):
    x, y, z = start
    vx = vy = vz = 0.0
    yaw = yaw0
    ux, uy = math.cos(yaw_pad), math.sin(yaw_pad)
    small_c = (PAD[0] + SMALL_FWD * ux, PAD[1] + SMALL_FWD * uy, PAD[2])
    vet = []
    t = 0.0
    while t < t_max:
        g = guide((x, y, z), yaw, PAD, yaw_pad)
        if g.phase == 'arrived':
            return vet
        # Dieu khien vi tri P -> lenh van toc (nhu position_controller), FC bam co quan tinh.
        cx, cy = kp * (g.target[0] - x), kp * (g.target[1] - y)
        n = math.hypot(cx, cy)
        if n > vmax:
            cx, cy = cx * vmax / n, cy * vmax / n
        cz = max(-vmax, min(vmax, kp * (g.target[2] - z)))
        ax, ay, az = (cx - vx) / tau, (cy - vy) / tau, (cz - vz) / tau
        na = math.hypot(ax, ay)
        if na > amax:
            ax, ay = ax * amax / na, ay * amax / na
        vx, vy, vz = vx + ax * dt, vy + ay * dt, vz + az * dt
        x, y, z = x + vx * dt, y + vy * dt, z + vz * dt
        yaw = wrap(yaw + max(-rate * dt, min(rate * dt, wrap(g.yaw - yaw))))
        # Than nghieng theo gia toc ngang: toi -> chuc mui (pitch +), trai -> roll am (FLU).
        a_fwd = ax * math.cos(yaw) + ay * math.sin(yaw)
        a_left = -ax * math.sin(yaw) + ay * math.cos(yaw)
        pitch, roll = math.atan2(a_fwd, G), -math.atan2(a_left, G)
        r_wb = rot(yaw, pitch, roll)
        vet.append({'phase': g.phase, 'pos': (x, y, z), 'pitch': math.degrees(pitch),
                    'big': tag_visible((x, y, z), r_wb, PAD, BIG, yaw_pad),
                    'small': tag_visible((x, y, z), r_wb, small_c, SMALL, yaw_pad)})
        t += dt
    raise AssertionError(f'khong toi noi sau {t_max} s')


def test_kiem_tra_tam_nhin_dung_voi_so_tinh_tay():
    """Than phang, ngay tren tam bai: tag to con thay o 0,8 m, mat o 0,6 m (tinh tay ~0,74 m)."""
    yp = math.radians(90)
    r = rot(yp, 0.0, 0.0)
    assert tag_visible((PAD[0], PAD[1], 0.8), r, PAD, BIG, yp)
    assert not tag_visible((PAD[0], PAD[1], 0.6), r, PAD, BIG, yp)
    # Ngoc mui 5 do (ham): o 0,9 m da mat tag to (tinh tay mat tu 1,08 m).
    assert not tag_visible((PAD[0], PAD[1], 0.9), rot(yp, math.radians(-5), 0.0), PAD, BIG, yp)


@pytest.mark.parametrize('start,yaw0', [
    ((5.0, -6.0, 5.0), math.radians(90)),
    ((-2.0, -2.0, 5.0), math.radians(0)),
    ((9.0, 4.0, 3.0), math.radians(180)),
    ((5.0, 9.0, 4.0), math.radians(-90)),
])
def test_luon_thay_tag_suot_pha_final_khi_than_nghieng(start, yaw0):
    vet = mo_phong(start, yaw0, math.radians(90))
    final = [s for s in vet if s['phase'] == 'final']
    assert final, 'phai di qua pha final'
    mat = [s for s in final if not (s['big'] or s['small'])]
    nghieng = max(abs(s['pitch']) for s in final)
    assert not mat, (f'{len(mat)}/{len(final)} buoc final khong thay tag nao; vd {mat[0]["pos"]}, '
                     f'pitch {mat[0]["pitch"]:.1f} do; nghieng lon nhat {nghieng:.1f} do')
