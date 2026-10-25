"""Co cau kep MG90 cua drone_v2 trong mo phong - thuan Python, pytest duoc.

Hinh hoc lay tu drone_v2.urdf (3DModels/Version2.0/assembly_urdf, Assem1.SLDASM 10-25), khung
base_link, mat phang Y-Z (moi chot song song truc X). Moi ben la co cau 4 khau KIN: horn MG90 ->
thanh CauNoi -> tai ham GapHang -> chot ham tren GiaPin. URDF cat vong o chot tai ham va cho 4
joint bi dong <mimic> tuyen tinh (sai toi ~10 do); o day giai CHINH XAC cho Gazebo.

Goc servo: 0 = tu the CAD = DONG het (diem chet, khe ngon 119,2 mm om hop 120 mm), -180 do = MO HET
co (het hanh trinh MG90, diem chet thu hai, hai ham 56 do, day ngon cao hon chan 45 mm) - user
10-25: khong kep thi mo het co cho ngon khong cham dat.
"""

import math

H = (0.0, -0.08462)                       # tam horn (y, z)
SIDES = {
    # ben: (lo horn, tai ham, chot ham) o goc 0
    'left': ((-0.00927, -0.08469), (0.03453, -0.08434), (0.03151, -0.0625)),
    'right': ((0.00927, -0.08454), (-0.03452, -0.08434), (-0.0315, -0.0625)),
}
OPEN_RAD = math.radians(-180.0)
CLOSED_RAD = 0.0
SERVO_RATE = 10.5                         # rad/s, MG90 0,10 s/60 do (URDF)

# Ngon khi DONG (lay tu mesh URDF): day ngon, mat trong, be rong theo X.
TIP_Z = -0.1928
INNER_Y = 0.0596
FINGER_HALF_X = 0.02


def _rot(v, a):
    c, s = math.cos(a), math.sin(a)
    return (c * v[0] - s * v[1], s * v[0] + c * v[1])


def _sub(a, b):
    return (a[0] - b[0], a[1] - b[1])


def _cross(a, b):
    return a[0] * b[1] - a[1] * b[0]


def _ang(v):
    return math.atan2(v[1], v[0])


def _wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


def four_bar(side, servo):
    """(goc ham, goc thanh so voi horn) rad tai goc servo, cung nhanh voi tu the CAD."""
    h0, e0, p = SIDES[side]
    L = math.dist(e0, h0)
    R = math.dist(e0, p)
    h = tuple(H[i] + _rot(_sub(h0, H), servo)[i] for i in range(2))
    d = _sub(p, h)
    dist = math.hypot(*d)
    a = (L * L - R * R + dist * dist) / (2 * dist)
    u = (d[0] / dist, d[1] / dist)
    k = math.sqrt(max(L * L - a * a, 0.0))
    side_cad = math.copysign(1.0, _cross(_sub(p, h0), _sub(e0, h0)))
    for sgn in (1.0, -1.0):
        e = (h[0] + a * u[0] - sgn * k * u[1], h[1] + a * u[1] + sgn * k * u[0])
        if math.copysign(1.0, _cross(d, _sub(e, h))) == side_cad:
            break
    jaw = _wrap(_ang(_sub(e, p)) - _ang(_sub(e0, p)))
    # Thanh quay qua 180 do khi mo het co: giu goc lien tuc trong [-90, 270) do.
    rod = _ang(_sub(e, h)) - _ang(_sub(e0, h0)) - servo
    rod = (rod + math.pi / 2) % (2 * math.pi) - math.pi / 2
    return jaw, rod


def joint_targets(servo):
    """Goc moi joint (ten theo URDF) tai goc servo."""
    jl, rl = four_bar('left', servo)
    jr, rr = four_bar('right', servo)
    return {'gripper_servo_joint': servo, 'gripper_rod_left_joint': rl,
            'gripper_rod_right_joint': rr, 'gripper_jaw_left_joint': jl,
            'gripper_jaw_right_joint': jr}


def step_servo(current, target, dt):
    """Servo chay toi dich voi toc do toi da SERVO_RATE."""
    step = SERVO_RATE * dt
    return current + max(-step, min(step, target - current))


def box_in_jaws(box_body, box_yaw_rel, box_half, box_height):
    """Hop co nam giua hai ham de kep duoc khong.

    box_body: tam DAY hop (x, y, z) trong khung base_link; box_yaw_rel: goc hop so voi mui (rad).
    Kep duoc khi: mat hop song song ham (+-20 do, hop vuong nen theo 90 do), tam hop lot giua hai
    ngon dang mo (|y| <= nua khe mo tru nua hop), ngon nam tron tren mat hop theo X, va day ngon
    luc dong nam trong chieu cao mat ben hop (cach day va nap >= 1 cm).
    """
    x, y, z = box_body
    yaw = abs(_wrap(4 * box_yaw_rel)) / 4
    open_half = INNER_Y + 0.061           # khe o -120 do (242 mm) - mo het co con rong hon
    return (yaw <= math.radians(20)
            and abs(y) <= open_half - box_half
            and abs(x) <= box_half - FINGER_HALF_X
            and z + 0.01 <= TIP_Z <= z + box_height - 0.01)
