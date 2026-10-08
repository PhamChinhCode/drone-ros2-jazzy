"""Kiem flow_geometry bang canh tong hop: san phang, camera lap that (90 mm, nghieng 20 do).

Chieu diem san qua hai tu the da biet -> cap pixel -> base_velocity phai tra lai dung van toc.
"""

import cv2
import numpy as np
import pytest

from drone_perception.flow_geometry import base_velocity, quat_to_matrix

# Noi tai that cua OV9281 640x400 (camera_info tren Pi 5, 2026-10-08).
K = np.array([[299.94, 0.0, 321.58], [0.0, 298.75, 180.21], [0.0, 0.0, 1.0]])
D0 = np.zeros(5)
D_THAT = np.array([-0.2906, 0.07802, 0.000241, -0.000844, -0.008976])   # plumb_bob tren Pi 5
P_CAM_B = np.array([0.09, 0.0, 0.0])
DT = 0.1


def rot(yaw=0.0, pitch=0.0, roll=0.0):
    """Rz(yaw) Ry(pitch) Rx(roll) - quy uoc static_transform_publisher (x y z yaw pitch roll)."""
    cy, sy, cp, sp, cr, sr = (np.cos(yaw), np.sin(yaw), np.cos(pitch), np.sin(pitch),
                              np.cos(roll), np.sin(roll))
    rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    return rz @ ry @ rx


# Dung chuoi TF trong estimation.launch.py: base -> camera_link (pitch 70 do) -> optical.
R_BO = rot(pitch=1.2217) @ rot(yaw=-np.pi / 2, roll=-np.pi / 2)


def chup(points_w, base_pos, R_wb, D):
    """Pixel anh tho cua cac diem san + mask nam trong khung hinh."""
    cam = base_pos + R_wb @ P_CAM_B
    X = (points_w - cam) @ (R_wb @ R_BO)
    px, _ = cv2.projectPoints(X.reshape(-1, 1, 3), np.zeros(3), np.zeros(3), K, D)
    px = px.reshape(-1, 2)
    # Chi diem trong vung mo hinh meo con don dieu (ban kinh chuan hoa < 1,6): xa hon thi
    # da thuc meo 'gap' diem ngoai tam nhin vao trong anh - ong kinh that khong lam vay.
    r = np.linalg.norm(X[:, :2], axis=1) / np.maximum(X[:, 2], 1e-9)
    ok = ((X[:, 2] > 0.1) & (r < 1.6) & (px[:, 0] >= 0) & (px[:, 0] < 640)
          & (px[:, 1] >= 0) & (px[:, 1] < 400))
    return px, ok


def chay(pos0, R_wb0, pos1, R_wb1, omega_b, D=D0):
    rng = np.random.default_rng(1)
    pts = np.column_stack([rng.uniform(-6, 6, 4000), rng.uniform(-6, 6, 4000), np.zeros(4000)])
    px0, ok0 = chup(pts, pos0, R_wb0, D)
    px1, ok1 = chup(pts, pos1, R_wb1, D)
    ok = ok0 & ok1
    h0 = (pos0 + R_wb0 @ P_CAM_B)[2]
    h1 = (pos1 + R_wb1 @ P_CAM_B)[2]
    return base_velocity(px0[ok], px1[ok], K, D, R_BO, P_CAM_B, R_wb0, R_wb1, h0, h1, DT,
                         np.asarray(omega_b, float))


def test_tf_lap_dat_nhin_xuong_lech_20_do_ve_truoc():
    truc = R_BO @ [0, 0, 1]
    np.testing.assert_allclose(truc, [np.sin(np.radians(20)), 0, -np.cos(np.radians(20))],
                               atol=1e-3)


@pytest.mark.parametrize('v_w', [(1.0, 0, 0), (0, 0.5, 0), (-0.7, 0.4, 0)])
def test_van_toc_ngang_dung_khi_bay_bang(v_w):
    """Cach cu bao thap van toc tien ~17 % - cach moi phai dung."""
    v_w = np.array(v_w)
    p0 = np.array([0.0, 0.0, 1.0])
    v, n = chay(p0, rot(), p0 + v_w * DT, rot(), [0, 0, 0])
    assert n >= 50
    np.testing.assert_allclose(v, v_w, atol=0.01)


def test_van_toc_ra_khung_than_khi_dang_quay_huong():
    """Huong mui 0,7 rad: van toc the gioi phai doi sang khung than (FLU)."""
    R = rot(yaw=0.7)
    v_w = np.array([0.6, -0.3, 0.0])
    p0 = np.array([1.0, 2.0, 1.5])
    v, _ = chay(p0, R, p0 + v_w * DT, R, [0, 0, 0])
    np.testing.assert_allclose(v, R.T @ v_w, atol=0.01)


def test_xoay_yaw_tai_cho_khong_sinh_van_toc_gia():
    """Cach cu: yaw 1 rad/s o 1 m sinh ~0,43 m/s ngang gia. Tam than dung yen -> ~0."""
    p0 = np.array([0.0, 0.0, 1.0])
    v, _ = chay(p0, rot(), p0, rot(yaw=1.0 * DT), [0, 0, 1.0])
    np.testing.assert_allclose(v[:2], [0, 0], atol=0.01)


@pytest.mark.parametrize('roll,pitch', [(0.15, 0), (0, 0.15), (0.1, -0.1)])
def test_lac_roll_pitch_tai_cho_khong_sinh_van_toc_gia(roll, pitch):
    p0 = np.array([0.0, 0.0, 1.2])
    omega = np.array([roll, pitch, 0]) / DT
    v, _ = chay(p0, rot(), p0, rot(pitch=pitch, roll=roll), omega)
    np.testing.assert_allclose(v[:2], [0, 0], atol=0.02)


def test_nghieng_va_leo_cung_luc():
    """Than nghieng 10 do (dang tang toc), leo 0,3 m/s, tien 1 m/s o 3 m."""
    R = rot(pitch=np.radians(10), roll=np.radians(-5))
    v_w = np.array([1.0, 0.2, 0.3])
    p0 = np.array([0.0, 0.0, 3.0])
    v, _ = chay(p0, R, p0 + v_w * DT, R, [0, 0, 0])
    np.testing.assert_allclose(v, R.T @ v_w, atol=0.02)


def test_anh_tho_co_meo_ong_kinh():
    """Diem pixel lay tu anh THO co meo: phai khu meo truoc khi chieu xuong dat."""
    D = D_THAT
    v_w = np.array([0.8, 0.3, 0.0])
    p0 = np.array([0.0, 0.0, 1.0])
    v, _ = chay(p0, rot(), p0 + v_w * DT, rot(), [0, 0, 0], D)
    np.testing.assert_allclose(v, v_w, atol=0.01)


def test_ti_le_theo_do_cao():
    """Cung do dich pixel, do cao gap doi -> van toc gap doi (nhu cach cu, nhung dung he so)."""
    v1, _ = chay(np.array([0, 0, 1.0]), rot(), np.array([0.05, 0, 1.0]), rot(), [0, 0, 0])
    v2, _ = chay(np.array([0, 0, 2.0]), rot(), np.array([0.10, 0, 2.0]), rot(), [0, 0, 0])
    np.testing.assert_allclose(v2, 2 * v1, rtol=0.02)


def test_bo_diem_goc_anh_ngoai_vung_mo_hinh_meo():
    """Goc anh (ban kinh meo ~1,23 > ~1,03 toi da cua mo hinh) khong khu meo dung duoc -> bo."""
    from drone_perception.flow_geometry import pixel_rays
    _, ok = pixel_rays(np.array([[321.6, 180.2], [600.0, 300.0], [2.0, 2.0], [638.0, 398.0]]),
                       K, D_THAT)
    assert ok.tolist() == [True, True, False, False]


def test_khong_con_tia_hop_le_thi_khong_publish():
    px = np.array([[320.0, 180.0], [100.0, 50.0]])
    v, n = base_velocity(px, px, K, D0, R_BO, P_CAM_B, rot(), rot(), 1.0, 1.0, DT,
                         np.zeros(3), max_ray_angle_deg=1.0)
    assert v is None and n == 0


def test_quat_to_matrix_khop_rot():
    q = (0.0, np.sin(0.35), 0.0, np.cos(0.35))     # pitch 0,7 rad
    np.testing.assert_allclose(quat_to_matrix(*q), rot(pitch=0.7), atol=1e-9)
