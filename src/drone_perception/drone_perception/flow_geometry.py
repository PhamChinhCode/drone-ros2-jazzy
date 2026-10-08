"""Doi cap diem optical flow (pixel) thanh van toc than may - hinh hoc thuan, pytest duoc.

Camera KHONG nhin thang xuong: lap truoc tam 90 mm, nghieng 20 do ve phia truoc (do 2026-10-08).
Cach cu (flow_px * do_cao / f, phat trong khung quang hoc) gia dinh nhin thang xuong nen:
bao thap van toc tien ~17 %, diem tren/duoi anh o do sau khac nhau (~1,6 lan) bi tron trung vi,
va xoay yaw 1 rad/s o 1 m sinh ~0,43 m/s van toc ngang gia (chua ke pitch/roll).

Cach moi: khu meo tung diem, chieu tia nhin xuong MAT DAT bang huong camera that tai DUNG luc
chup moi khung (IMU x TF lap dat), lay do dich camera tren mat dat, tru canh tay don khi quay.
Moi chuyen dong quay tu triet tieu vi ca hai khung deu chieu bang tu the cua chinh no.
Gia dinh: mat dat phang, nam ngang; do cao camera = do cao laser + do lech dung cua camera.
"""

import cv2
import numpy as np


def quat_to_matrix(x, y, z, w):
    """Ma tran quay tu quaternion (x, y, z, w) - khung con -> khung cha."""
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def pixel_rays(px, K, D, max_reproj_px=1.0):
    """Pixel anh THO (Nx2) -> (tia nhin khung quang hoc Nx3 z = 1, mask khu meo dang tin).

    Mo hinh meo hieu chuan (k1 ~ -0,29) chi don dieu toi ban kinh meo ~1,03, trong khi goc anh
    toi ~1,23: o do khu meo tra ve diem sai. Chieu nguoc lai ma lech > max_reproj_px thi bo.
    Lap 20 lan thay vi 5 mac dinh cua undistortPoints: do tren anh 640x400, 5 lan chi khu meo
    dung 72 % dien tich, 20 lan dung 90 % (10 % con lai la goc anh ngoai mo hinh that su).
    """
    px = np.asarray(px, np.float64).reshape(-1, 2)
    norm = cv2.undistortPointsIter(
        px.reshape(-1, 1, 2), K, D, None, None,
        (cv2.TERM_CRITERIA_COUNT | cv2.TERM_CRITERIA_EPS, 20, 1e-9)).reshape(-1, 2)
    rays = np.column_stack([norm, np.ones(len(norm))])
    back, _ = cv2.projectPoints(rays.reshape(-1, 1, 3), np.zeros(3), np.zeros(3), K, D)
    ok = np.linalg.norm(back.reshape(-1, 2) - px, axis=1) <= max_reproj_px
    return rays, ok


def ground_points(rays_opt, R_wo, height, max_ray_angle_deg):
    """Giao tia nhin voi mat dat nam ngang cach camera `height` m ben duoi.

    Tra (diem mat dat so voi camera trong khung the gioi Nx3, mask hop le). Tia gan chan troi
    (lech phuong thang dung > max_ray_angle_deg) bi loai: giao diem xa, sai so do cao phong dai.
    """
    rw = rays_opt @ R_wo.T
    down = -rw[:, 2]
    valid = down > np.cos(np.radians(max_ray_angle_deg)) * np.linalg.norm(rw, axis=1)
    scale = np.where(valid, height / np.where(valid, down, 1.0), 0.0)
    return rw * scale[:, None], valid


def base_velocity(old_px, new_px, K, D, R_bo, p_cam_b, R_wb0, R_wb1, h0, h1, dt, omega_b,
                  max_ray_angle_deg=65.0):
    """Van toc goc base_link trong khung base_link (FLU) tu mot cap khung.

    old_px/new_px: cung diem dac trung o khung truoc/sau (pixel anh tho).
    R_bo, p_cam_b: quay va vi tri base_link -> camera_optical_frame (TF lap dat).
    R_wb0/R_wb1: tu the base_link trong khung the gioi (IMU) luc chup khung truoc/sau.
    h0/h1: do cao CAMERA tren mat dat luc chup. omega_b: van toc goc than may (rad/s).
    Tra (v_base_b, so diem dung duoc) - v None neu khong con diem nao.
    """
    r0, u0 = pixel_rays(old_px, K, D)
    r1, u1 = pixel_rays(new_px, K, D)
    g0, ok0 = ground_points(r0, R_wb0 @ R_bo, h0, max_ray_angle_deg)
    g1, ok1 = ground_points(r1, R_wb1 @ R_bo, h1, max_ray_angle_deg)
    ok = u0 & u1 & ok0 & ok1
    n = int(np.count_nonzero(ok))
    if n == 0 or dt <= 0.0:
        return None, n
    # Diem mat dat dung yen: camera dich bao nhieu thi diem (nhin tu camera) lui bay nhieu.
    # median chong nhieu: vai diem bam nham khong keo lech ket qua.
    v_cam_w = np.median(g0[ok] - g1[ok], axis=0) / dt
    # Camera truoc tam 90 mm: quay quanh tam cung lam camera di chuyen (omega x r) - tru di.
    v_base_b = R_wb1.T @ v_cam_w - np.cross(omega_b, p_cam_b)
    return v_base_b, n
