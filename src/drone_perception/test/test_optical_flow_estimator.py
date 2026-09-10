"""Kiem thuat toan optical flow bang anh tu sinh, khong can ROS lan camera that."""

import cv2
import numpy as np
import pytest

from drone_perception.optical_flow_estimator import OpticalFlowEstimator

FOCAL_PX = 323.62      # dung fx that cua camera da hieu chinh
DT = 0.02              # 50 Hz
ALT = 2.0              # met


def anh_co_dac_trung(w=640, h=400, seed=0):
    """Anh xam co nhieu goc ro rang de goodFeaturesToTrack bam duoc."""
    rng = np.random.default_rng(seed)
    img = np.zeros((h, w), dtype=np.uint8)
    for _ in range(60):
        x, y = rng.integers(30, w - 40), rng.integers(30, h - 40)
        cv2.rectangle(img, (x, y), (x + 12, y + 12), 255, -1)
    return img


def dich(img, dx, dy):
    M = np.float32([[1, 0, dx], [0, 1, dy]])
    return cv2.warpAffine(img, M, (img.shape[1], img.shape[0]))


def test_khung_dau_tien_khong_publish():
    """Chua co khung truoc thi khong the tinh flow - phai tra None."""
    est = OpticalFlowEstimator(FOCAL_PX)
    vel, n = est.process(anh_co_dac_trung(), DT, ALT)
    assert vel is None
    assert n == 0


def test_dt_khong_hop_le():
    est = OpticalFlowEstimator(FOCAL_PX)
    assert est.process(anh_co_dac_trung(), 0.0, ALT) == (None, 0)
    assert est.process(anh_co_dac_trung(), -0.01, ALT) == (None, 0)


@pytest.mark.parametrize('dx,dy', [(6, 0), (0, 5), (4, -3), (-7, 2)])
def test_van_toc_khop_quang_dich_da_biet(dx, dy):
    """Dich anh dung dx,dy pixel -> van toc phai bang dx*ALT/FOCAL/DT."""
    est = OpticalFlowEstimator(FOCAL_PX)
    a = anh_co_dac_trung()
    est.process(a, DT, ALT)                       # khung dau: chi phat hien dac trung
    vel, n = est.process(dich(a, dx, dy), DT, ALT)

    assert vel is not None, 'phai co ket qua o khung thu hai'
    assert n >= 8
    mong_doi = np.array([dx, dy]) * ALT / FOCAL_PX / DT
    np.testing.assert_allclose(vel, mong_doi, atol=0.05)


def test_duoi_nguong_dac_trung_thi_khong_publish():
    """Anh trong -> khong bam duoc dac trung -> KHONG publish, va reset de bat dau lai."""
    est = OpticalFlowEstimator(FOCAL_PX, min_tracked_features=8)
    est.process(anh_co_dac_trung(), DT, ALT)
    vel, n = est.process(np.zeros((400, 640), dtype=np.uint8), DT, ALT)
    assert vel is None
    assert n < 8
    assert est.prev_gray is None, 'phai reset de lan sau phat hien lai tu dau'


def test_van_toc_ti_le_thuan_do_cao():
    """Cung mot flow pixel, bay cao gap doi -> van toc that gap doi."""
    a = anh_co_dac_trung()
    b = dich(a, 5, 0)

    e1 = OpticalFlowEstimator(FOCAL_PX)
    e1.process(a, DT, 1.0)
    v1, _ = e1.process(b, DT, 1.0)

    e2 = OpticalFlowEstimator(FOCAL_PX)
    e2.process(a, DT, 2.0)
    v2, _ = e2.process(b, DT, 2.0)

    np.testing.assert_allclose(v2, v1 * 2.0, rtol=1e-6)
