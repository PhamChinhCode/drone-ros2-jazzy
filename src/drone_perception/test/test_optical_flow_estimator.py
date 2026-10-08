"""Kiem thuat toan optical flow bang anh tu sinh, khong can ROS lan camera that."""

import cv2
import numpy as np
import pytest

from drone_perception.optical_flow_estimator import OpticalFlowEstimator


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
    """Chua co khung truoc thi khong the bam - phai tra None."""
    est = OpticalFlowEstimator()
    pairs, n = est.track(anh_co_dac_trung())
    assert pairs is None
    assert n == 0


@pytest.mark.parametrize('dx,dy', [(6, 0), (0, 5), (4, -3), (-7, 2)])
def test_cap_diem_khop_quang_dich_da_biet(dx, dy):
    """Dich anh dung dx,dy pixel -> moi cap diem bam duoc phai lech dung dx,dy."""
    est = OpticalFlowEstimator()
    a = anh_co_dac_trung()
    est.track(a)                                  # khung dau: chi phat hien dac trung
    pairs, n = est.track(dich(a, dx, dy))

    assert pairs is not None, 'phai co ket qua o khung thu hai'
    assert n >= 8
    old, new = pairs
    np.testing.assert_allclose(np.median(new - old, axis=0), [dx, dy], atol=0.05)


def test_duoi_nguong_dac_trung_thi_khong_publish():
    """Anh trong -> khong bam duoc dac trung -> KHONG publish, va reset de bat dau lai."""
    est = OpticalFlowEstimator(min_tracked_features=8)
    est.track(anh_co_dac_trung())
    pairs, n = est.track(np.zeros((400, 640), dtype=np.uint8))
    assert pairs is None
    assert n < 8
    assert est.prev_gray is None, 'phai reset de lan sau phat hien lai tu dau'
