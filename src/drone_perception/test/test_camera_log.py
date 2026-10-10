"""Kiem nen anh camera cho bag: dung kich thuoc, bo phan dem cuoi hang (step > width)."""

import cv2
import numpy as np

from drone_perception.camera_log import encode_jpeg


def test_nen_roi_giai_nen_dung_kich_thuoc_va_noi_dung():
    img = np.zeros((40, 64), np.uint8)
    img[10:30, 20:44] = 255
    jpg = encode_jpeg(64, 40, 64, img.tobytes(), 90)
    back = cv2.imdecode(np.frombuffer(jpg, np.uint8), cv2.IMREAD_GRAYSCALE)
    assert back.shape == (40, 64)
    assert abs(int(back[20, 32]) - 255) < 10 and int(back[2, 2]) < 10


def test_bo_dem_cuoi_hang():
    step = 80                                   # moi hang 64 byte anh + 16 byte dem
    buf = np.full((40, step), 255, np.uint8)
    buf[:, :64] = 0
    back = cv2.imdecode(np.frombuffer(encode_jpeg(64, 40, step, buf.tobytes(), 90), np.uint8),
                        cv2.IMREAD_GRAYSCALE)
    assert back.shape == (40, 64) and int(back.max()) < 10
