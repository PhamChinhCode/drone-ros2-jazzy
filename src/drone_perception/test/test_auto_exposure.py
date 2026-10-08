"""Kiem luat tu phoi sang, khong can camera."""

import numpy as np

from drone_perception.auto_exposure import ExposureLimits, image_stats, next_setting

LIM = ExposureLimits()


def test_trong_deadband_thi_giu_nguyen():
    assert next_setting(300, 32, 62.0, 0.0, LIM) is None


def test_anh_toi_tang_exposure_truoc_gain():
    """Toi gap doi -> exposure len, gain van thap nhat vi exposure chua cham tran."""
    exp, gain = next_setting(100, 16, 30.0, 0.0, LIM)
    assert exp == 200 and gain == 16


def test_exposure_cham_tran_thi_tang_gain():
    exp, gain = next_setting(500, 16, 30.0, 0.0, LIM)
    assert exp == LIM.exposure_max and gain == 32


def test_moi_lan_doi_toi_da_gap_doi():
    """Anh gan den (do 10-08: mean 6,4 voi 40/16) khong nhay vot mot buoc."""
    exp, gain = next_setting(40, 16, 6.4, 0.0, LIM)
    assert exp == 80 and gain == 16


def test_giam_sang_thi_gain_giam_truoc():
    exp, gain = next_setting(500, 64, 120.0, 0.0, LIM)
    assert exp == 500 and gain == 32


def test_chay_sang_thi_giam_du_mean_thap():
    """Nhieu diem chay (troi nang) phai giam du mean trung binh chua cao."""
    exp, gain = next_setting(300, 32, 50.0, 0.10, LIM)
    assert exp * gain < 300 * 32


def test_khong_vuot_gioi_han():
    assert next_setting(4, 16, 250.0, 0.5, LIM) is None
    assert next_setting(500, 128, 1.0, 0.0, LIM) is None


def test_image_stats():
    img = np.full((400, 640), 60, np.uint8)
    img[:40, :] = 255
    mean, sat = image_stats(img)
    assert abs(sat - 0.1) < 0.01
    assert abs(mean - (0.9 * 60 + 0.1 * 255)) < 1.0
