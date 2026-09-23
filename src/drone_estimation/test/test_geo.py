"""Kiem doi toa do WGS84 <-> khung ban do tag va cong loc mau GPS."""

import math

import pytest

from drone_estimation.geo import (GeoOrigin, gps_measurement, lla_to_map, map_to_lla,
                                  origin_from_params)

HA_NOI = GeoOrigin(21.028511, 105.804817, 15.0)


def test_goc_la_diem_0():
    assert lla_to_map(HA_NOI.lat_deg, HA_NOI.lon_deg, HA_NOI.alt_m, HA_NOI) == \
        pytest.approx((0.0, 0.0, 0.0), abs=1e-6)


def test_mot_giay_vi_do_ve_phia_bac():
    # 1" vi do o vi do 21,03: ban kinh cong kinh tuyen M = a(1-e2)/(1-e2 sin2)^1.5 = 6 343 640 m,
    # nhan 4,8481e-6 rad = 30,755 m (tinh tay doc lap voi code).
    x, y, z = lla_to_map(HA_NOI.lat_deg + 1 / 3600, HA_NOI.lon_deg, HA_NOI.alt_m, HA_NOI)
    assert x == pytest.approx(0.0, abs=1e-3)
    assert y == pytest.approx(30.755, abs=0.005)
    assert z == pytest.approx(0.0, abs=1e-3)


def test_mot_giay_kinh_do_ve_phia_dong():
    # 1" kinh do ~ 30,92 * cos(21,03) = 28,87 m.
    x, y, _ = lla_to_map(HA_NOI.lat_deg, HA_NOI.lon_deg + 1 / 3600, HA_NOI.alt_m, HA_NOI)
    assert x == pytest.approx(28.87, abs=0.03)
    assert y == pytest.approx(0.0, abs=1e-3)


def test_khu_hoi_1_km():
    for x, y, z in [(1000.0, -250.0, 12.0), (-3.2, 7.9, -1.5), (0.0, 0.0, 30.0)]:
        lat, lon, alt = map_to_lla(x, y, z, HA_NOI)
        assert lla_to_map(lat, lon, alt, HA_NOI) == pytest.approx((x, y, z), abs=1e-4)


def test_ban_do_xoay_90_do():
    # Truc N ban do chi ve phia Dong that: di 10 m ve Dong that la +10 m theo y ban do.
    o = GeoOrigin(HA_NOI.lat_deg, HA_NOI.lon_deg, HA_NOI.alt_m, north_yaw_deg=90.0)
    lat, lon, alt = map_to_lla(0.0, 10.0, 0.0, o)
    east_that, bac_that, _ = lla_to_map(lat, lon, alt, HA_NOI)
    assert (east_that, bac_that) == pytest.approx((10.0, 0.0), abs=1e-3)


def test_ban_do_xoay_nho_giu_khoang_cach():
    o = GeoOrigin(10.77, 106.70, 5.0, north_yaw_deg=-17.5)
    x, y, _ = lla_to_map(10.7705, 106.7003, 5.0, o)
    x0, y0, _ = lla_to_map(10.7705, 106.7003, 5.0, GeoOrigin(10.77, 106.70, 5.0))
    assert math.hypot(x, y) == pytest.approx(math.hypot(x0, y0), abs=1e-6)


def test_goc_chua_khai():
    assert origin_from_params(False, 0.0, 0.0, 0.0, 0.0) is None
    assert origin_from_params(True, 21.0, 105.0, 3.0, 1.5) == GeoOrigin(21.0, 105.0, 3.0, 1.5)
    with pytest.raises(ValueError):
        origin_from_params(True, 121.0, 105.0, 0.0, 0.0)


def test_cong_loc_gps():
    assert gps_measurement(3, 1.2, 14, 3, 3.0, 6) == (True, '')
    assert gps_measurement(1, 1.2, 14, 3, 3.0, 6)[0] is False     # chua fix
    assert gps_measurement(3, 4.5, 14, 3, 3.0, 6)[0] is False     # sai so qua lon
    assert gps_measurement(3, 0.0, 14, 3, 3.0, 6)[0] is False     # 0 = khong biet
    assert gps_measurement(3, float('nan'), 14, 3, 3.0, 6)[0] is False
    assert gps_measurement(4, 0.8, 5, 3, 3.0, 6)[0] is False      # it ve tinh
