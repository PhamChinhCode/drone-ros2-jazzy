import math
import statistics

import pytest

from drone_sim.wind_model import GustyWind


def mau(w, n, dt=0.1):
    return [w.step(dt) for _ in range(n)]


def test_khong_gio_thi_bang_0():
    assert all(v == (0.0, 0.0) for v in mau(GustyWind(0.0, 30.0), 50))


def test_toc_do_trung_binh_va_do_giat():
    v = mau(GustyWind(5.0, 0.0, gust=0.35, seed=1), 20000)
    s = [math.hypot(*x) for x in v]
    assert statistics.mean(s) == pytest.approx(5.0, rel=0.05)
    assert statistics.pstdev(s) == pytest.approx(5.0 * 0.35, rel=0.15)
    assert min(s) >= 0.0


def test_huong_xoay_quanh_huong_chinh():
    v = mau(GustyWind(5.0, 90.0, dir_sigma_deg=25.0, seed=2), 20000)
    a = [math.degrees(math.atan2(y, x)) for x, y in v]
    assert statistics.mean(a) == pytest.approx(90.0, abs=3.0)
    assert 15.0 < statistics.pstdev(a) < 35.0


def test_giat_co_quan_tinh_khong_nhay_tung_buoc():
    """Lech giua hai mau 0,1 s nho hon nhieu so voi do giat - gio doi dan trong vai giay."""
    s = [math.hypot(*x) for x in mau(GustyWind(5.0, 0.0, seed=3), 5000)]
    buoc = statistics.pstdev([b - a for a, b in zip(s, s[1:])])
    assert buoc < 0.3 * statistics.pstdev(s)


def test_cung_seed_cung_ket_qua():
    assert mau(GustyWind(4.0, 10.0, seed=7), 30) == mau(GustyWind(4.0, 10.0, seed=7), 30)
