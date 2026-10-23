"""Bu laser khi vet sang qua nap hop 12 cm (box_range)."""

import pytest

from drone_estimation.box_range import BoxRange

DT = 0.05   # laser 20 Hz


def chay(b, ds, t0=0.0):
    return [b.update(d, t0 + i * DT) for i, d in enumerate(ds)]


def test_tat_thi_giu_nguyen():
    b = BoxRange()
    assert b.update(0.48, 0.0) == (0.48, True)
    assert b.update(None, 0.05) == (None, False)


def test_vet_sang_qua_nap_hop_khi_lui_ve_hop():
    b = BoxRange()
    b.set_box(0.12)
    out = chay(b, [0.80] * 5 + [0.68] * 10 + [0.80] * 5)
    assert all(g == pytest.approx(0.80) for g, ok in out) and all(ok for _, ok in out)


def test_ha_doc_van_theo_duoc_va_qua_nap_van_dung():
    b = BoxRange()
    b.set_box(0.12)
    ds = [0.80 - 0.005 * i for i in range(40)]                 # xuong 0,10 m/s
    ds = [d - 0.12 if 15 <= i < 25 else d for i, d in enumerate(ds)]
    out = chay(b, ds)
    for i, (g, _) in enumerate(out):
        assert g == pytest.approx(0.80 - 0.005 * i)


def test_so_doc_vat_mep_khong_keo_lech_ground():
    b = BoxRange()
    b.set_box(0.12)
    out = chay(b, [0.30] * 3 + [0.25, 0.18, 0.18, 0.25, 0.30])     # 0,25 = nua vet sang tren nap
    assert out[4][0] == pytest.approx(0.30) and out[5][0] == pytest.approx(0.30)
    assert out[-1][0] == pytest.approx(0.30)


def test_mat_laser_sat_nap_thi_giu_ground_khong_bao_mat():
    b = BoxRange()
    b.set_box(0.12)
    chay(b, [0.20] * 3)
    assert b.update(None, 1.0) == (pytest.approx(0.20), False) and b.on_box


def test_mat_laser_khi_con_cao_van_la_mat():
    b = BoxRange()
    b.set_box(0.12)
    chay(b, [0.80] * 3)
    assert b.update(None, 1.0) == (None, False)


def test_doi_pha_thi_quen_trang_thai():
    b = BoxRange()
    b.set_box(0.12)
    chay(b, [0.80, 0.68])
    b.set_box(0.0)
    assert b.update(0.68, 1.0) == (0.68, True)
