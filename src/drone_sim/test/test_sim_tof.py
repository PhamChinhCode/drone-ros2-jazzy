"""Kiem mo hinh tia ToF: dat phang, vet sang tren / vat mep / ngoai nap hop, than nghieng."""

import math

import pytest

from drone_sim.sim_tof import quat_to_mat, tof_range

I = quat_to_mat(0.0, 0.0, 0.0, 1.0)
TOF = (-0.10, 0.0, -0.009)                 # URDF drone_v2
HOP = [(0.0, 0.0, 0.0, 0.06, 0.12)]        # hop 12 cm tai goc


def test_dat_phang_bang_do_cao_tof():
    assert tof_range((0.0, 0.0, 0.209), I, TOF, []) == pytest.approx(0.20, abs=1e-3)


def test_drone_dung_tam_hop_tof_van_thay_dat():
    # ToF 0,10 m sau tam, nua hop 0,06 m, vet sang ban kinh ~1 cm -> con khe ~3 cm.
    assert tof_range((0.0, 0.0, 0.209), I, TOF, HOP) == pytest.approx(0.20, abs=1e-3)


def test_lech_ve_duoi_5_cm_thi_tof_doc_nap_hop():
    # ToF tai x = -0,05: vet sang (ban kinh ~4 mm o 8 cm) nam tron tren nap -> doc ngan 12 cm.
    assert tof_range((0.05, 0.0, 0.209), I, TOF, HOP) == pytest.approx(0.08, abs=1e-3)


def test_vat_mep_doi_khi_hon_nua_vet_sang_tren_nap():
    # ToF tai x = -0,06 + e: tam vet sang ngay mep hop.
    assert tof_range((0.04 - 0.005, 0.0, 0.609), I, TOF, HOP) == pytest.approx(0.60, abs=0.01)
    assert tof_range((0.04 + 0.005, 0.0, 0.609), I, TOF, HOP) == pytest.approx(0.48, abs=0.01)


def test_hop_xoay_theo_huong_bai():
    goc = [(0.0, 0.0, math.radians(45), 0.06, 0.12)]       # duong cheo 0,085 m doc truc x
    assert tof_range((0.03, 0.0, 0.209), I, TOF, goc) == pytest.approx(0.08, abs=1e-3)


def test_than_nghieng_tia_xien():
    q = (0.0, math.sin(math.radians(10) / 2), 0.0, math.cos(math.radians(10) / 2))   # chuc mui 10 do
    r = tof_range((0.0, 0.0, 1.009), quat_to_mat(*q), (0.0, 0.0, -0.009), [])
    assert r == pytest.approx(1.0 / math.cos(math.radians(10)), rel=0.01)
