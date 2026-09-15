"""Kiem chi so dap ung buoc."""

import pytest

from drone_sim.step_response import analyze


def test_bac_mot_khong_vuot():
    # 0 -> 2 m tuyen tinh trong 2 s roi dung yen.
    samples = [(i * 0.1, min(2.0, i * 0.1)) for i in range(51)]
    r = analyze(samples, 0.0, 2.0, 0.1)
    assert r['rise_s'] == pytest.approx(1.6)        # 0,2 m -> 1,8 m
    assert r['overshoot_pct'] == 0.0
    assert r['settle_s'] == pytest.approx(1.9)
    assert r['final_error_m'] == pytest.approx(0.0)


def test_vuot_dich_va_vao_dai_lan_cuoi():
    samples = [(0.0, 0.0), (1.0, 2.5), (2.0, 1.95), (3.0, 1.7), (4.0, 2.05), (5.0, 2.0)]
    r = analyze(samples, 0.0, 2.0, 0.1)
    assert r['overshoot_pct'] == pytest.approx(25.0)
    assert r['settle_s'] == pytest.approx(4.0)       # ra dai luc 3,0 nen tinh lai tu 4,0


def test_chua_toi_dich_va_buoc_am():
    r = analyze([(0.0, 2.0), (1.0, 1.5), (2.0, 1.4)], 2.0, 0.0, 0.1)
    assert r['rise_s'] is None and r['settle_s'] is None
    assert r['final_error_m'] == pytest.approx(1.4)
    with pytest.raises(ValueError):
        analyze([], 0.0, 1.0, 0.1)
