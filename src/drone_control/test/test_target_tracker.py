"""Kiem su kien bat duoc / mat bam cua landing_target_bridge_node."""

from drone_control.target_tracker import TargetTracker


def test_bat_duoc_mot_lan_mat_bam_mot_lan():
    t = TargetTracker(0.7)
    assert t.check(0.0) is None                 # chua tung thay: khong bao gi
    assert t.seen(1.0) == 'acquired'
    assert t.seen(1.04) is None
    assert t.check(1.5) is None
    assert t.check(1.8) == 'lost'
    assert t.check(2.5) is None                 # khong spam
    assert t.seen(3.0) == 'acquired'


def test_doi_id_khi_dang_bam_bao_mat():
    t = TargetTracker(0.7)
    assert t.reset() is None
    t.seen(1.0)
    assert t.reset() == 'lost'
    assert t.check(5.0) is None
