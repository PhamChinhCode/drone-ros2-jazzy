"""Kiem log nghiep vu: chi ghi su kien, dung thu muc theo mission_id, anh xac nhan gripper."""

import json
import os

import numpy as np

from drone_safety.mission_log import MissionLog


def doc(tmp_path, mission_id):
    with open(tmp_path / str(mission_id) / 'events.jsonl', encoding='utf-8') as f:
        return [json.loads(line) for line in f]


def state(log, t, mission_id, s, detail=''):
    return log.on_mission_state(t, mission_id, s, 0, 0, -1, detail)


def test_chi_ghi_khi_doi_trang_thai(tmp_path):
    log = MissionLog(str(tmp_path))
    state(log, 1.0, 5, 'IDLE', 'nhan ke hoach')
    state(log, 1.2, 5, 'IDLE', 'nhan ke hoach')
    state(log, 1.4, 5, 'TAKEOFF', 'da arm')
    state(log, 1.6, 5, 'TAKEOFF', 'leo')
    ev = doc(tmp_path, 5)
    assert [e['event_type'] for e in ev] == ['MISSION_OPEN', 'STATE_CHANGE']
    assert ev[1]['from'] == 'IDLE' and ev[1]['to'] == 'TAKEOFF' and ev[1]['detail'] == 'da arm'
    assert ev[1]['mission_id'] == 5 and ev[1]['stamp'] == 1.4 and ev[1]['time']


def test_mission_moi_sang_thu_muc_moi(tmp_path):
    log = MissionLog(str(tmp_path))
    state(log, 1.0, 1, 'IDLE')
    state(log, 2.0, 2, 'IDLE')
    assert len(doc(tmp_path, 1)) == 1 and len(doc(tmp_path, 2)) == 1


def test_chua_co_mission_state_thi_bo_su_kien(tmp_path):
    log = MissionLog(str(tmp_path))
    assert log.on_failsafe(1.0, 5, 1, True, 'EKF') is None
    assert os.listdir(tmp_path) == []


def test_failsafe_va_marker(tmp_path):
    log = MissionLog(str(tmp_path))
    state(log, 1.0, 3, 'MARKER_SEARCH')
    log.on_target_lost(2.0, False, 1)
    log.on_target_lost(3.0, True, 1)
    log.on_failsafe(4.0, 1, 2, True, 'tim marker 21 s')
    ev = doc(tmp_path, 3)[1:]
    assert [e['event_type'] for e in ev] == ['MARKER_MATCH', 'MARKER_LOST', 'FAILSAFE']
    assert ev[0]['marker_id'] == 1 and ev[2]['escalate_to'] == 2 and ev[2]['active'] is True


def test_gripper_doi_xac_nhan_kem_anh(tmp_path):
    log = MissionLog(str(tmp_path))
    state(log, 1.0, 4, 'ACTUATE_GRIPPER')
    img = np.full((40, 64), 128, dtype=np.uint8)
    assert log.on_gripper(1.1, False, 0, 0.0, img) is None        # lan dau: chi lay moc
    assert log.on_gripper(1.2, False, 0, 0.0, img) is None
    rec = log.on_gripper(1.3, True, 1, 2.5, img)
    assert rec['event_type'] == 'GRIP_CONFIRMED' and rec['force_n'] == 2.5
    assert os.path.isfile(tmp_path / '4' / rec['photo'])
    rec = log.on_gripper(1.4, False, 0, 0.0, None)                # khong co anh moi
    assert rec['event_type'] == 'GRIP_RELEASED' and rec['photo'] is None


def test_bay_lai_cung_mission_id_khong_ghi_de_anh(tmp_path):
    img = np.zeros((8, 8), dtype=np.uint8)
    for _ in range(2):
        log = MissionLog(str(tmp_path))
        state(log, 1.0, 9, 'ACTUATE_GRIPPER')
        log.on_gripper(1.1, False, 0, 0.0, img)
        log.on_gripper(1.2, True, 1, 0.0, img)
    assert len(os.listdir(tmp_path / '9' / 'photos')) == 2
    assert len(doc(tmp_path, 9)) == 4
