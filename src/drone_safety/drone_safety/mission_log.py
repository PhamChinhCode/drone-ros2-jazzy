"""Ghi log nghiep vu cua mission_logger_node, thuan Python de pytest duoc khong can ROS.

Mot thu muc moi mission_id: <log_root>/<mission_id>/events.jsonl (mot dong JSON moi su kien,
ghi noi tiep) va photos/<seq>_<event>.jpg. Chi ghi khi co SU KIEN (doi trang thai, failsafe bat /
het, bat / mat tag, gripper xac nhan doi) - khong ghi moi chu ky.
"""

import datetime
import json
import os

import cv2


class MissionLog:

    def __init__(self, log_root):
        self.log_root = os.path.expanduser(log_root)
        self.mission_id = None
        self.dir = None
        self.last_state = None
        self.gripper_confirmed = None
        self.photo_seq = 0

    def on_mission_state(self, now_s, mission_id, state, wp_index, retry_count,
                         expected_marker_id, detail):
        """Mission moi -> mo thu muc; trang thai doi -> STATE_CHANGE. Tra list su kien da ghi."""
        written = []
        if mission_id != self.mission_id:
            self._open(mission_id)
            written.append(self.write(now_s, 'MISSION_OPEN', {'state': state, 'detail': detail}))
            self.last_state = state
        if state != self.last_state:
            written.append(self.write(now_s, 'STATE_CHANGE', {
                'from': self.last_state, 'to': state, 'wp_index': wp_index,
                'retry_count': retry_count, 'expected_marker_id': expected_marker_id,
                'detail': detail}))
            self.last_state = state
        return written

    def on_failsafe(self, now_s, fs_type, escalate_to, active, detail):
        return self.write(now_s, 'FAILSAFE', {'type': fs_type, 'escalate_to': escalate_to,
                                              'active': active, 'detail': detail})

    def on_target_lost(self, now_s, lost, expected_marker_id):
        event = 'MARKER_LOST' if lost else 'MARKER_MATCH'
        return self.write(now_s, event, {'marker_id': expected_marker_id})

    def on_gripper(self, now_s, confirmed, state, force_n, image):
        """sensor_confirmed doi -> GRIP_CONFIRMED / GRIP_RELEASED kem anh (image = mang numpy
        mono8 hoac None). Lan dau nhan chi ghi nho moc, khong ghi su kien."""
        if self.gripper_confirmed is None or confirmed == self.gripper_confirmed:
            self.gripper_confirmed = confirmed
            return None
        self.gripper_confirmed = confirmed
        event = 'GRIP_CONFIRMED' if confirmed else 'GRIP_RELEASED'
        payload = {'gripper_state': state, 'force_n': force_n, 'photo': None}
        if image is not None and self.dir is not None:
            self.photo_seq += 1
            rel = os.path.join('photos', f'{self.photo_seq:03d}_{event}.jpg')
            if cv2.imwrite(os.path.join(self.dir, rel), image):
                payload['photo'] = rel
        return self.write(now_s, event, payload)

    def write(self, now_s, event_type, payload):
        """Ghi mot dong; chua co mission_id (chua nhan /mission/state) thi bo. Tra dict da ghi."""
        if self.dir is None:
            return None
        record = {'stamp': now_s,
                  'time': datetime.datetime.fromtimestamp(now_s).isoformat(timespec='milliseconds'),
                  'mission_id': self.mission_id, 'event_type': event_type, **payload}
        with open(os.path.join(self.dir, 'events.jsonl'), 'a', encoding='utf-8') as f:
            f.write(json.dumps(record, ensure_ascii=False) + '\n')
            f.flush()
            # Su kien thua (vai dong moi chuyen bay): fsync de mat dien giua chung khong mat dong.
            os.fsync(f.fileno())
        return record

    def _open(self, mission_id):
        self.mission_id = mission_id
        self.dir = os.path.join(self.log_root, str(mission_id))
        os.makedirs(os.path.join(self.dir, 'photos'), exist_ok=True)
        # Cung mission_id bay lai (vd ke hoach thu tren ban): noi tiep file, anh khong ghi de.
        self.photo_seq = len(os.listdir(os.path.join(self.dir, 'photos')))
