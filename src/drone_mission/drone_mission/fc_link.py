"""Trang thai OFFBOARD cua FC doc tu NAMED_VALUE_INT, thuan Python de pytest duoc.

Nguon duy nhat: docs/GIAO_UOC_FC_ROS2.md muc 6.2, 6.3, 9.3. FC phat moi ten 2 Hz; gia tri qua han
coi la KHONG BIET (None) - khong suy quyen tu RC_CHANNELS hay custom_mode (6.3 dong cuoi).
"""

# MAV_RESULT trong COMMAND_ACK (muc 6.2, 7).
MAV_RESULT_ACCEPTED = 0
MAV_RESULT_TEMPORARILY_REJECTED = 1
MAV_RESULT_DENIED = 2

MAV_CMD_COMPONENT_ARM_DISARM = 400
# param2 cua DISARM: cat dong co o moi do cao, bo qua cong 20 cm (11.1 #12f). CHI GCS goi.
DISARM_FORCE_MAGIC = 21196

OB_STATE_KHOA = 0
OB_STATE_TAT = 1
OB_STATE_DANG_CHAY = 2

OB_EXIT_NAMES = {
    0: '-', 1: 'KHOI_DONG', 2: 'CONG_TAC', 3: 'HET_HAN', 4: 'DAY_CAN', 5: 'DISARM',
    6: 'KEP_DAI', 7: 'MAT_GOC', 8: 'MAT_SONG', 9: 'CHE_DO',
}

# Bang bit OB_ARM_BLK (9.3). Co mot trong cac bit nay thi ARM se bi DENIED - can nguoi (6.3).
ARM_BLK_DENIED_MASK = 0x0001 | 0x0040 | 0x0080 | 0x0400 | 0x1000

# Hai chu ky phat 2 Hz + tre duong truyen.
DEFAULT_STALE_S = 1.5


class FcStatus:
    """Gia tri moi nhat cua tung ten NAMED_VALUE_INT kem thoi diem nhan."""

    def __init__(self, stale_s=DEFAULT_STALE_S):
        self.stale_s = stale_s
        self._values = {}

    def update(self, name, value, now_s):
        self._values[name] = (int(value), now_s)

    def get(self, name, now_s):
        """Gia tri con han, hoac None neu chua nhan / qua han."""
        item = self._values.get(name)
        if item is None or now_s - item[1] > self.stale_s:
            return None
        return item[0]

    def has_authority(self, now_s):
        """True/False theo OB_AUTH; None khi khong biet - KHONG duoc coi la co quyen."""
        auth = self.get('OB_AUTH', now_s)
        return None if auth is None else auth == 1

    def arm_ready(self, now_s):
        return self.get('OB_ARM_RDY', now_s) == 1

    def disarm_ready(self, now_s):
        return self.get('OB_DIS_RDY', now_s) == 1


def local_refusal(arm, armed, status, now_s):
    """Ly do Pi KHONG gui lenh ARM/DISARM thuong, hoac '' neu duoc gui.

    - Mat quyen hoac khong biet quyen -> khong gui lenh nao, ke ca DISARM (6.3), tru DISARM
      khi dang khong arm: FC luon tra ACCEPTED va khong lam gi (6.2).
    - ARM chi gui khi OB_ARM_RDY = 1 (6.3) - khong thu lai mu.
    """
    if not arm and armed is False:
        return ''
    auth = status.has_authority(now_s)
    if auth is None:
        return 'khong biet quyen: chua co OB_AUTH tu FC hoac da qua han'
    if not auth:
        return 'Pi mat quyen (OB_AUTH = 0) - cho nguoi lai gat ch8 xuong-len'
    if arm and not status.arm_ready(now_s):
        blk = status.get('OB_ARM_BLK', now_s)
        return f'FC chua san sang arm (OB_ARM_RDY != 1, OB_ARM_BLK = {blk})'
    return ''
