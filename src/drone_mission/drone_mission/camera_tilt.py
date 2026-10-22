"""Goc nghieng camera <-> xung servo, thuan Python (pytest duoc).

Servo camera ES08MD II tren FC (lenh 183 servo 2). Do 2026-10-22 bang tag 36h11 (docs/
ke_hoach_camera_servo_gap_hang.md muc 1):
  - tang us -> camera ngoc ve MUI; quy uoc o day: goc DUONG = ve mui, 0 = nhin thang xuong;
  - PHI tuyen (doc 0,073 -> 0,109 do/us) nen dung bang tra noi suy, khong dung mot he so;
  - ro ~1,9 do khi doi chieu tien toi; cung chieu thi lap lai <= 0,5 do -> luon tien toi dich tu
    phia us THAP (vuot xuong overshoot_us roi len), giong luc hieu chinh.
"""

import bisect
import math


class TiltTable:
    """Bang tra goc (do, + ve mui) <-> us, noi suy tuyen tinh tung doan, kep trong bang."""

    def __init__(self, tilt_deg, pulse_us):
        if len(tilt_deg) != len(pulse_us) or len(tilt_deg) < 2:
            raise ValueError('bang goc/xung phai cung do dai va >= 2 diem')
        pairs = sorted(zip(tilt_deg, pulse_us))
        self.deg = [float(d) for d, _ in pairs]
        self.us = [float(u) for _, u in pairs]
        if any(b <= a for a, b in zip(self.us, self.us[1:])):
            raise ValueError('xung phai tang dan theo goc (tang us -> ngoc ve mui)')

    def to_us(self, tilt_deg):
        return round(_interp(tilt_deg, self.deg, self.us))

    def to_deg(self, us):
        return _interp(us, self.us, self.deg)


def _interp(x, xs, ys):
    if x <= xs[0]:
        return ys[0]
    if x >= xs[-1]:
        return ys[-1]
    i = bisect.bisect_right(xs, x)
    t = (x - xs[i - 1]) / (xs[i] - xs[i - 1])
    return ys[i - 1] + t * (ys[i] - ys[i - 1])


def approach_moves(current_us, target_us, overshoot_us):
    """Chuoi xung gui de toi target_us tu phia us THAP (chong ro).

    current_us None (chua biet servo dang o dau) cung vuot xuong cho chac.
    """
    if current_us is not None and target_us >= current_us:
        return [target_us]
    return [target_us - overshoot_us, target_us]


def camera_link_pitch(tilt_deg):
    """Pitch (rad) cua base_link -> camera_link (quy uoc static_transform_publisher, x toi):
    90 do = nhin thang xuong; nghieng 20 do ve mui = 70 do (gia tri cung cu 1,2217)."""
    return math.radians(90.0 - tilt_deg)
