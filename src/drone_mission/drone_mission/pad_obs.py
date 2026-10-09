"""Loc tu the bai DO tu tag (/landing_target/pad_odom) - thuan Python, pytest duoc.

Bai dung yen nen trung binh truot (EMA) la du; yaw trung binh tren vong tron. Mau xa vi tri bai
khai bao qua max_offset_m bi loai: thuong la ban tin con sot cua bai cu ngay sau khi doi diem, hoac
tag nhan nham. Huong thi KHONG loc theo huong khai bao - chinh muc dich la bai quay bat ky huong nao.
"""

import math


class PadObsFilter:

    def __init__(self, alpha=0.3, max_offset_m=3.0):
        self.alpha, self.max_offset_m = alpha, max_offset_m
        self.reset()

    def reset(self):
        self.pos = None
        self.c = self.s = 0.0           # trung binh cos/sin cua yaw

    def update(self, x, y, z, yaw, declared_xy):
        """Them mot mau; tra False neu bi loai (qua xa vi tri khai bao)."""
        if math.hypot(x - declared_xy[0], y - declared_xy[1]) > self.max_offset_m:
            return False
        if self.pos is None:
            self.pos = (x, y, z)
            self.c, self.s = math.cos(yaw), math.sin(yaw)
            return True
        a = self.alpha
        self.pos = tuple(p + a * (v - p) for p, v in zip(self.pos, (x, y, z)))
        self.c += a * (math.cos(yaw) - self.c)
        self.s += a * (math.sin(yaw) - self.s)
        return True

    def value(self):
        """(x, y, z, yaw) hoac None neu chua co mau nao."""
        if self.pos is None:
            return None
        return self.pos + (math.atan2(self.s, self.c),)
