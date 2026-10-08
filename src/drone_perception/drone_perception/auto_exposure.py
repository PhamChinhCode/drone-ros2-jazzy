"""Logic tu phoi sang thuan, tach khoi ROS de pytest duoc.

OV9281 doc thang qua V4L2 khong co IPA nen KHONG co auto-exposure: tu dong vong o day.
Do sang = exposure (so dong) x analogue_gain (16 = 1x). Uu tien exposure truoc vi gain
them nhieu; exposure co tran (exposure_max) de anh khong nhoe khi bay.
"""

from dataclasses import dataclass

import numpy as np


@dataclass
class ExposureLimits:
    target_mean: float = 60.0
    deadband: float = 0.15          # |mean - target| / target duoi muc nay thi giu nguyen
    max_saturated: float = 0.02     # ti le diem >= 250 toi da truoc khi buoc giam sang
    max_step: float = 2.0           # moi lan doi toi da x2 hoac /2 - tranh dao dong
    exposure_min: int = 4
    exposure_max: int = 500
    gain_min: int = 16
    gain_max: int = 128


def image_stats(gray, stride=4):
    """(do sang trung binh, ti le diem chay sang) tren anh lay mau thua."""
    sub = gray[::stride, ::stride]
    return float(sub.mean()), float(np.count_nonzero(sub >= 250)) / sub.size


def next_setting(exposure, gain, mean, saturated, lim):
    """(exposure, gain) moi, hoac None neu giu nguyen.

    Phan bo lai tu dau moi lan: exposure gan tran truoc voi gain thap nhat, het tran exposure
    moi tang gain. Nho vay khi giam sang thi gain tu giam truoc.
    """
    if saturated > lim.max_saturated:
        ratio = 0.7
    else:
        if abs(mean - lim.target_mean) <= lim.deadband * lim.target_mean:
            return None
        ratio = lim.target_mean / max(mean, 1.0)
    ratio = min(max(ratio, 1.0 / lim.max_step), lim.max_step)

    want = exposure * gain * ratio
    new_exp = int(round(min(max(want / lim.gain_min, lim.exposure_min), lim.exposure_max)))
    new_gain = int(round(min(max(want / new_exp, lim.gain_min), lim.gain_max)))
    if (new_exp, new_gain) == (exposure, gain):
        return None
    return new_exp, new_gain
