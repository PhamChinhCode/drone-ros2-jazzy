"""Vong P yaw tren Pi (WP5, docs/ke_hoach_huong_bay_hai_tag.md) - thuan Python, pytest duoc.

mission_manager_node phat yaw mong muon (ENU, rad) tren /mission/yaw; position_controller_node doi
ra yaw_rate FLU (duong = quay trai = yaw ENU tang), kep max_rate de than khong quay giat va tag
khong truot khoi khung hinh khi dang bam.
"""

import math


def wrap(a):
    """Goc ve (-pi, pi]."""
    return math.atan2(math.sin(a), math.cos(a))


def yaw_rate_command(yaw_sp, yaw, kp, max_rate):
    """yaw_rate (rad/s) = kp * sai so yaw theo duong NGAN, kep +-max_rate."""
    return max(-max_rate, min(max_rate, kp * wrap(yaw_sp - yaw)))
