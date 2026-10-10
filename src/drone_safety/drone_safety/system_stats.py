"""Doc tai CPU / nhiet do / ha xung / RAM / dia cua Pi - thuan Python, pytest duoc.

10-10: unattended-upgrade + needrestart lam stack khoi dong lai 7 lan va tai len 28, EKF tre; 10-09
lenh do cua chinh minh day tai len 30. Can ghi kem chuyen bay de biet Pi co qua tai luc su co.
"""

import os
import shutil
import subprocess

# Bit cua vcgencmd get_throttled (Raspberry Pi): 0-3 dang xay ra, 16-19 da tung xay ra tu khi boot.
THROTTLED_BITS = {0: 'thieu ap', 1: 'gioi han tan so', 2: 'dang ha xung', 3: 'gioi han nhiet',
                  16: 'tung thieu ap', 17: 'tung gioi han tan so', 18: 'tung ha xung',
                  19: 'tung gioi han nhiet'}


def parse_loadavg(text):
    """'/proc/loadavg' -> (1 phut, 5 phut, 15 phut)."""
    a, b, c = text.split()[:3]
    return float(a), float(b), float(c)


def parse_meminfo(text):
    """'/proc/meminfo' -> % RAM dang dung (theo MemAvailable)."""
    kv = {}
    for line in text.splitlines():
        name, _, rest = line.partition(':')
        kv[name] = int(rest.split()[0])
    return 100.0 * (1.0 - kv['MemAvailable'] / kv['MemTotal'])


def parse_throttled(text):
    """'throttled=0x50005' -> (gia tri, [ten bit dang bat])."""
    value = int(text.strip().split('=')[1], 16)
    return value, [name for bit, name in THROTTLED_BITS.items() if value & (1 << bit)]


def read_stats(root='/'):
    """Doc mot luot; muc nao khong doc duoc thi bo qua (vd may khong phai Pi)."""
    out = {}
    try:
        with open(os.path.join(root, 'proc/loadavg')) as f:
            out['load_1m'], out['load_5m'], out['load_15m'] = parse_loadavg(f.read())
        out['cpu_count'] = os.cpu_count()
    except OSError:
        pass
    try:
        with open(os.path.join(root, 'sys/class/thermal/thermal_zone0/temp')) as f:
            out['cpu_temp_c'] = int(f.read()) / 1000.0
    except (OSError, ValueError):
        pass
    try:
        with open(os.path.join(root, 'proc/meminfo')) as f:
            out['mem_used_pct'] = parse_meminfo(f.read())
    except (OSError, KeyError, ValueError):
        pass
    try:
        du = shutil.disk_usage(os.path.expanduser('~'))
        out['disk_free_gb'] = du.free / 1e9
    except OSError:
        pass
    try:
        r = subprocess.run(['vcgencmd', 'get_throttled'], capture_output=True, text=True,
                           timeout=1.0)
        out['throttled'], out['throttled_flags'] = parse_throttled(r.stdout)
    except (OSError, subprocess.SubprocessError, ValueError, IndexError):
        pass
    return out
