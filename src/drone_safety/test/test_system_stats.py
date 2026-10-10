"""Kiem doc so lieu he thong Pi: loadavg, meminfo, bit ha xung."""

import pytest

from drone_safety.system_stats import parse_loadavg, parse_meminfo, parse_throttled, read_stats


def test_loadavg():
    assert parse_loadavg('27.86 12.94 7.24 4/679 19541\n') == (27.86, 12.94, 7.24)


def test_meminfo_theo_memavailable():
    text = 'MemTotal:        8000 kB\nMemFree:         1000 kB\nMemAvailable:    6000 kB\n'
    assert parse_meminfo(text) == pytest.approx(25.0)


def test_throttled_khong_co_gi():
    assert parse_throttled('throttled=0x0\n') == (0, [])


def test_throttled_dang_thieu_ap_va_tung_ha_xung():
    value, flags = parse_throttled('throttled=0x40001')
    assert value == 0x40001 and flags == ['thieu ap', 'tung ha xung']


def test_read_stats_thieu_file_khong_loi(tmp_path):
    s = read_stats(str(tmp_path))                 # khong co /proc, /sys gia -> bo qua, khong crash
    assert 'load_1m' not in s and 'cpu_temp_c' not in s
