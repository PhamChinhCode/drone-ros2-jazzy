#!/usr/bin/env python3
"""Bao cao chuyen bay tu bag cua Pi (~/drone_logs/bag_*): dien bien + cac dau hieu su co.

Chay tren Pi hoac may co ROS 2 Jazzy + drone_interfaces (source install/setup.bash truoc):
    python3 tools/flight_report.py ~/drone_logs/bag_20261010_065644
    python3 tools/flight_report.py <bag> --tu 17:03 --den 17:10     # chi mot khoang gio
    python3 tools/flight_report.py <bag> --anh /tmp/anh             # xuat anh camera_log (2 Hz)

Doc duoc ca bag dang ghi do / mat dien (chua co metadata.yaml): doc thang tung file .mcap.
Muc in ra: arm/disarm + che do FC, dien bien FSM, failsafe, co OFFBOARD cua FC (OB_*), cong tac
va can RC (nguoi lai cham can / gat ch5-ch8), lien ket GCS, suc khoe EKF, tung lan ha theo tag
(mat tag o do cao nao, toc do ngang lon nhat, lech tag luc cham dat), tai/nhiet Pi, canh bao log.
"""

import argparse
import datetime
import glob
import math
import os
from collections import Counter, defaultdict

import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

STATE = {0: 'IDLE', 1: 'TAKEOFF', 2: 'ENROUTE', 3: 'MARKER_SEARCH', 4: 'PRECISION_LAND',
         5: 'ACTUATE_GRIPPER', 6: 'RETRY_LOITER', 7: 'RTH', 8: 'EMERGENCY_LAND',
         9: 'MISSION_COMPLETE', 10: 'FAILSAFE', 11: 'ALIGN_HEADING', 12: 'PAD_ALIGN',
         13: 'FINAL_APPROACH'}
FS_TYPE = {0: '-', 1: 'MARKER_TIMEOUT', 2: 'GRIP_CONFIRM_FAIL', 3: 'LINK_LOST', 4: 'LOW_BATTERY',
           5: 'EKF_UNHEALTHY', 6: 'FC_COMM_LOST'}
OB_STATE = {0: 'KHOA', 1: 'TAT', 2: 'DANG_CHAY'}
OB_EXIT = {0: '-', 1: 'KHOI_DONG', 2: 'CONG_TAC', 3: 'HET_HAN', 4: 'DAY_CAN', 5: 'DISARM',
           6: 'KEP_DAI', 7: 'MAT_GOC', 8: 'MAT_SONG', 9: 'CHE_DO'}
TOPICS = ('/mavros/state', '/mission/state', '/failsafe_event', '/mavros/debug_value/named_value_int',
          '/mavros/rc/in', '/gcs_link/connected', '/ekf/health', '/landing_target/pose',
          '/range/vertical', '/odometry/filtered', '/system/stats', '/rosout',
          '/log/camera/compressed', '/mavros/battery')
TAG_GAP_S = 0.5            # bridge ngung phat pose -> FSM coi la mat tag sau ~1 s
STICK_TOUCH_US = 60         # can lech khoi giua hon muc nay = nguoi lai cham can (FC thoat OFFBOARD)


def readers(path):
    """Mot hoac nhieu SequentialReader: thu muc co metadata, hoac tung file .mcap le."""
    if os.path.isdir(path) and os.path.isfile(os.path.join(path, 'metadata.yaml')):
        uris = [path]
    elif os.path.isdir(path):
        uris = sorted(glob.glob(os.path.join(path, '*.mcap')))
    else:
        uris = [path]
    for uri in uris:
        r = rosbag2_py.SequentialReader()
        sid = 'mcap' if uri.endswith('.mcap') else ''
        r.open(rosbag2_py.StorageOptions(uri=uri, storage_id=sid), rosbag2_py.ConverterOptions('', ''))
        yield r


def load(path, t_from, t_to):
    data = defaultdict(list)
    types = {}
    for r in readers(path):
        for t in r.get_all_topics_and_types():
            types.setdefault(t.name, t.type)
        r.set_filter(rosbag2_py.StorageFilter(topics=[t for t in TOPICS if t in types]))
        while r.has_next():
            topic, raw, ns = r.read_next()
            ts = ns / 1e9
            if (t_from and ts < t_from) or (t_to and ts > t_to):
                continue
            data[topic].append((ts, deserialize_message(raw, get_message(types[topic]))))
    for v in data.values():
        v.sort(key=lambda x: x[0])
    return data, types


def clock(ts):
    return datetime.datetime.fromtimestamp(ts).strftime('%H:%M:%S.%f')[:-4]


def value_at(series, ts):
    """Gia tri cuoi cung truoc hoac dung ts (series [(t, v)])."""
    lo, hi, best = 0, len(series) - 1, None
    while lo <= hi:
        mid = (lo + hi) // 2
        if series[mid][0] <= ts:
            best, lo = series[mid][1], mid + 1
        else:
            hi = mid - 1
    return best


def section(title):
    print(f'\n=== {title}')


def report(data, types, anh_dir):
    all_ts = [ts for v in data.values() for ts, _ in v]
    if not all_ts:
        print('Bag khong co topic nao can thiet.')
        return
    t0, t1 = min(all_ts), max(all_ts)
    print(f'Bag {clock(t0)} -> {clock(t1)} ({t1 - t0:.0f} s); topic co du lieu: '
          f'{sum(1 for v in data.values() if v)}/{len(TOPICS)} muc can, tong {len(types)} topic')
    missing = [t for t in TOPICS if not data.get(t)]
    if missing:
        print('Thieu (khong ghi / khong phat):', ', '.join(missing))

    rng = [(ts, m.range) for ts, m in data['/range/vertical'] if m.min_range <= m.range <= m.max_range]

    section('Arm / che do FC (/mavros/state)')
    prev = None
    for ts, m in data['/mavros/state']:
        cur = (m.connected, m.armed, m.mode)
        if cur != prev:
            print(f'{clock(ts)}  ket noi={m.connected} arm={m.armed} mode={m.mode}')
            prev = cur

    section('Dien bien nhiem vu (FSM)')
    prev = None
    for ts, m in data['/mission/state']:
        if (m.mission_id, m.state) != prev:
            h = value_at(rng, ts)
            hs = f'laser {h:.2f} m' if h is not None else ''
            print(f'{clock(ts)}  [{m.mission_id}] {STATE.get(m.state, m.state):15s} wp{m.current_wp_index} '
                  f'thu lai {m.retry_count} {hs:13s} {m.detail}')
            prev = (m.mission_id, m.state)

    section('Failsafe')
    for ts, m in data['/failsafe_event']:
        print(f'{clock(ts)}  {"BAT" if m.active else "het"} {FS_TYPE.get(m.type, m.type)} '
              f'-> leo thang {m.escalate_to}: {m.detail}')

    section('Co OFFBOARD cua FC (doi gia tri)')
    last = {}
    for ts, m in data['/mavros/debug_value/named_value_int']:
        if m.name not in ('OB_STATE', 'OB_AUTH', 'OB_EXIT', 'OB_ARM_BLK', 'OB_ARM_RDY'):
            continue
        v = m.value_int
        if last.get(m.name) != v:
            txt = {'OB_STATE': OB_STATE, 'OB_EXIT': OB_EXIT}.get(m.name, {}).get(v, '')
            txt = txt or (f'0x{v:04x}' if m.name == 'OB_ARM_BLK' else '')
            print(f'{clock(ts)}  {m.name} = {v} {txt}')
            last[m.name] = v

    section('RC: cong tac ch5/ch6/ch8, nguoi lai cham can khi dang arm')
    armed = [(ts, m.armed) for ts, m in data['/mavros/state']]
    prev_sw, prev_touch = None, False
    for ts, m in data['/mavros/rc/in']:
        ch = list(m.channels)
        if len(ch) < 8:
            continue
        sw = tuple('LEN' if c > 1700 else 'XUONG' if c < 1300 else 'GIUA' for c in (ch[4], ch[5], ch[7]))
        if sw != prev_sw:
            print(f'{clock(ts)}  ch5={sw[0]} ch6={sw[1]} ch8={sw[2]}')
            prev_sw = sw
        touch = bool(value_at(armed, ts)) and any(abs(c - 1500) > STICK_TOUCH_US for c in ch[:4])
        if touch != prev_touch:
            print(f'{clock(ts)}  {"CHAM CAN" if touch else "tha can"}  ch1-4 = {ch[:4]}')
            prev_touch = touch

    section('Lien ket GCS')
    prev = None
    for ts, m in data['/gcs_link/connected']:
        if m.data != prev:
            print(f'{clock(ts)}  {"co ket noi" if m.data else "MAT KET NOI"}')
            prev = m.data

    section('Suc khoe EKF (khoang khong healthy)')
    bad = None
    for ts, m in data['/ekf/health']:
        if not m.healthy and bad is None:
            bad = (ts, m.reason)
        elif m.healthy and bad is not None:
            print(f'{clock(bad[0])}  khong healthy {ts - bad[0]:.1f} s: {bad[1]}')
            bad = None
    if bad:
        print(f'{clock(bad[0])}  khong healthy toi cuoi bag: {bad[1]}')

    section('Tung lan ha theo tag (PRECISION_LAND)')
    segs, start = [], None
    for ts, m in data['/mission/state']:
        if m.state == 4 and start is None:
            start = ts
        elif m.state != 4 and start is not None:
            segs.append((start, ts, STATE.get(m.state, m.state), m.detail))
            start = None
    tag = [(ts, math.hypot(m.pose.position.x, m.pose.position.y)) for ts, m in data['/landing_target/pose']]
    odo = [(ts, math.hypot(m.twist.twist.linear.x, m.twist.twist.linear.y)) for ts, m in data['/odometry/filtered']]
    for a, b, nxt, why in segs:
        h0 = value_at(rng, a)
        ts_tag = [ts for ts, _ in tag if a <= ts <= b]
        gaps = [(p, q) for p, q in zip([a] + ts_tag, ts_tag + [b]) if q - p > TAG_GAP_S]
        vmax = max((v for ts, v in odo if a <= ts <= b), default=float('nan'))
        offs = [o for ts, o in tag if a <= ts <= b]
        print(f'{clock(a)}  {b - a:5.1f} s tu laser {h0 if h0 is None else round(h0, 2)} m -> {nxt} '
              f'({why}); v ngang max {vmax:.2f} m/s; lech tag cuoi '
              f'{offs[-1] * 100 if offs else float("nan"):.0f} cm')
        if '/landing_target/pose' not in types:
            print('      (bag khong ghi /landing_target/pose - khong xet duoc mat tag)')
            continue
        for p, q in gaps:
            h = value_at(rng, p)
            print(f'      mat tag {q - p:.1f} s tu {clock(p)} o laser {h if h is None else round(h, 2)} m')

    section('Pi: tai / nhiet / ha xung (/system/stats)')
    worst = defaultdict(float)
    warns = Counter()
    for ts, m in data['/system/stats']:
        for st in m.status:
            kv = {k.key: k.value for k in st.values}
            for k in ('load_1m', 'cpu_temp_c', 'mem_used_pct'):
                if k in kv:
                    worst[k] = max(worst[k], float(kv[k]))
            if st.level:
                warns[st.message] += 1
    if worst:
        print('max: ' + ', '.join(f'{k} {v:.1f}' for k, v in worst.items()))
    for msg, n in warns.most_common(5):
        print(f'  canh bao {n} lan: {msg}')

    section('Pin')
    v = [m.voltage for _, m in data['/mavros/battery'] if m.voltage > 0.5]
    print(f'{min(v):.2f} -> {max(v):.2f} V' if v else 'FC khong bao dien ap (0 V) - KHONG co failsafe pin')

    section('Log node muc WARN tro len (/rosout, gom theo node + noi dung)')
    cnt, first = Counter(), {}
    for ts, m in data['/rosout']:
        if m.level < 30:
            continue
        key = (m.level, m.name, m.msg[:110])
        cnt[key] += 1
        first.setdefault(key, ts)
    for (lvl, name, msg), n in sorted(cnt.items(), key=lambda kv: (-kv[0][0], -kv[1]))[:25]:
        print(f'{"ERROR" if lvl >= 40 else "WARN "} x{n:<5d} {clock(first[(lvl, name, msg)])} {name}: {msg}')

    frames = data['/log/camera/compressed']
    section(f'Anh camera_log: {len(frames)} khung')
    if anh_dir and frames:
        os.makedirs(anh_dir, exist_ok=True)
        for ts, m in frames:
            with open(os.path.join(anh_dir, clock(ts).replace(':', '') + '.jpg'), 'wb') as f:
                f.write(bytes(m.data))
        print(f'da xuat vao {anh_dir} (ten file = gio:phut:giay)')


def parse_clock(s, day_ts):
    if not s:
        return None
    hh, mm, *ss = (int(x) for x in s.split(':'))
    d = datetime.datetime.fromtimestamp(day_ts)
    return d.replace(hour=hh, minute=mm, second=ss[0] if ss else 0, microsecond=0).timestamp()


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('bag', help='thu muc bag_* hoac mot file .mcap')
    ap.add_argument('--tu', help='HH:MM[:SS] bat dau (gio may Pi)')
    ap.add_argument('--den', help='HH:MM[:SS] ket thuc')
    ap.add_argument('--anh', help='thu muc xuat anh /log/camera/compressed')
    a = ap.parse_args()
    day = os.path.getmtime(a.bag)
    data, types = load(a.bag, parse_clock(a.tu, day), parse_clock(a.den, day))
    report(data, types, a.anh)


if __name__ == '__main__':
    main()
