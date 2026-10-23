"""Giu bag theo hoat dong (user 10-23): bag ghi lien tuc, cat file 1 phut (full_system.launch.py);
chi GIU file trung khoang co hoat dong +-PAD_S, file khac giu IDLE_KEEP_S roi xoa.

Hoat dong = FC da arm, HOAC Pi co quyen (OB_AUTH = 1), HOAC co nhiem vu (FSM khac IDLE /
MISSION_COMPLETE). log_activity_node ghi moc thoi gian (giay epoch) moi giay dang hoat dong vao
<log_dir>/hoat_dong.txt. Bay tay (ALTHOLD/POSHOLD) cung giu vi FC arm - flow/ToF di qua Pi.

Ngoai ra van giu tran dung luong (mac dinh 10 GB): con vuot thi xoa file cu nhat bat ke hoat dong.
File moi nhat (dang ghi) KHONG BAO GIO bi xoa. Nhat ky nghiep vu (<mission_id>/) khong dong vao.

Chay: scripts/prune_logs.sh (drone_startup.sh goi luc khoi dong va moi 10 phut).
"""

import glob
import os
import shutil
import sys
import time

ACTIVE_FILE = 'hoat_dong.txt'
PAD_S = 120.0              # giu them truoc / sau khoang hoat dong (luc arm, sau khi roi)
IDLE_KEEP_S = 1800.0       # file khong hoat dong giu chung nay (xem lai lan thu tren ban)
SPLIT_S = 60.0             # --max-bag-duration cua recorder


def la_hoat_dong(armed, ob_auth, mission_active):
    return bool(armed) or ob_auth is True or bool(mission_active)


def trung_hoat_dong(start_s, end_s, moc, pad_s=PAD_S):
    """File [start_s, end_s] co moc hoat dong nao trong khoang +-pad_s khong."""
    return any(start_s - pad_s <= t <= end_s + pad_s for t in moc)


def chon_xoa(files, moc, now_s, cap_bytes, pad_s=PAD_S, idle_keep_s=IDLE_KEEP_S,
             split_s=SPLIT_S):
    """files: list (path, mtime_s, size_bytes) cac file bag. Tra list path can xoa.

    mtime = luc file ngung ghi; file bat dau ~split_s truoc do.
    """
    files = sorted(files, key=lambda f: f[1])
    if not files:
        return []
    cu = files[:-1]                                   # file moi nhat dang ghi: giu
    xoa = [f for f in cu if now_s - f[1] > idle_keep_s
           and not trung_hoat_dong(f[1] - split_s, f[1], moc, pad_s)]
    total = sum(f[2] for f in files) - sum(f[2] for f in xoa)
    for f in cu:
        if total <= cap_bytes:
            break
        if f not in xoa:
            xoa.append(f)
            total -= f[2]
    return [f[0] for f in xoa]


def doc_moc(path):
    try:
        with open(path, encoding='ascii') as f:
            return [float(x) for x in f.read().split()]
    except (OSError, ValueError):
        return []


def main():
    log_dir = os.environ.get('DRONE_LOG_DIR', os.path.expanduser('~/drone_logs'))
    cap = float(os.environ.get('DRONE_LOG_CAP_GB', '10')) * 1024 ** 3
    if not os.path.isdir(log_dir):
        return 0
    files = []
    for p in glob.glob(os.path.join(log_dir, 'bag_*', '*.mcap')) + \
            glob.glob(os.path.join(log_dir, 'bag_*', '*.db3')):
        st = os.stat(p)
        files.append((p, st.st_mtime, st.st_size))
    xoa = chon_xoa(files, doc_moc(os.path.join(log_dir, ACTIVE_FILE)), time.time(), cap)
    for p in xoa:
        os.remove(p)
    for d in glob.glob(os.path.join(log_dir, 'bag_*', '')):
        if not glob.glob(d + '*.mcap') and not glob.glob(d + '*.db3'):
            shutil.rmtree(d, ignore_errors=True)
    if xoa:
        print(f'[prune_logs] da xoa {len(xoa)} file bag')
    return 0


if __name__ == '__main__':
    sys.exit(main())
