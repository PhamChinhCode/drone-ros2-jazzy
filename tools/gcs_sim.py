#!/usr/bin/env python3
"""gcs_sim - GCS toi thieu de thu kenh GCS <-> Pi. Khong can ROS.

Vai trò (giao uoc GCS <-> Pi muc 7.3): CA HAI BEN test nguoc vao no. Pi test khi chua co GCS
that; GCS test bang cach so voi hanh vi cua no. No cung la dac ta CHAY DUOC - khi tai lieu va no
lech nhau thi it nhat co cho de do.

  python3 tools/gcs_sim.py                                  # chi nghe, in telemetry
  python3 tools/gcs_sim.py --plan config/missions/ban_home.yaml
  python3 tools/gcs_sim.py --cmd rtl                        # rtl land start disarm abort pause
  python3 tools/gcs_sim.py --plan <yaml> --cmd start        # nap roi cat canh

Doc chung dinh dang YAML voi `ros2 run drone_mission send_mission_plan`, nen mot file ke hoach
chay duoc ca hai duong: noi bo ROS va qua day.
"""

import argparse
import os
import socket
import sys
import time

GOC = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(GOC, 'src', 'drone_comms'))

try:
    from drone_comms import dialect_gcs as d
    from drone_comms.tagmap import to_ascii
except ImportError:
    sys.exit('Thieu module dialect. Sinh bang: python3 tools/sinh_dialect.py')

SYSID_GCS, COMPID_GCS = 255, 190
SYSID_PI, COMPID_PI = 1, 191
CONTRACT_VER = 400                       # ban 0.4

ACTIONS = {'none': 0, 'pickup': 1, 'dropoff': 2}
LENH = {'rtl': 20, 'land': 21, 'start': 300, 'disarm': 400, 'pause': 193,
        'abort': 42100, 'arm': 400}
TEN_KET_QUA = {0: 'ACCEPTED', 1: 'ERR_TIMEOUT', 2: 'ERR_COUNT', 3: 'ERR_UNKNOWN_TAG',
               4: 'ERR_PARAM', 5: 'ERR_BUSY', 6: 'ERR_CONTRACT'}
TEN_ACK = {0: 'ACCEPTED', 1: 'TEMPORARILY_REJECTED', 2: 'DENIED', 3: 'UNSUPPORTED', 4: 'FAILED'}
TEN_TRANG_THAI = ['IDLE', 'TAKEOFF', 'ENROUTE', 'MARKER_SEARCH', 'PRECISION_LAND',
                  'ACTUATE_GRIPPER', 'RETRY_LOITER', 'RTH', 'EMERGENCY_LAND',
                  'MISSION_COMPLETE', 'FAILSAFE']
TEN_CO = [(1, 'POS'), (2, 'GLOBAL_POS'), (4, 'BATTERY'), (8, 'FC_LINK'), (16, 'MARKER'),
          (32, 'GRIPPER'), (64, 'EKF_OK'), (128, 'RSSI'), (256, 'HOME')]
TEN_TRANG = [(1, 'ARMED'), (2, 'PI_AUTH'), (4, 'CARRYING')]


def doc_ke_hoach(duong):
    import yaml
    k = yaml.safe_load(open(duong))
    return k, [dict(seq=i, marker_id=int(w['marker_id']),
                    action=ACTIONS[str(w.get('action', 'none')).lower()],
                    alt_m=float(w['alt_m']), acceptance_radius_m=float(w['acceptance_radius_m']),
                    max_vel_mps=float(w['max_vel_mps']), loiter_s=float(w.get('loiter_s', 0.0)))
               for i, w in enumerate(k['waypoints'])]


def ten_co(gia_tri, bang):
    co = [t for b, t in bang if gia_tri & b]
    return ','.join(co) if co else '-'


class GcsSim:

    def __init__(self, dich, cong_nghe, ver=CONTRACT_VER, khoa=None):
        self.ver = ver
        self.ml = d.MAVLink(None, srcSystem=SYSID_GCS, srcComponent=COMPID_GCS)
        self.ml.robust_parsing = True
        if khoa:
            self.ml.signing.secret_key = khoa
            self.ml.signing.link_id = 0
            self.ml.signing.timestamp = 0
            self.ml.signing.sign_outgoing = True
            self.ml.signing.allow_unsigned_callback = lambda _m, _i: False
        self.s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.s.setblocking(False)
        self.s.bind(('0.0.0.0', cong_nghe))
        self.dich = dich
        self.seq_pi = None
        self.rx_ok = self.rx_drop = 0
        self.ke_hoach = None
        self.diem = []
        self.xong_nap = None
        self.heartbeat_luc = 0.0
        self.bo_diem = -1

    def gui(self, msg):
        self.s.sendto(msg.pack(self.ml), self.dich)
        # pack() KHONG tang seq (chi MAVLink.send() tang). Thieu dong nay thi phia Pi dem
        # rx_drop sai hoan toan.
        self.ml.seq = (self.ml.seq + 1) % 256

    def nap_ke_hoach(self, k, diem):
        self.ke_hoach, self.diem, self.xong_nap = k, diem, None
        print(f'--> nap ke hoach {k["mission_id"]} "{k.get("plan_name","")}" ({len(diem)} diem)')
        self.gui(d.MAVLink_drone_mission_count_message(
            mission_id=int(k['mission_id']),
            issued_stamp_us=int(time.time() * 1e6),
            search_timeout_s=float(k.get('search_timeout_s', 0.0)),
            count=len(diem), max_retries=int(k.get('max_retries', 0)),
            # Muc 8.4: char[] la ASCII khong dau, BEN GUI bo dau. Voi plan_name ben gui la
            # GCS. Cat theo BYTE sau khi bo dau, khong cat theo ky tu roi moi ma hoa.
            plan_name=to_ascii(str(k.get('plan_name', '')), 20),
            contract_ver=self.ver))

    def heartbeat(self):
        """1 Hz, HAI CHIEU theo muc 5.1. Thieu no thi Pi khong bao gio thay GCS song:
        watchdog cua Pi dua vao goi NHAN DUOC, khong dua vao goi minh gui (muc 9.2)."""
        t = time.monotonic()
        if t - self.heartbeat_luc < 1.0:
            return
        self.heartbeat_luc = t
        self.gui(d.MAVLink_heartbeat_message(
            type=d.MAV_TYPE_GCS, autopilot=d.MAV_AUTOPILOT_INVALID, base_mode=0,
            custom_mode=0, system_status=d.MAV_STATE_ACTIVE, mavlink_version=3))

    def gui_lenh(self, ten, so_lan=1, raw=None):
        c = raw if raw is not None else LENH[ten]
        p1 = 1.0 if ten == 'arm' else 0.0
        p2 = 21196.0 if ten == 'disarm' else 0.0
        for i in range(so_lan):
            # confirmation TANG moi lan, dung co che chuan. Day la diem A7 kiem: bat bien phai
            # den tu TRANG THAI, khong tu khoa (command, confirmation) - loi P4 cua ban 0.1.
            print(f'--> lenh {ten or raw} (MAV_CMD {c}) confirmation={i}')
            self.gui(d.MAVLink_command_long_message(
                target_system=SYSID_PI, target_component=COMPID_PI, command=c, confirmation=i,
                param1=p1, param2=p2, param3=0, param4=0, param5=0, param6=0, param7=0))

    def doc(self):
        while True:
            try:
                data, _ = self.s.recvfrom(1500)
            except (BlockingIOError, OSError):
                return
            for m in self.ml.parse_buffer(data) or []:
                if m.get_type() == 'BAD_DATA':
                    continue
                if m.get_srcSystem() != SYSID_PI or m.get_srcComponent() != COMPID_PI:
                    continue
                self.rx_ok += 1
                if self.seq_pi is not None:
                    self.rx_drop += (m.get_seq() - self.seq_pi - 1) & 0xFF
                self.seq_pi = m.get_seq()
                self.xu_ly(m)

    def xu_ly(self, m):
        t = m.get_type()
        if t == 'DRONE_MISSION_REQUEST':
            if m.seq == self.bo_diem:
                print(f'<-- hoi diem {m.seq}, CO Y KHONG GUI (kiem A4/A5)')
                return
            if self.diem and m.seq < len(self.diem):
                w = self.diem[m.seq]
                print(f'<-- hoi diem {m.seq}, gui')
                self.gui(d.MAVLink_drone_mission_item_message(
                    mission_id=m.mission_id, expected_marker_id=w['marker_id'],
                    alt_m=w['alt_m'], acceptance_radius_m=w['acceptance_radius_m'],
                    max_vel_mps=w['max_vel_mps'], loiter_s=w['loiter_s'],
                    seq=w['seq'], action=w['action']))
        elif t == 'DRONE_MISSION_ACK':
            ly_do = m.reason.rstrip('\x00') if isinstance(m.reason, str) else ''
            print(f'<-- ACK nap: {TEN_KET_QUA.get(m.result, m.result)}'
                  + (f' | {ly_do}' if ly_do else ''))
            self.xong_nap = m.result
        elif t == 'COMMAND_ACK':
            print(f'<-- ACK lenh {m.command}: {TEN_ACK.get(m.result, m.result)}')
        elif t == 'STATUSTEXT':
            print(f'<-- [{m.severity}] {m.text.rstrip(chr(0)) if isinstance(m.text, str) else m.text}')
        elif t == 'DRONE_TELEMETRY':
            print(f'<-- {TEN_TRANG_THAI[m.mission_state] if m.mission_state < 11 else m.mission_state:<16} '
                  f'wp {m.current_wp_index}/{m.wp_total} thu_lai {m.retry_count} | '
                  f'alt {m.alt_m:+.2f} | tim tag {m.expected_marker_id} bam {m.marker_id_tracking} | '
                  f'crc 0x{m.tagmap_crc:08X} v{m.contract_ver} | co [{ten_co(m.valid_flags, TEN_CO)}] '
                  f'[{ten_co(m.status_flags, TEN_TRANG)}]')
        elif t == 'DRONE_LINK_STATS':
            print(f'<-- link: rx_ok {m.rx_ok} rx_drop {m.rx_drop} tx_sent {m.tx_sent} '
                  f'tx_dropped {m.tx_dropped} queue {m.queue_depth}')
        elif t == 'HEARTBEAT':
            pass


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument('--host', default='127.0.0.1')
    ap.add_argument('--port', type=int, default=14551, help='cong Pi lang nghe')
    ap.add_argument('--listen', type=int, default=14550, help='cong GCS lang nghe')
    ap.add_argument('--plan')
    ap.add_argument('--cmd', choices=sorted(LENH))
    ap.add_argument('--raw-cmd', type=int, help='gui MAV_CMD bat ky (kiem A6: lenh la)')
    ap.add_argument('--lan', type=int, default=1, help='so lan gui lenh, confirmation 0,1,2... (A7)')
    ap.add_argument('--cmd2', choices=sorted(LENH), help='lenh thu hai, gui o giay --cmd2-luc')
    ap.add_argument('--cmd2-luc', type=float, default=20.0, help='giay de gui --cmd2')
    ap.add_argument('--khoa', default='~/.drone_gcs_key',
                    help='tep khoa chu ky (muc 7.6); --khoa "" de chay khong chu ky')
    ap.add_argument('--ver', type=int, default=CONTRACT_VER,
                    help='contract_ver gui trong COUNT (kiem A12: lech MAJOR)')
    ap.add_argument('--giay', type=float, default=15.0, help='chay bao lau roi thoat')
    ap.add_argument('--bo-diem', type=int, default=-1,
                    help='KHONG gui diem so N (kiem A4/A5: Pi phai hoi lai roi ERR_TIMEOUT)')
    a = ap.parse_args()

    khoa = None
    if a.khoa:
        duong = os.path.expanduser(a.khoa)
        if os.path.isfile(duong):
            khoa = open(duong, 'rb').read().strip()
            if len(khoa) != 32:
                sys.exit(f'khoa {duong} phai dung 32 byte')
            print(f'chu ky goi DA BAT (khoa {duong})')
        else:
            print(f'khong thay khoa {duong} - chay KHONG chu ky')
    g = GcsSim((a.host, a.port), a.listen, a.ver, khoa)
    g.bo_diem = a.bo_diem
    print(f'gcs_sim: nghe :{a.listen}, gui toi {a.host}:{a.port}')
    t0 = time.monotonic()
    da_nap = da_lenh = da_lenh2 = False
    while time.monotonic() - t0 < a.giay:
        g.heartbeat()
        g.doc()
        dt = time.monotonic() - t0
        if a.plan and not da_nap and dt > 1.0:
            g.nap_ke_hoach(*doc_ke_hoach(a.plan))
            da_nap = True
        if (a.cmd or a.raw_cmd) and not da_lenh and dt > 1.0 \
                and (not a.plan or g.xong_nap is not None):
            g.gui_lenh(a.cmd, a.lan, a.raw_cmd)
            da_lenh = True
        if a.cmd2 and not da_lenh2 and dt > a.cmd2_luc:
            g.gui_lenh(a.cmd2)
            da_lenh2 = True
        time.sleep(0.02)
    print(f'--- ket thuc: rx_ok {g.rx_ok}, rx_drop {g.rx_drop}')


if __name__ == '__main__':
    main()
