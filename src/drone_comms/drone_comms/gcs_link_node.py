"""gcs_link_node - duong lien lac rieng Pi 4 <-> GCS qua 4G/LTE (UDP, MAVLink 2).

Doc lap voi USART3/radio telemetry cua FC (phuong an (b), muc 1 tai lieu kien truc): khong phai
sua firmware FC de forward ban tin nhiem vu tuy bien.

Hop dong: docs/GIAO_UOC_GCS_PI.md. Dialect: docs/mavlink/drone_gcs.xml (sinh ma bang
tools/sinh_dialect.py, KHONG cheo tay bo ma hoa).

Bon nguyen tac bat buoc:
  1. Lenh khan cap di qua HANG DOI UU TIEN RIENG - khong xep sau telemetry (muc 5.3);
  2. TU phat hien mat ket noi bang watchdog - khong doi GCS bao, vi luc mat ket noi GCS khong
     bao duoc gi ca (muc 9.2);
  3. Pi GOI RA TRUOC va gui ve dia chi nguon cua goi HOP LE GAN NHAT tu (1, 191): modem 4G nam
     sau CGNAT nen GCS khong mo ket noi toi Pi duoc, va dia chi ay doi khi modem noi lai (muc 2.1);
  4. KHONG chuyen tiep bat ky ban tin nao giua kenh nay va kenh FC (muc 1.2).
"""

import math
import queue
import socket

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import Bool

from drone_comms.qos import EVENT_QOS, SENSOR_QOS
from drone_comms.tagmap import to_ascii
from drone_interfaces.msg import (MissionPlan, MissionPlanAck, MissionWaypoint,
                                  TelemetryPacket)

PRIORITY_EMERGENCY = 0
PRIORITY_MISSION = 1
PRIORITY_TELEMETRY = 2

SYSID_PI, COMPID_PI = 1, 191        # MAV_COMP_ID_ONBOARD_COMPUTER (muc 2.2)
SYSID_GCS, COMPID_GCS = 255, 190    # MAV_COMP_ID_MISSIONPLANNER

SO_DIEM_TOI_DA = 16                 # muc 3.2
ITEM_TIMEOUT_S = 1.0
ITEM_HOI_LAI_TOI_DA = 5
KHONG_DO_DUOC_32 = 0xFFFFFFFF       # muc 7.4: truong khong do duoc thi UINT32_MAX, khong phai 0
KHONG_DO_DUOC_16 = 0xFFFF

# MAV_CMD (muc 4.1)
CMD_NAV_RTL, CMD_NAV_LAND, CMD_MISSION_START = 20, 21, 300
CMD_ARM_DISARM, CMD_PAUSE_CONTINUE, CMD_ABORT = 400, 193, 42100
LENH_KHAN = (CMD_NAV_RTL, CMD_NAV_LAND, CMD_ARM_DISARM, CMD_ABORT)
ACK_ACCEPTED, ACK_DENIED, ACK_UNSUPPORTED = 0, 2, 3


class GcsLinkNode(Node):

    def __init__(self):
        super().__init__('gcs_link_node')

        self.declare_parameter('gcs_host', '127.0.0.1')
        self.declare_parameter('gcs_port', 14550)
        self.declare_parameter('local_port', 14551)
        self.declare_parameter('link_timeout_s', 5.0)
        self.declare_parameter('heartbeat_interval_s', 1.0)
        self.declare_parameter('contract_major', 0)

        try:
            from drone_comms import dialect_gcs
        except ImportError:
            self.get_logger().fatal(
                'Thieu module dialect. Sinh bang: python3 tools/sinh_dialect.py '
                '(no bi gitignore vi la ma dan xuat tu docs/mavlink/drone_gcs.xml)')
            raise
        self.ml = dialect_gcs.MAVLink(None, srcSystem=SYSID_PI, srcComponent=COMPID_PI)
        self.ml.robust_parsing = True
        self.d = dialect_gcs

        # PriorityQueue: so nho hon di truoc, nen lenh khan luon vuot len truoc telemetry.
        self.tx_queue = queue.PriorityQueue()
        self.tx_counter = 0
        self.tx_telemetry_cho = 0          # muc 5.3: giu toi da MOT goi telemetry trong hang doi
        self.dem = dict(rx_ok=0, rx_drop=0, rx_bad_crc=0, tx_sent=0, tx_dropped=0)
        self.seq_gcs = None
        self.last_rx_time = None
        self.connected = None              # None = chua bao lan nao
        self.gcs_addr = None               # dia chi goi HOP LE gan nhat (muc 2.1)

        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.socket.setblocking(False)
        self.socket.bind(('0.0.0.0', self.get_parameter('local_port').value))

        self.odom = None
        self.pos_valid = False             # lay tu valid_flags cua goi telemetry moi nhat
        self.nap = None                    # luot nap ke hoach dang chay
        self.cho_ack = None                # mission_id dang cho mission_manager phan quyet

        self.create_subscription(
            TelemetryPacket, '/telemetry/outgoing', self.on_telemetry, EVENT_QOS)
        self.create_subscription(Odometry, '/odometry/filtered', self.on_odom, SENSOR_QOS)
        self.create_subscription(
            MissionPlanAck, '/mission/plan_ack', self.on_plan_ack, EVENT_QOS)
        self.pub_plan = self.create_publisher(MissionPlan, '/mission/plan', EVENT_QOS)
        self.pub_connected = self.create_publisher(Bool, '/gcs_link/connected', EVENT_QOS)

        self.create_timer(0.01, self.pump_rx)
        self.create_timer(0.05, self.pump_tx)          # 20 Hz rut hang doi
        self.create_timer(0.5, self.check_watchdog)
        self.create_timer(self.get_parameter('heartbeat_interval_s').value, self.send_heartbeat)
        self.create_timer(0.2, self.send_pos_att)      # 5 Hz, chi khi POS_VALID
        self.create_timer(5.0, self.send_link_stats)   # 0,2 Hz
        self.get_logger().info(
            f"lang nghe UDP :{self.get_parameter('local_port').value}, goi ra "
            f"{self.get_parameter('gcs_host').value}:{self.get_parameter('gcs_port').value}")

    # ------------------------------------------------------------------ gui

    def enqueue(self, priority, msg):
        """tx_counter giu thu tu FIFO trong cung mot muc uu tien."""
        self.tx_counter += 1
        self.tx_queue.put((priority, self.tx_counter, msg))

    def dich_ra(self):
        """Dia chi gui: goi hop le gan nhat, chua co thi dia chi cau hinh (Pi goi ra truoc)."""
        return self.gcs_addr or (self.get_parameter('gcs_host').value,
                                 self.get_parameter('gcs_port').value)

    def pump_tx(self):
        dich = self.dich_ra()
        for _ in range(20):
            try:
                uu_tien, _, msg = self.tx_queue.get_nowait()
            except queue.Empty:
                return
            if uu_tien == PRIORITY_TELEMETRY:
                self.tx_telemetry_cho -= 1
            try:
                self.socket.sendto(msg.pack(self.ml), dich)
                # pymavlink chi tang seq trong MAVLink.send(); pack() thi KHONG. Khong tu tang
                # thi moi goi mang seq = 0 va bo dem mat goi cua ca hai ben thanh vo nghia -
                # dung thu ma muc 7.4 ton tai de do. Phat hien o phep kiem A2.
                self.ml.seq = (self.ml.seq + 1) % 256
                self.dem['tx_sent'] += 1
            except OSError as e:
                self.get_logger().warning(f'gui that bai: {e}')

    def on_telemetry(self, msg):
        """Telemetry BI BO khi nghen, khong xep hang: mot goi trang thai cu 5 giay la vo gia tri
        ma lai chiem bang thong cua goi hien tai (muc 5.3). Giu toi da MOT goi cho."""
        if self.tx_telemetry_cho >= 1:
            self.dem['tx_dropped'] += 1
            return
        self.tx_telemetry_cho += 1
        self.pos_valid = bool(msg.valid_flags & TelemetryPacket.VALID_POS)
        self.enqueue(PRIORITY_TELEMETRY, self.d.MAVLink_drone_telemetry_message(
            stamp_us=int(msg.stamp.sec * 1e6 + msg.stamp.nanosec / 1e3),
            mission_id=msg.mission_id, contract_ver=msg.contract_ver,
            lat=int(msg.lat * 1e7), lon=int(msg.lon * 1e7),
            marker_id_tracking=msg.marker_id_tracking, alt_m=msg.alt_m,
            vel_ned=list(msg.vel_ned), battery_pct=msg.battery_pct, battery_v=msg.battery_v,
            valid_flags=msg.valid_flags, status_flags=msg.status_flags,
            gcs_rssi_dbm=msg.gcs_rssi_dbm, mission_state=msg.mission_state,
            current_wp_index=msg.current_wp_index, wp_total=msg.wp_total,
            retry_count=msg.retry_count, gripper_state=msg.gripper_state,
            failsafe_type=msg.failsafe_type, expected_marker_id=msg.expected_marker_id,
            tagmap_crc=msg.tagmap_crc, home_n_mm=msg.home_n_mm, home_e_mm=msg.home_e_mm))

    def on_odom(self, msg):
        self.odom = msg

    def send_pos_att(self):
        """LOCAL_POSITION_NED (32) + ATTITUDE (30) 5 Hz. Muc 5.1.

        Dieu kien phat la POS_VALID VA KHONG GI KHAC (P20): "nam tren dat" khong suy ra
        POS_VALID = 0, va GCS CAN vi tri tren dat cho canh bao truoc MISSION_START (P18).

        Hai ban tin chuan nay KHONG co cho cho co hieu luc, nen quy tac la KHONG PHAT khi
        POS_VALID = 0. Khong phat = khong biet - cach duy nhat giu duoc R3 voi ban tin chuan.

        POS_VALID lay tu valid_flags cua goi telemetry moi nhat, KHONG tu tinh lai: de dinh nghia
        "da neo VA khoe VA con tuoi" chi nam o MOT cho (telemetry_aggregator_node).
        """
        if not self.pos_valid or self.odom is None:
            return
        p = self.odom.pose.pose.position
        v = self.odom.twist.twist.linear
        q = self.odom.pose.pose.orientation
        t_ms = int(self.now_s() * 1000) & 0xFFFFFFFF
        # ENU -> NED. N va E la truc BAN DO TAG, khong phai Bac/Dong dia ly (muc 5.2b y 2).
        self.enqueue(PRIORITY_TELEMETRY, self.d.MAVLink_local_position_ned_message(
            time_boot_ms=t_ms, x=float(p.y), y=float(p.x), z=float(-p.z),
            vx=float(v.y), vy=float(v.x), vz=float(-v.z)))
        # Quaternion -> roll/pitch/yaw. yaw do so voi truc N cua ban do tag.
        sr = 2.0 * (q.w * q.x + q.y * q.z)
        cr = 1.0 - 2.0 * (q.x * q.x + q.y * q.y)
        sp = max(-1.0, min(1.0, 2.0 * (q.w * q.y - q.z * q.x)))
        sy = 2.0 * (q.w * q.z + q.x * q.y)
        cy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        # ENU yaw (nguoc chieu kim dong ho tu truc x=E) -> NED yaw (tu truc N, thuan chieu).
        yaw_enu = math.atan2(sy, cy)
        self.enqueue(PRIORITY_TELEMETRY, self.d.MAVLink_attitude_message(
            time_boot_ms=t_ms, roll=math.atan2(sr, cr), pitch=math.asin(sp),
            yaw=math.atan2(math.cos(yaw_enu), math.sin(yaw_enu)),
            rollspeed=0.0, pitchspeed=0.0, yawspeed=0.0))

    def send_heartbeat(self):
        """1 Hz, KHONG BAO GIO TAT: giu anh xa NAT song. Mat anh xa la mat duong ve (muc 2.1)."""
        self.enqueue(PRIORITY_TELEMETRY - 1, self.d.MAVLink_heartbeat_message(
            type=self.d.MAV_TYPE_ONBOARD_CONTROLLER, autopilot=self.d.MAV_AUTOPILOT_INVALID,
            base_mode=0, custom_mode=0, system_status=self.d.MAV_STATE_ACTIVE, mavlink_version=3))

    def send_link_stats(self):
        self.enqueue(PRIORITY_TELEMETRY, self.d.MAVLink_drone_link_stats_message(
            rx_ok=self.dem['rx_ok'], rx_drop=self.dem['rx_drop'],
            rx_bad_crc=self.dem['rx_bad_crc'],
            rx_bad_sig=KHONG_DO_DUOC_32,        # chua bat chu ky goi (muc 7.6)
            tx_sent=self.dem['tx_sent'], tx_dropped=self.dem['tx_dropped'],
            queue_depth=self.tx_queue.qsize(),
            rtt_ms=KHONG_DO_DUOC_16))           # TIMESYNC chua hien thuc

    def statustext(self, muc, text):
        """Chia doan thay vi cat: chuoi ly do thuong dai hon 50 byte (muc 7.5)."""
        b = to_ascii(text, 250)
        for i in range(0, max(1, len(b)), 50):
            self.enqueue(PRIORITY_EMERGENCY if muc <= 2 else PRIORITY_TELEMETRY,
                         self.d.MAVLink_statustext_message(severity=muc, text=b[i:i + 50],
                                                           id=0, chunk_seq=i // 50))

    # ------------------------------------------------------------------ nhan

    def pump_rx(self):
        while True:
            try:
                data, addr = self.socket.recvfrom(1500)
            except (BlockingIOError, OSError):
                return
            for msg in self.ml.parse_buffer(data) or []:
                if msg.get_type() == 'BAD_DATA':
                    self.dem['rx_bad_crc'] += 1
                    continue
                if msg.get_srcSystem() != SYSID_GCS or msg.get_srcComponent() != COMPID_GCS:
                    continue            # bo qua IM LANG moi gui khac ngoai bang muc 2.2
                self.dem['rx_ok'] += 1
                if self.seq_gcs is not None:
                    mat = (msg.get_seq() - self.seq_gcs - 1) & 0xFF
                    self.dem['rx_drop'] += mat
                self.seq_gcs = msg.get_seq()
                self.last_rx_time = self.now_s()
                self.gcs_addr = addr    # goi HOP LE gan nhat, khong phai goi dau tien (muc 2.1)
                self.on_rx(msg)

    def on_rx(self, msg):
        t = msg.get_type()
        if t == 'DRONE_MISSION_COUNT':
            self.bat_dau_nap(msg)
        elif t == 'DRONE_MISSION_ITEM':
            self.nhan_item(msg)
        elif t == 'COMMAND_LONG':
            self.nhan_lenh(msg)

    # ------------------------------------------------------------------ nap ke hoach (muc 3)

    def ack_nap(self, mission_id, ket_qua, ly_do):
        self.enqueue(PRIORITY_MISSION, self.d.MAVLink_drone_mission_ack_message(
            mission_id=mission_id, result=ket_qua, reason=to_ascii(ly_do, 50)))
        self.nap = None

    def bat_dau_nap(self, msg):
        # Lech MAJOR -> TU CHOI nap ke hoach, nhung VAN phat telemetry va VAN nhan lenh khan
        # (muc 6.2): lenh ha canh la thu phai luon di duoc, ke ca khi hai ben hieu khac nhau ve
        # moi thu khac. contract_ver = 0 la GCS cu chua khai truong nay -> bo qua kiem, canh bao.
        major_gcs = getattr(msg, 'contract_ver', 0) // 10000
        if getattr(msg, 'contract_ver', 0) == 0:
            self.statustext(4, 'GCS khong khai contract_ver - khong kiem duoc lech phien ban')
        elif major_gcs != self.get_parameter('contract_major').value:
            self.ack_nap(msg.mission_id, self.d.DRONE_MISSION_ERR_CONTRACT,
                         f'MAJOR GCS = {major_gcs}, Pi = '
                         f'{self.get_parameter("contract_major").value}')
            return
        if not 1 <= msg.count <= SO_DIEM_TOI_DA:
            self.ack_nap(msg.mission_id, self.d.DRONE_MISSION_ERR_COUNT,
                         f'count = {msg.count}, phai trong 1..{SO_DIEM_TOI_DA}')
            return
        # COUNT moi khi dang nap thi HUY luot cu - GCS thao tac lai la hop le (muc 3.2).
        if self.nap is not None:
            self.get_logger().info(f'huy luot nap {self.nap["id"]}, bat dau luot {msg.mission_id}')
        self.nap = dict(id=msg.mission_id, count=msg.count, cho=0, diem={},
                        hoi_luc=self.now_s(), so_lan_hoi=1, header=msg)
        self.hoi_item()

    def hoi_item(self):
        self.enqueue(PRIORITY_MISSION, self.d.MAVLink_drone_mission_request_message(
            mission_id=self.nap['id'], seq=self.nap['cho']))

    def nhan_item(self, msg):
        n = self.nap
        if n is None or msg.mission_id != n['id'] or msg.seq != n['cho']:
            return          # goi trung do phat lai la binh thuong, bo qua khong tang bo dem
        n['diem'][msg.seq] = msg
        n['cho'] += 1
        n['so_lan_hoi'] = 1
        n['hoi_luc'] = self.now_s()
        if n['cho'] < n['count']:
            self.hoi_item()
        else:
            self.nap_xong()

    def nap_xong(self):
        n, h = self.nap, self.nap['header']
        plan = MissionPlan()
        plan.mission_id = n['id']
        plan.plan_name = h.plan_name.rstrip('\x00') if isinstance(h.plan_name, str) else ''
        plan.max_retries = h.max_retries
        plan.search_timeout_s = h.search_timeout_s
        plan.issued_stamp.sec = int(h.issued_stamp_us // 1_000_000)
        plan.issued_stamp.nanosec = int(h.issued_stamp_us % 1_000_000) * 1000
        for seq in sorted(n['diem']):
            it = n['diem'][seq]
            wp = MissionWaypoint()
            wp.seq = it.seq
            wp.expected_marker_id = it.expected_marker_id
            wp.action = it.action
            wp.alt_m = it.alt_m
            wp.acceptance_radius_m = it.acceptance_radius_m
            wp.max_vel_mps = it.max_vel_mps
            wp.loiter_s = it.loiter_s
            plan.waypoints.append(wp)
        self.pub_plan.publish(plan)
        # ACK chi gui SAU KHI mission_manager_node phan quyet (muc 3.2), khong gui som o day.
        self.cho_ack = n['id']
        self.nap = None
        self.get_logger().info(f'da chuyen ke hoach {plan.mission_id} ({len(plan.waypoints)} diem) '
                               'cho mission_manager_node, cho phan quyet')

    def on_plan_ack(self, msg):
        """Phan quyet cua mission_manager_node -> DRONE_MISSION_ACK. Chuoi ly do chuyen THANG
        len day khong sua (muc 3.3): no den tu ben duy nhat duoc phan quyet."""
        if self.cho_ack is None or msg.mission_id != self.cho_ack:
            return
        self.cho_ack = None
        if msg.accepted:
            self.enqueue(PRIORITY_MISSION, self.d.MAVLink_drone_mission_ack_message(
                mission_id=msg.mission_id, result=self.d.DRONE_MISSION_ACCEPTED, reason=b''))
            self.get_logger().info(f'ke hoach {msg.mission_id}: ACCEPTED')
            return
        ma = self.doan_ma_loi(msg.reason)
        self.enqueue(PRIORITY_MISSION, self.d.MAVLink_drone_mission_ack_message(
            mission_id=msg.mission_id, result=ma, reason=to_ascii(msg.reason, 50)))
        self.statustext(4, f'tu choi ke hoach {msg.mission_id}: {msg.reason}')
        self.get_logger().warning(f'ke hoach {msg.mission_id}: tu choi ({msg.reason})')

    @staticmethod
    def doan_ma_loi(ly_do):
        """Ma so cho may, CHUOI cho nguoi. Chuoi luon duoc chuyen nguyen van nen doan sai ma chi
        lam GCS mat phan loai, khong mat thong tin (muc 3.3)."""
        t = ly_do.lower()
        if 'tag' in t and ('khong co' in t or 'khong ton tai' in t):
            return 3        # ERR_UNKNOWN_TAG
        if 'dang' in t or 'chi nhan khi idle' in t or 'cho cat canh' in t:
            return 5        # ERR_BUSY
        return 4            # ERR_PARAM

    def check_nap_timeout(self):
        n = self.nap
        if n is None or self.now_s() - n['hoi_luc'] < ITEM_TIMEOUT_S:
            return
        if n['so_lan_hoi'] >= ITEM_HOI_LAI_TOI_DA:
            self.ack_nap(n['id'], self.d.DRONE_MISSION_ERR_TIMEOUT,
                         f'thieu diem {n["cho"]} sau {ITEM_HOI_LAI_TOI_DA} lan hoi')
            return
        n['so_lan_hoi'] += 1
        n['hoi_luc'] = self.now_s()
        self.hoi_item()

    # ------------------------------------------------------------------ lenh (muc 4)

    def ack_lenh(self, command, ket_qua):
        uu_tien = PRIORITY_EMERGENCY if command in LENH_KHAN else PRIORITY_MISSION
        self.enqueue(uu_tien, self.d.MAVLink_command_ack_message(
            command=command, result=ket_qua, progress=0, result_param2=0,
            target_system=SYSID_GCS, target_component=COMPID_GCS))

    def nhan_lenh(self, msg):
        """Bat bien theo TRANG THAI, khong theo (command, confirmation): co che phat lai chuan
        TANG confirmation moi lan nen khoa do khong bao gio trung (muc 4.2, loi P4)."""
        c = msg.command
        if c == CMD_ARM_DISARM and msg.param1 >= 0.5:
            # ARM tra DENIED chu khong UNSUPPORTED: lenh 400 CO duoc ho tro, chi hanh dong arm bi
            # tu choi vinh vien (muc 9.3). UNSUPPORTED se khien GCS tuong la loi phien ban.
            self.ack_lenh(c, ACK_DENIED)
            self.statustext(4, 'GCS khong duoc ARM: arm la quyet dinh tai cho, can nguoi nhin thay drone')
            return
        if c == CMD_PAUSE_CONTINUE:
            self.ack_lenh(c, ACK_UNSUPPORTED)      # da cap so, FSM chua hien thuc (muc 4.1)
            return
        if c not in (CMD_NAV_RTL, CMD_NAV_LAND, CMD_MISSION_START, CMD_ARM_DISARM, CMD_ABORT):
            self.ack_lenh(c, ACK_UNSUPPORTED)      # lenh la -> UNSUPPORTED, khong bao gio im lang
            return
        self.ack_lenh(c, ACK_ACCEPTED)
        self.get_logger().info(f'nhan lenh {c} -> ACCEPTED (chuyen cho FSM chua hien thuc)')

    # ------------------------------------------------------------------ watchdog (muc 9.2)

    def check_watchdog(self):
        self.check_nap_timeout()
        gioi_han = self.get_parameter('link_timeout_s').value
        song = self.last_rx_time is not None and self.now_s() - self.last_rx_time <= gioi_han
        if song != self.connected:
            self.connected = song
            self.get_logger().info(f'GCS {"co ket noi" if song else "MAT KET NOI"}')
        # Phat khi DOI trang thai VA dinh ky: failsafe_monitor_node co the restart va se lo moi
        # su kien truoc do (quy tac R5, muc 9.2).
        self.pub_connected.publish(Bool(data=bool(song)))

    def now_s(self):
        return self.get_clock().now().nanoseconds / 1e9


def main(args=None):
    rclpy.init(args=args)
    node = GcsLinkNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
