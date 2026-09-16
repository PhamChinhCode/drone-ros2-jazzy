"""telemetry_aggregator_node - gop nhieu nguon thanh dung mot goi TelemetryPacket.

Tach khoi gcs_link_node de logic gop du lieu khong lan voi logic ma hoa/giai ma giao thuc.

Publish o TAN SO CO DINH bang timer (mac dinh 2 Hz), KHONG publish theo su kien cua tung
nguon - tranh lam ngap kenh 4G/radio bang thong hep.
"""

import rclpy
from mavros_msgs.msg import DebugValue, State
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.parameter import Parameter
from sensor_msgs.msg import BatteryState, NavSatFix

from rclpy.qos import QoSProfile, QoSReliabilityPolicy

from drone_comms.qos import EVENT_QOS, SENSOR_QOS
from drone_comms.tagmap import doc_known_tags, tagmap_crc
from drone_interfaces.msg import (EkfHealth, FailsafeEvent, GripperStatus, MarkerQuality,
                                  MissionState, TelemetryPacket)

P = TelemetryPacket

# Kenh NAMED_VALUE_INT ghep NHIEU TEN vao mot topic: FC phat ca chum 8 ten trong mot
# nhip. Hang doi depth 1 chi giu duoc ten CUOI cua chum, va OB_AUTH lai la ten DAU -
# no bi rot gan nhu moi lan. Phai du sau cho ca chum (phat hien o nghiem thu 10.B).
NAMED_VALUE_QOS = QoSProfile(depth=20, reliability=QoSReliabilityPolicy.BEST_EFFORT)


class TelemetryAggregatorNode(Node):

    def __init__(self):
        super().__init__('telemetry_aggregator_node')

        self.declare_parameter('publish_rate_hz', 2.0)
        # Qua han thi HA CO, khong giu gia tri cu: mot so cu 5 giay la mot loi noi doi (R3).
        self.declare_parameter('stale_s', 2.0)
        self.declare_parameter('contract_ver', 500)     # ban 0.5 = 0*10000 + 5*100
        self.declare_parameter('known_tags', Parameter.Type.DOUBLE_ARRAY)

        # Moi nguon giu (ban_tin, thoi_diem_nhan) de xet do tuoi rieng tung nguon.
        self.nguon = {}
        try:
            self.tagmap_crc = tagmap_crc(doc_known_tags(self.get_parameter('known_tags').value))
            self.get_logger().info(f'tagmap_crc = 0x{self.tagmap_crc:08X}')
        except Exception as e:
            self.tagmap_crc = 0
            self.get_logger().error(f'khong tinh duoc tagmap_crc ({e}) - GCS se khoa nap ke hoach')

        self.create_subscription(MissionState, '/mission/state', self.on_mission_state, EVENT_QOS)
        self.create_subscription(State, '/mavros/state', self.on_fc_state, EVENT_QOS)
        self.create_subscription(BatteryState, '/mavros/battery', self.on_battery, SENSOR_QOS)
        self.create_subscription(
            NavSatFix, '/mavros/global_position/global', self.on_global_pos, SENSOR_QOS)
        self.create_subscription(GripperStatus, '/gripper/status', self.on_gripper, EVENT_QOS)
        self.create_subscription(
            MarkerQuality, '/marker/tracking_quality', self.on_marker, EVENT_QOS)
        self.create_subscription(FailsafeEvent, '/failsafe_event', self.on_failsafe, EVENT_QOS)
        self.create_subscription(Odometry, '/odometry/filtered', self.on_odom, SENSOR_QOS)
        self.create_subscription(EkfHealth, '/ekf/health', self.on_ekf, EVENT_QOS)
        # OB_AUTH: nguoi lai da trao quyen cho Pi qua ch5/ch8 chua (giao uoc FC 6.3).
        # Thieu bit nay thi nguoi van hanh KHONG HIEU vi sao MISSION_START bi tu choi -
        # nguyen nhan thuong gap nhat lai la thu GCS khong nhin thay duoc.
        self.create_subscription(DebugValue, '/mavros/debug_value/named_value_int',
                                 self.on_named_value, NAMED_VALUE_QOS)

        self.pub_telemetry = self.create_publisher(
            TelemetryPacket, '/telemetry/outgoing', EVENT_QOS)
        self.create_timer(1.0 / self.get_parameter('publish_rate_hz').value, self.publish_packet)

    def ghi(self, ten, msg):
        self.nguon[ten] = (msg, self.get_clock().now().nanoseconds / 1e9)

    def lay(self, ten):
        """Tra ban tin neu con tuoi, None neu qua han hoac chua bao gio nhan."""
        v = self.nguon.get(ten)
        if v is None:
            return None
        msg, t = v
        return msg if self.now_s() - t <= self.get_parameter('stale_s').value else None

    def now_s(self):
        return self.get_clock().now().nanoseconds / 1e9

    def on_mission_state(self, msg):
        self.ghi('mission', msg)

    def on_fc_state(self, msg):
        self.ghi('fc', msg)

    def on_battery(self, msg):
        self.ghi('battery', msg)

    def on_global_pos(self, msg):
        # status < 0 (NO_FIX): bo mach khong co GPS -> lat/lon = 0 va altitude la do cao ap suat
        # rac. Bo qua de GCS khong hien toa do 0,0 o do cao ~37 m.
        if msg.status.status >= 0:
            self.ghi('gps', msg)

    def on_gripper(self, msg):
        self.ghi('gripper', msg)

    def on_marker(self, msg):
        self.ghi('marker', msg)

    def on_failsafe(self, msg):
        self.ghi('failsafe', msg)

    def on_odom(self, msg):
        self.ghi('odom', msg)

    def on_ekf(self, msg):
        self.ghi('ekf', msg)

    def on_named_value(self, msg):
        if msg.name == 'OB_AUTH':
            self.ghi('ob_auth', msg)


    def read_rssi_dbm(self):
        """TODO: doc RSSI modem 4G qua AT command/ModemManager.

        Tra None = CHUA DOC DUOC, khong tra 0: 0 dBm la mot muc tin hieu that (rat manh), nen
        dien 0 la noi doi. Bit RSSI_VALID ha xuong (R3, giao uoc P10).
        """
        return None

    def publish_packet(self):
        """Gop thanh mot TelemetryPacket va publish o TAN SO CO DINH.

        Phat dung nhip KE CA khi khong nguon nao cap nhat (quy tac R5: trang thai phai hoi lai
        duoc, vi GCS co the khoi dong lai bat cu luc nao va se lo moi su kien truoc do).

        Nguon thieu thi de gia tri mac dinh va HA CO tuong ung - tuyet doi khong thay bang mot
        so "an toan" (R3: "0 la mot loi noi doi khac"). Day la phan de sai nhat cua ca node.
        """
        m = TelemetryPacket()
        m.stamp = self.get_clock().now().to_msg()
        m.contract_ver = int(self.get_parameter('contract_ver').value)
        m.tagmap_crc = self.tagmap_crc
        m.marker_id_tracking = -1
        m.expected_marker_id = -1
        co = 0
        trang_thai = 0

        ekf = self.lay('ekf')
        odom = self.lay('odom')
        if ekf is not None and ekf.healthy:
            co |= P.VALID_EKF_HEALTHY
        # POS_VALID = da NEO theo bang tag VA EKF khoe VA odom con tuoi (giao uoc muc 5.2b y 1).
        # Chua neo thi toa do thuoc khung cho EKF khoi dong, khong phai goc ban do tag.
        if odom is not None and ekf is not None and ekf.anchored and ekf.healthy:
            co |= P.VALID_POS
            p = odom.pose.pose.position
            t = odom.twist.twist.linear
            m.alt_m = float(p.z)
            m.vel_ned = [float(t.y), float(t.x), float(-t.z)]     # ENU -> NED
        fc = self.lay('fc')
        if fc is not None and fc.connected:
            co |= P.VALID_FC_LINK
            if fc.armed:
                trang_thai |= P.STATUS_ARMED
        bat = self.lay('battery')
        if bat is not None and bat.percentage >= 0.0:
            co |= P.VALID_BATTERY
            m.battery_pct = float(bat.percentage * 100.0)
            m.battery_v = float(bat.voltage)
        gps = self.lay('gps')
        if gps is not None:
            co |= P.VALID_GLOBAL_POS
            m.lat, m.lon = float(gps.latitude), float(gps.longitude)
        grip = self.lay('gripper')
        if grip is not None:
            co |= P.VALID_GRIPPER
            m.gripper_state = grip.state
            if grip.state == GripperStatus.GRIP_STATE_CLOSED:
                trang_thai |= P.STATUS_CARRYING
        mk = self.lay('marker')
        if mk is not None and mk.visible:
            co |= P.VALID_MARKER
            m.marker_id_tracking = mk.marker_id
        mis = self.lay('mission')
        if mis is not None:
            m.mission_id = mis.mission_id
            m.mission_state = mis.state
            m.flight_result = mis.mission_result
            m.current_wp_index = mis.current_wp_index
            m.wp_total = mis.wp_total
            m.retry_count = mis.retry_count
            m.expected_marker_id = mis.expected_marker_id
            if mis.home_valid:
                co |= P.VALID_HOME
                m.home_n_mm = int(round(mis.home_odom[1] * 1000))   # ENU y -> NED n
                m.home_e_mm = int(round(mis.home_odom[0] * 1000))   # ENU x -> NED e
        fs = self.lay('failsafe')
        m.failsafe_type = fs.type if fs is not None and fs.active else FailsafeEvent.FS_NONE
        rssi = self.read_rssi_dbm()
        if rssi is not None:
            co |= P.VALID_RSSI
            m.gcs_rssi_dbm = int(rssi)

        auth = self.lay('ob_auth')
        if auth is not None and auth.value_int:
            trang_thai |= P.STATUS_PI_HAS_AUTHORITY
        m.valid_flags = co
        m.status_flags = trang_thai
        self.pub_telemetry.publish(m)


def main(args=None):
    rclpy.init(args=args)
    node = TelemetryAggregatorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
