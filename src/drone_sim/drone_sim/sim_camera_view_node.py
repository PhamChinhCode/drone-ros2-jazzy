"""sim_camera_view_node - cua so xem anh camera drone khi mo phong. CHI dung trong mo phong.

Ve len anh /camera/image_raw (camera Gazebo gan tren link servo, nen anh nghieng that theo goc):
  - goc camera /camera/tilt (do, + ve mui; "dang quay" khi NaN) va khoang cach ToF;
  - trang thai nhiem vu + chi tiet (/mission/state);
  - khung + id moi tag apriltag_ros dang giai ma (tag to xanh, tag hop cam), tam anh (cx, cy).
Mo cua so OpenCV tren DISPLAY cua launch (sim chay tren ubt: man hinh :1).
"""

import math

import cv2
import numpy as np
import rclpy
from apriltag_msgs.msg import AprilTagDetectionArray
from rclpy.node import Node
from sensor_msgs.msg import Image, Range
from std_msgs.msg import Float32

from drone_control.qos import EVENT_QOS, SENSOR_QOS
from drone_interfaces.msg import MissionState

TEN_TRANG_THAI = {getattr(MissionState, n): n for n in (
    'IDLE', 'TAKEOFF', 'ENROUTE', 'MARKER_SEARCH', 'PRECISION_LAND', 'ACTUATE_GRIPPER',
    'RETRY_LOITER', 'RTH', 'EMERGENCY_LAND', 'MISSION_COMPLETE', 'FAILSAFE', 'ALIGN_HEADING',
    'PAD_ALIGN', 'FINAL_APPROACH')}
WINDOW = 'Camera drone (mo phong)'
XANH, CAM, VANG = (60, 220, 60), (0, 150, 255), (0, 230, 255)


class SimCameraViewNode(Node):

    def __init__(self):
        super().__init__('sim_camera_view_node')
        self.declare_parameter('rate_hz', 15.0)
        self.declare_parameter('scale', 1.5)        # phong to anh 640 x 400 cho de nhin
        self.declare_parameter('cx', 321.58)        # tam anh that (camera_info Pi 5)
        self.declare_parameter('cy', 180.21)
        # Cho mo cua so (px man hinh): canh phai cua so Gazebo, de khong bi che.
        self.declare_parameter('win_x', 1000)
        self.declare_parameter('win_y', 60)
        self.img = self.tilt = self.range_m = self.state = None
        self.dets = []
        self.create_subscription(Image, '/camera/image_raw', self.on_image, SENSOR_QOS)
        self.create_subscription(Float32, '/camera/tilt', self.on_tilt, EVENT_QOS)
        self.create_subscription(Range, '/range/vertical', self.on_range, SENSOR_QOS)
        self.create_subscription(AprilTagDetectionArray, '/apriltag/detections', self.on_dets,
                                 SENSOR_QOS)
        self.create_subscription(MissionState, '/mission/state', self.on_state, EVENT_QOS)
        cv2.namedWindow(WINDOW, cv2.WINDOW_NORMAL)
        s = self.get_parameter('scale').value
        cv2.resizeWindow(WINDOW, int(640 * s), int(400 * s))
        cv2.moveWindow(WINDOW, self.get_parameter('win_x').value, self.get_parameter('win_y').value)
        cv2.setWindowProperty(WINDOW, cv2.WND_PROP_TOPMOST, 1)     # luon noi tren cua so Gazebo
        self.create_timer(1.0 / self.get_parameter('rate_hz').value, self.draw)

    def on_image(self, msg):
        a = np.frombuffer(msg.data, np.uint8).reshape(msg.height, msg.step)
        if msg.encoding in ('mono8', '8UC1'):
            self.img = cv2.cvtColor(a[:, :msg.width], cv2.COLOR_GRAY2BGR)
        else:                                       # rgb8 / bgr8
            img = a[:, :msg.width * 3].reshape(msg.height, msg.width, 3)
            self.img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR) if msg.encoding == 'rgb8' else img

    def on_tilt(self, msg):
        self.tilt = msg.data

    def on_range(self, msg):
        self.range_m = msg.range if msg.min_range <= msg.range <= msg.max_range else None

    def on_dets(self, msg):
        self.dets = [(d.id, [(c.x, c.y) for c in d.corners]) for d in msg.detections]

    def on_state(self, msg):
        self.state = msg

    def draw(self):
        if self.img is None:
            return
        s = self.get_parameter('scale').value
        img = cv2.resize(self.img, None, fx=s, fy=s, interpolation=cv2.INTER_NEAREST)
        g = self.get_parameter
        c = (int(g('cx').value * s), int(g('cy').value * s))
        cv2.drawMarker(img, c, VANG, cv2.MARKER_CROSS, 24, 1)
        for tag_id, corners in self.dets:
            mau = CAM if tag_id >= 10 else XANH
            pts = np.array([(x * s, y * s) for x, y in corners], np.int32)
            cv2.polylines(img, [pts], True, mau, 2)
            cv2.putText(img, f'id {tag_id}' + (' (hop)' if tag_id >= 10 else ''),
                        tuple(pts[0]), cv2.FONT_HERSHEY_SIMPLEX, 0.6, mau, 2)
        if self.tilt is None:
            goc = 'camera: --'
        elif math.isnan(self.tilt):
            goc = 'camera: DANG QUAY'
        else:
            huong = 've mui' if self.tilt > 0.5 else 've duoi' if self.tilt < -0.5 else 'thang xuong'
            goc = f'camera: {self.tilt:+.1f} do ({huong})'
        tof = 'ToF: --' if self.range_m is None else f'ToF: {self.range_m:.2f} m'
        st = self.state
        dong = [f'{goc}   {tof}',
                'nhiem vu: --' if st is None else
                f'nhiem vu: {TEN_TRANG_THAI.get(st.state, st.state)}  diem {st.current_wp_index}',
                '' if st is None else st.detail[:70]]
        for i, t in enumerate(dong):
            y = 26 + i * 26
            cv2.putText(img, t, (10, y), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 0, 0), 4)
            cv2.putText(img, t, (10, y), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (255, 255, 255), 1)
        cv2.imshow(WINDOW, img)
        cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)
    node = SimCameraViewNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.try_shutdown()
