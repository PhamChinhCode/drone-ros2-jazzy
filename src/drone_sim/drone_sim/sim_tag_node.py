"""sim_tag_node - gia lap phat hien AprilTag tu ground truth cua Gazebo. CHI dung trong mo phong.

Thay ca chuoi camera -> image_proc -> apriltag_ros ma KHONG can camera: lay vi tri that cua
drone trong Gazebo, tinh pose tag trong khung anh, roi phat dung hai thu ma
landing_target_bridge_node can:

  /apriltag/detections            AprilTagDetectionArray (chi ID, dung de loc)
  TF camera_optical_frame -> <ten khung tag>

Dung dung chuoi TF cua drone that (base_link -> camera_link -> camera_optical_frame, so do lay
tu estimation.launch.py) nen phep lookupTransform cua bridge di qua y het duong that, ke ca sai
so do lap camera nghieng.

Tam nhin mo phong bang hinh chop: tag chi "thay duoc" khi nam trong FOV, trong khoang cach cho
phep, va khong bi che. KHONG mo phong: mo anh khi bay nhanh, thieu sang, tag bi loa, sai so PnP
theo goc nghieng. Vi vay tag o day DE thay hon ngoai doi - gain tune duoc chi la diem xuat phat.
"""

import math
import random

import numpy as np
import rclpy
from apriltag_msgs.msg import AprilTagDetection, AprilTagDetectionArray
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.exceptions import ParameterUninitializedException
from rclpy.node import Node
from rclpy.parameter import Parameter
from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster

from drone_control.qos import SENSOR_QOS
from drone_estimation.pad_map import build_pad_map, parse_headings

# Lay tu estimation.launch.py - phai khop voi drone that.
# z = -0,06: khop camera Gazebo gan duoi bung X3 (worlds/drone_tune.sdf, sim_launch.CAM_XYZ).
BASE_TO_CAM_XYZ = (0.09, 0.0, -0.06)        # do 2026-10-08: truoc tam 90 mm
BASE_TO_CAM_RPY = (0.0, 1.2217, 0.0)        # yaw, pitch, roll (pitch 70 do)
CAM_TO_OPTICAL_RPY = (-1.5708, 0.0, -1.5708)


def rpy_to_mat(yaw, pitch, roll):
    """Quy uoc ZYX giong static_transform_publisher (doi so: x y z yaw pitch roll)."""
    cy, sy = math.cos(yaw), math.sin(yaw)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cr, sr = math.cos(roll), math.sin(roll)
    return (np.array([[cy, -sy, 0.0], [sy, cy, 0.0], [0.0, 0.0, 1.0]])
            @ np.array([[cp, 0.0, sp], [0.0, 1.0, 0.0], [-sp, 0.0, cp]])
            @ np.array([[1.0, 0.0, 0.0], [0.0, cr, -sr], [0.0, sr, cr]]))


def quat_to_mat(x, y, z, w):
    n = math.sqrt(x * x + y * y + z * z + w * w) or 1.0
    x, y, z, w = x / n, y / n, z / n, w / n
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def mat_to_quat(m):
    tr = m[0, 0] + m[1, 1] + m[2, 2]
    if tr > 0.0:
        s = math.sqrt(tr + 1.0) * 2.0
        return ((m[2, 1] - m[1, 2]) / s, (m[0, 2] - m[2, 0]) / s, (m[1, 0] - m[0, 1]) / s, 0.25 * s)
    i = int(np.argmax([m[0, 0], m[1, 1], m[2, 2]]))
    j, k = (i + 1) % 3, (i + 2) % 3
    s = math.sqrt(m[i, i] - m[j, j] - m[k, k] + 1.0) * 2.0
    q = [0.0, 0.0, 0.0]
    q[i], q[j], q[k] = 0.25 * s, (m[j, i] + m[i, j]) / s, (m[k, i] + m[i, k]) / s
    return (q[0], q[1], q[2], (m[k, j] - m[j, k]) / s)


class SimTagNode(Node):

    def __init__(self):
        super().__init__('sim_tag_node')
        self.declare_parameter('odom_topic', '/model/X3/odometry')
        self.declare_parameter('publish_rate_hz', 30.0)      # apriltag that do duoc ~29 Hz (7.1)
        # Chieu bang noi tai THAT (camera_info Pi 5, 2026-10-08): cy = 180 nen FOV doc LECH - nhin
        # xuong 36,3 do nhung chi len 31,1 do so voi truc quang. Hinh chop doi xung +-32 do cu (fx
        # 323,6) cho thay tag gan duoi than lau hon thuc te.
        self.declare_parameter('fx', 299.94)
        self.declare_parameter('fy', 298.75)
        self.declare_parameter('cx', 321.58)
        self.declare_parameter('cy', 180.21)
        self.declare_parameter('image_width', 640)
        self.declare_parameter('image_height', 400)
        # Ca tag (canh tag_size_m; tag nho: pad_small_tag_size_m) phai nam trong anh va canh
        # >= min_tag_px moi tinh la thay.
        self.declare_parameter('tag_size_m', 0.25)
        self.declare_parameter('min_tag_px', 20.0)
        self.declare_parameter('max_range_m', 8.0)
        self.declare_parameter('min_range_m', 0.15)
        self.declare_parameter('noise_m', 0.0)               # nhieu pose tag, gia lap sai so PnP
        self.declare_parameter('dropout_prob', 0.0)          # ti le khung bi mat detection
        self.declare_parameter('known_tags', Parameter.Type.DOUBLE_ARRAY)
        self.declare_parameter('tag_frames', Parameter.Type.STRING_ARRAY)
        # Bai hai tag (tags.yaml): bai co huong thi co tag nho id + offset, truoc tam forward_m.
        self.declare_parameter('known_tags_heading', Parameter.Type.DOUBLE_ARRAY)
        self.declare_parameter('pad_small_tag_id_offset', 10)
        self.declare_parameter('pad_small_tag_forward_m', 0.21)
        self.declare_parameter('pad_small_tag_size_m', 0.10)

        g = self.get_parameter
        try:
            headings = parse_headings(g('known_tags_heading').value)
        except ParameterUninitializedException:
            headings = {}
        pads = build_pad_map(g('known_tags').value, g('tag_frames').value, headings,
                             g('pad_small_tag_id_offset').value, g('pad_small_tag_forward_m').value)
        # id -> (vi tri, ten khung, canh, ma tran xoay tag -> the gioi theo huong bai)
        self.tags = {i: (np.array(t.pos), t.frame,
                         g('pad_small_tag_size_m').value if t.small else g('tag_size_m').value,
                         rpy_to_mat(t.yaw or 0.0, 0.0, 0.0))
                     for i, t in pads.items()}
        self.get_logger().info('tag mo phong: ' + ', '.join(
            f'{i} @ {tuple(np.round(p, 3))} ({f}, {s:.2f} m)'
            for i, (p, f, s, _) in self.tags.items()))

        # base_link -> camera_optical_frame, gop san (khong doi trong luc chay).
        r_cam = rpy_to_mat(*BASE_TO_CAM_RPY)
        self.r_base_opt = r_cam @ rpy_to_mat(*CAM_TO_OPTICAL_RPY)
        self.t_base_opt = np.array(BASE_TO_CAM_XYZ)

        self.odom = None
        self.tf_static = StaticTransformBroadcaster(self)
        self.tf_static.sendTransform([
            self.make_tf('base_link', 'camera_link', self.t_base_opt, rpy_to_mat(*BASE_TO_CAM_RPY)),
            self.make_tf('camera_link', 'camera_optical_frame', np.zeros(3),
                         rpy_to_mat(*CAM_TO_OPTICAL_RPY))])
        self.tf_bc = TransformBroadcaster(self)
        self.pub_det = self.create_publisher(
            AprilTagDetectionArray, '/apriltag/detections', SENSOR_QOS)

        self.create_subscription(Odometry, self.get_parameter('odom_topic').value,
                                 self.on_odom, SENSOR_QOS)
        self.create_timer(1.0 / self.get_parameter('publish_rate_hz').value, self.tick)

    def make_tf(self, parent, child, t, r):
        tf = TransformStamped()
        tf.header.stamp = self.get_clock().now().to_msg()
        tf.header.frame_id, tf.child_frame_id = parent, child
        tf.transform.translation.x, tf.transform.translation.y, tf.transform.translation.z = t
        q = mat_to_quat(r)
        (tf.transform.rotation.x, tf.transform.rotation.y,
         tf.transform.rotation.z, tf.transform.rotation.w) = q
        return tf

    def on_odom(self, msg):
        self.odom = msg

    def visible(self, v_opt, size_m):
        """v_opt: vi tri tag trong khung anh (x phai, y xuong, z theo huong nhin).

        Chieu tam tag bang K that; tag thay duoc khi ca o vuong canh = kich thuoc tag (theo pixel)
        nam trong anh va du lon. Gan dung: bo qua meo ong kinh va do nghieng cua mat tag.
        """
        g = self.get_parameter
        z = v_opt[2]
        if z <= 0.0:
            return False                      # tag o phia sau camera
        d = float(np.linalg.norm(v_opt))
        if not (g('min_range_m').value <= d <= g('max_range_m').value):
            return False
        u = g('cx').value + g('fx').value * v_opt[0] / z
        v = g('cy').value + g('fy').value * v_opt[1] / z
        half = 0.5 * g('fx').value * size_m / d
        return (2.0 * half >= g('min_tag_px').value
                and half <= u <= g('image_width').value - half
                and half <= v <= g('image_height').value - half)

    def tick(self):
        if self.odom is None:
            return
        p = self.odom.pose.pose.position
        o = self.odom.pose.pose.orientation
        r_wb = quat_to_mat(o.x, o.y, o.z, o.w)       # than -> the gioi
        p_w = np.array([p.x, p.y, p.z])
        noise = self.get_parameter('noise_m').value
        dropout = self.get_parameter('dropout_prob').value

        stamp = self.get_clock().now().to_msg()
        det_msg = AprilTagDetectionArray()
        det_msg.header.stamp = stamp
        det_msg.header.frame_id = 'camera_optical_frame'
        tfs = []
        for tag_id, (p_tag_w, frame, size_m, r_w_tag) in self.tags.items():
            v_base = r_wb.T @ (p_tag_w - p_w)                 # tag trong base_link
            v_opt = self.r_base_opt.T @ (v_base - self.t_base_opt)
            if not self.visible(v_opt, size_m) or random.random() < dropout:
                continue
            if noise > 0.0:
                v_opt = v_opt + np.array([random.gauss(0.0, noise) for _ in range(3)])
            det = AprilTagDetection()
            det.id = tag_id
            det_msg.detections.append(det)
            tfs.append(self.make_tf('camera_optical_frame', frame, v_opt,
                                    self.r_base_opt.T @ r_wb.T @ r_w_tag))
        if tfs:
            self.tf_bc.sendTransform(tfs)
        self.pub_det.publish(det_msg)


def main(args=None):
    rclpy.init(args=args)
    node = SimTagNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
