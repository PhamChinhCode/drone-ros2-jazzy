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
from rclpy.node import Node
from rclpy.parameter import Parameter
from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster

from drone_control.qos import SENSOR_QOS

# Lay tu estimation.launch.py - phai khop voi drone that.
BASE_TO_CAM_XYZ = (0.06, 0.0, 0.0)
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
        # Nua goc FOV suy tu hieu chinh 640x400 (fx 323.62, fy 320.33): atan(320/fx), atan(200/fy)
        self.declare_parameter('hfov_half_deg', 44.7)
        self.declare_parameter('vfov_half_deg', 32.0)
        self.declare_parameter('max_range_m', 8.0)
        self.declare_parameter('min_range_m', 0.15)
        self.declare_parameter('noise_m', 0.0)               # nhieu pose tag, gia lap sai so PnP
        self.declare_parameter('dropout_prob', 0.0)          # ti le khung bi mat detection
        self.declare_parameter('known_tags', Parameter.Type.DOUBLE_ARRAY)
        self.declare_parameter('tag_frames', Parameter.Type.STRING_ARRAY)

        flat = self.get_parameter('known_tags').value
        frames = self.get_parameter('tag_frames').value
        self.tags = {int(flat[i * 4]): (np.array(flat[i * 4 + 1:i * 4 + 4]), frames[i])
                     for i in range(len(frames))}
        self.get_logger().info(
            'tag mo phong: ' + ', '.join(f'{i} @ {tuple(p)} ({f})' for i, (p, f) in self.tags.items()))

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

    def visible(self, v_opt):
        """v_opt: vi tri tag trong khung anh (x phai, y xuong, z theo huong nhin)."""
        z = v_opt[2]
        if z <= 0.0:
            return False                      # tag o phia sau camera
        d = float(np.linalg.norm(v_opt))
        if not (self.get_parameter('min_range_m').value <= d
                <= self.get_parameter('max_range_m').value):
            return False
        return (abs(v_opt[0]) <= math.tan(math.radians(self.get_parameter('hfov_half_deg').value)) * z
                and abs(v_opt[1]) <= math.tan(math.radians(self.get_parameter('vfov_half_deg').value)) * z)

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
        for tag_id, (p_tag_w, frame) in self.tags.items():
            v_base = r_wb.T @ (p_tag_w - p_w)                 # tag trong base_link
            v_opt = self.r_base_opt.T @ (v_base - self.t_base_opt)
            if not self.visible(v_opt) or random.random() < dropout:
                continue
            if noise > 0.0:
                v_opt = v_opt + np.array([random.gauss(0.0, noise) for _ in range(3)])
            det = AprilTagDetection()
            det.id = tag_id
            det_msg.detections.append(det)
            tfs.append(self.make_tf('camera_optical_frame', frame, v_opt,
                                    self.r_base_opt.T @ r_wb.T))
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
