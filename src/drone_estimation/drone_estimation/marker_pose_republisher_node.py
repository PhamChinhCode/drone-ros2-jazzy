"""marker_pose_republisher_node - dua pose marker vao khung odom cho EKF.

apriltag_ros cho pose marker trong khung quang hoc cua camera. robot_localization can
PoseWithCovarianceStamped trong khung odom va KHONG tu lam buoc chuyen nay.

Diem de sai nhat: nham chieu transform camera <-> body. Trieu chung dac trung la
"thay dung marker nhung bay lech tam".

AprilTagDetection KHONG mang pose: apriltag_ros phat pose tag qua TF (khung anh -> ten trong
tag.frames cua apriltag.yaml). Node nay tra TF base_link -> tag tai dung stamp anh, xoay bang huong
odom -> base_link (tu IMU qua EKF) va tru vao toa do da biet cua tag.
"""

import math

import rclpy
from apriltag_msgs.msg import AprilTagDetectionArray
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import Buffer, TransformException, TransformListener

from drone_estimation.estimation_math import drone_position_from_tag, parse_known_tags
from drone_estimation.qos import SENSOR_QOS

TF_TIMEOUT = Duration(seconds=0.05)
LARGE_VARIANCE = 1e6   # truc khong do (goc) - EKF chi fuse x, y, z cua pose0


class MarkerPoseRepublisherNode(Node):

    def __init__(self):
        super().__init__('marker_pose_republisher_node')

        # Marker cho vi tri TUYET DOI nen covariance thap (tin nhieu) - nguoc voi optical flow.
        self.declare_parameter('pos_covariance', 0.01)      # m^2
        self.declare_parameter('target_frame', 'odom')
        # Toa do da biet cua tung bai dap trong khung target_frame, dang phang [id, x, y, z, ...].
        self.declare_parameter('known_tags', [0.0])
        # Ten khung TF tung tag, CUNG THU TU voi known_tags - khop tag.frames trong apriltag.yaml.
        self.declare_parameter('tag_frames', [''])
        self.declare_parameter('base_frame', 'base_link')

        self.known_tags = parse_known_tags(self.get_parameter('known_tags').value)
        frames = self.get_parameter('tag_frames').value
        if len(frames) != len(self.known_tags):
            raise ValueError(
                f'tag_frames co {len(frames)} ten, known_tags co {len(self.known_tags)} tag')
        self.tag_frames = dict(zip(self.known_tags.keys(), frames))
        self.warned = set()

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.create_subscription(
            AprilTagDetectionArray, '/apriltag/detections', self.on_detections, SENSOR_QOS)
        self.pub_pose = self.create_publisher(
            PoseWithCovarianceStamped, '/marker/pose_odom', SENSOR_QOS)

    def on_detections(self, msg):
        """Moi detection co trong known_tags -> mot pose drone trong target_frame."""
        target = self.get_parameter('target_frame').value
        base = self.get_parameter('base_frame').value
        var = self.get_parameter('pos_covariance').value
        stamp = Time.from_msg(msg.header.stamp)
        for det in msg.detections:
            if det.id not in self.known_tags:
                continue
            try:
                # Tag trong than may tai dung stamp anh (TF tag do apriltag_ros phat cung stamp).
                t_tag = self.tf_buffer.lookup_transform(base, self.tag_frames[det.id], stamp,
                                                        TF_TIMEOUT)
                # Huong than may: moi nhat la du, huong doi cham hon nhieu so voi vi tri.
                t_att = self.tf_buffer.lookup_transform(target, base, Time(), TF_TIMEOUT)
            except TransformException as exc:
                self.warn_once(f'tf{det.id}', f'khong tra duoc TF cho tag {det.id}: {exc}')
                continue
            tr, q = t_tag.transform.translation, t_att.transform.rotation
            pos = drone_position_from_tag(self.known_tags[det.id], (tr.x, tr.y, tr.z),
                                          (q.x, q.y, q.z, q.w))
            if not all(math.isfinite(v) for v in pos):
                continue

            out = PoseWithCovarianceStamped()
            out.header.stamp = msg.header.stamp
            out.header.frame_id = target
            out.pose.pose.position.x, out.pose.pose.position.y, out.pose.pose.position.z = pos
            out.pose.pose.orientation = q
            cov = [0.0] * 36
            for i in range(6):
                cov[i * 7] = var if i < 3 else LARGE_VARIANCE
            out.pose.covariance = cov
            self.pub_pose.publish(out)

    def warn_once(self, key, text):
        if key not in self.warned:
            self.warned.add(key)
            self.get_logger().warning(text)


def main(args=None):
    rclpy.init(args=args)
    node = MarkerPoseRepublisherNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
