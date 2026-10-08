"""auto_exposure_node - giu do sang anh camera on dinh trong nha lan ngoai troi.

Sensor khong co auto-exposure (doc thang V4L2, khong qua libcamera IPA). Truoc day phai chon tay
CAM_PROFILE trong drone_startup.sh; quen doi thi anh trong nha gan den (do 2026-10-08: profile
ngoai troi 40/16 cho mean 6,4/255, optical flow bam nhieu, chi phat 6-12 Hz). Node nay doc
/camera/image_raw, chinh exposure/gain cua sensor qua ioctl tren v4l-subdev.
"""

from concurrent.futures import ThreadPoolExecutor
import fcntl
import glob
import os
import signal
import struct
import threading

import numpy as np
import rclpy
from rclpy.experimental import EventsExecutor
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from sensor_msgs.msg import Image

from drone_perception.auto_exposure import ExposureLimits, image_stats, next_setting
from drone_perception.qos import SENSOR_QOS

# linux/videodev2.h: VIDIOC_G_CTRL / VIDIOC_S_CTRL, struct v4l2_control {__u32 id; __s32 value;}
VIDIOC_G_CTRL = 0xC008561B
VIDIOC_S_CTRL = 0xC008561C
V4L2_CID_EXPOSURE = 0x00980911
V4L2_CID_ANALOGUE_GAIN = 0x009E0903


def find_subdev(sensor_name):
    """/dev/v4l-subdevN cua sensor, tim theo ten trong sysfs (so N doi theo bo va overlay)."""
    for name_file in sorted(glob.glob('/sys/class/video4linux/v4l-subdev*/name')):
        with open(name_file) as f:
            if f.read().startswith(sensor_name):
                return '/dev/' + os.path.basename(os.path.dirname(name_file))
    return None


class AutoExposureNode(Node):

    def __init__(self):
        super().__init__('auto_exposure_node')
        d = ExposureLimits()
        self.declare_parameter('sensor_name', 'ov9281')
        self.declare_parameter('rate_hz', 5.0)
        self.declare_parameter('target_mean', d.target_mean)
        self.declare_parameter('deadband', d.deadband)
        self.declare_parameter('max_saturated', d.max_saturated)
        self.declare_parameter('exposure_min', d.exposure_min)
        self.declare_parameter('exposure_max', d.exposure_max)
        self.declare_parameter('gain_min', d.gain_min)
        self.declare_parameter('gain_max', d.gain_max)
        p = self.get_parameter
        self.limits = ExposureLimits(
            target_mean=p('target_mean').value, deadband=p('deadband').value,
            max_saturated=p('max_saturated').value,
            exposure_min=p('exposure_min').value, exposure_max=p('exposure_max').value,
            gain_min=p('gain_min').value, gain_max=p('gain_max').value)
        self.period_ns = int(1e9 / p('rate_hz').value)
        self.last_ns = 0

        sensor = p('sensor_name').value
        path = find_subdev(sensor)
        if path is None:
            # Gazebo / may khong co camera that: dung yen, khong lam gi anh.
            self.get_logger().warn(f"Khong thay subdev '{sensor}' - tat auto exposure")
            return
        self.fd = os.open(path, os.O_RDWR)
        self.exposure = self._get(V4L2_CID_EXPOSURE)
        self.gain = self._get(V4L2_CID_ANALOGUE_GAIN)
        self.get_logger().info(f'{path}: bat dau exposure={self.exposure} gain={self.gain}')
        self.create_subscription(Image, '/camera/image_raw', self.on_image, SENSOR_QOS)

    def _get(self, cid):
        buf = bytearray(struct.pack('Ii', cid, 0))
        fcntl.ioctl(self.fd, VIDIOC_G_CTRL, buf)
        return struct.unpack('Ii', buf)[1]

    def _set(self, cid, value):
        fcntl.ioctl(self.fd, VIDIOC_S_CTRL, bytearray(struct.pack('Ii', cid, value)))

    def on_image(self, msg):
        now = self.get_clock().now().nanoseconds
        if now - self.last_ns < self.period_ns or msg.encoding != 'mono8':
            return
        self.last_ns = now
        gray = np.frombuffer(msg.data, np.uint8).reshape(msg.height, msg.step)[:, :msg.width]
        mean, saturated = image_stats(gray)
        new = next_setting(self.exposure, self.gain, mean, saturated, self.limits)
        if new is None:
            return
        self._set(V4L2_CID_EXPOSURE, new[0])
        self._set(V4L2_CID_ANALOGUE_GAIN, new[1])
        self.get_logger().info(
            f'mean {mean:.0f} chay {100 * saturated:.1f}% -> exposure {self.exposure}->{new[0]}'
            f' gain {self.gain}->{new[1]}', throttle_duration_sec=2.0)
        self.exposure, self.gain = new


def main(args=None):
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = AutoExposureNode()
    executor = EventsExecutor()
    executor.add_node(node)
    # Tat sach khi dung dich vu: xem fc_velocity_node.main (dung executor TRUOC roi moi tat
    # context; cho co timeout de handler tin hieu luon chay).
    stop = threading.Event()
    for sig in (signal.SIGINT, signal.SIGTERM):
        signal.signal(sig, lambda *_: stop.set())
    spin = ThreadPoolExecutor(1).submit(executor.spin)
    spin.add_done_callback(lambda _: stop.set())
    while not stop.wait(0.5):
        pass
    executor.shutdown()
    try:
        spin.result()
    finally:
        node.destroy_node()
        rclpy.shutdown()
