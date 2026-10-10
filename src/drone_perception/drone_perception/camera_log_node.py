"""camera_log_node - anh camera thua (mac dinh 2 Hz, JPEG) de rosbag ghi lai cung chuyen bay.

Bag KHONG ghi luong anh goc (30 FPS ~7,7 MB/s, tung lam day the SD 09-17) nhung phan tich chuyen bay
can THAY duoc luc mat tag (bong che, nhoe, tag ra mep khung - 10-09). Node nhan anh o dang byte tho
(raw=True, khong giai ma tung khung) va chi giai ma + nen khung den han: ~40 KB/s, ~0,15 GB/gio.
Topic ra /log/camera/compressed (ngoai /camera/* nen khong bi bo loc bag loai).
"""

import rclpy
from rclpy.node import Node
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import CompressedImage, Image

from drone_perception.camera_log import encode_jpeg
from drone_perception.qos import SENSOR_QOS


class CameraLogNode(Node):

    def __init__(self):
        super().__init__('camera_log_node')
        self.declare_parameter('image_topic', '/camera/image_raw')
        self.declare_parameter('rate_hz', 2.0)
        self.declare_parameter('jpeg_quality', 70)
        self.period_ns = int(1e9 / self.get_parameter('rate_hz').value)
        self.quality = self.get_parameter('jpeg_quality').value
        self.last_ns = 0
        self.pub = self.create_publisher(CompressedImage, '/log/camera/compressed', SENSOR_QOS)
        self.create_subscription(Image, self.get_parameter('image_topic').value, self.on_image,
                                 SENSOR_QOS, raw=True)

    def on_image(self, raw):
        now_ns = self.get_clock().now().nanoseconds
        if now_ns - self.last_ns < self.period_ns:
            return
        self.last_ns = now_ns
        img = deserialize_message(raw, Image)
        if img.encoding not in ('mono8', '8UC1'):
            return
        jpg = encode_jpeg(img.width, img.height, img.step, img.data, self.quality)
        if jpg is None:
            return
        out = CompressedImage()
        out.header = img.header          # giu stamp anh goc de khop thoi gian voi cac topic khac
        out.format = 'jpeg'
        out.data = jpg
        self.pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = CameraLogNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
