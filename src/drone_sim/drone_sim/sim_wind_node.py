"""sim_wind_node - gio giat cho Gazebo (WindEffects trong worlds/drone_v2.sdf). CHI dung mo phong.

Moi 0,1 s (gio mo phong): GustyWind (wind_model) -> /world/<world>/wind (gz.msgs.Wind, gui thang
bang gz-transport vi ros_gz_bridge khong co kieu Wind) + /sim/wind (Vector3Stamped, ENU m/s).
wind_mps = 0: gio lang, khong gui gi.
"""

from geometry_msgs.msg import Vector3Stamped
from gz.msgs10.wind_pb2 import Wind
from gz.transport13 import Node as GzNode
import rclpy
from rclpy.node import Node

from drone_control.qos import SENSOR_QOS
from drone_sim.wind_model import GustyWind


class SimWindNode(Node):

    def __init__(self):
        super().__init__('sim_wind_node')
        self.declare_parameter('world', 'drone_tune')
        self.declare_parameter('wind_mps', 0.0)
        self.declare_parameter('wind_dir_deg', 0.0)     # huong gio THOI TOI, ENU (0 dong, 90 bac)
        self.declare_parameter('wind_gust', 0.35)       # do lech chuan giat / toc do trung binh
        self.declare_parameter('wind_seed', 0)
        g = lambda n: self.get_parameter(n).value       # noqa: E731
        self.wind = GustyWind(g('wind_mps'), g('wind_dir_deg'), gust=g('wind_gust'),
                              seed=g('wind_seed'))
        self.gz = GzNode()
        self.pub_gz = self.gz.advertise(f'/world/{g("world")}/wind', Wind)
        self.pub = self.create_publisher(Vector3Stamped, '/sim/wind', SENSOR_QOS)
        self.dt = 0.1
        if g('wind_mps') > 0.0:
            self.create_timer(self.dt, self.step)
            self.get_logger().info(f'gio {g("wind_mps"):.1f} m/s huong {g("wind_dir_deg"):.0f} '
                                   f'do, giat {g("wind_gust") * 100:.0f} %')

    def step(self):
        vx, vy = self.wind.step(self.dt)
        msg = Wind()
        msg.linear_velocity.x, msg.linear_velocity.y = vx, vy
        msg.enable_wind = True
        self.pub_gz.publish(msg)
        out = Vector3Stamped()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = 'odom'
        out.vector.x, out.vector.y = vx, vy
        self.pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = SimWindNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
