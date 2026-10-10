"""system_monitor_node - tai CPU, nhiet do, ha xung, RAM, dia cua Pi moi giay -> /system/stats.

Ghi vao bag de khi phan tich chuyen bay biet Pi co qua tai / nong / thieu ap luc su co khong (EKF
"Failed to meet update rate" di kem tai cao). Muc canh bao chi de loc nhanh, khong kich failsafe.
"""

import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from rclpy.node import Node

from drone_safety.system_stats import read_stats


class SystemMonitorNode(Node):

    def __init__(self):
        super().__init__('system_monitor_node')
        self.declare_parameter('rate_hz', 1.0)
        self.declare_parameter('warn_load_per_cpu', 2.0)    # load 1 phut / so nhan
        self.declare_parameter('warn_temp_c', 75.0)
        self.pub = self.create_publisher(DiagnosticArray, '/system/stats', 10)
        self.create_timer(1.0 / self.get_parameter('rate_hz').value, self.tick)

    def tick(self):
        s = read_stats()
        st = DiagnosticStatus(name='pi_system', hardware_id='pi5')
        st.values = [KeyValue(key=k, value=(f'{v:.2f}' if isinstance(v, float) else str(v)))
                     for k, v in s.items()]
        warn = []
        g = self.get_parameter
        if 'load_1m' in s and s['load_1m'] > g('warn_load_per_cpu').value * (s['cpu_count'] or 1):
            warn.append(f"tai {s['load_1m']:.1f}")
        if s.get('cpu_temp_c', 0.0) > g('warn_temp_c').value:
            warn.append(f"nhiet {s['cpu_temp_c']:.0f} C")
        if s.get('throttled', 0) & 0xF:
            warn.append('ha xung/thieu ap: ' + ', '.join(s['throttled_flags']))
        st.level = DiagnosticStatus.WARN if warn else DiagnosticStatus.OK
        st.message = '; '.join(warn) or 'ok'
        msg = DiagnosticArray(status=[st])
        msg.header.stamp = self.get_clock().now().to_msg()
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = SystemMonitorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
