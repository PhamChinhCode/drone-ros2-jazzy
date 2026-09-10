"""Hai profile QoS dung chung theo quy uoc muc 0 tai lieu huong dan."""

from rclpy.qos import QoSProfile, QoSReliabilityPolicy

# Topic tan so cao, chi can gia tri moi nhat: anh, IMU, odometry, setpoint.
SENSOR_QOS = QoSProfile(depth=1, reliability=QoSReliabilityPolicy.BEST_EFFORT)

# Topic su kien roi rac va quan trong: lenh, failsafe, ACK, trang thai nhiem vu.
EVENT_QOS = QoSProfile(depth=10, reliability=QoSReliabilityPolicy.RELIABLE)
