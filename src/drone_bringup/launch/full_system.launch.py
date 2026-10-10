"""Giai doan 3-5 - toan bo he thong: nhiem vu, giao tiep GCS, an toan va ghi log.

rosbag2 chay kem bang ExecuteProcess de tu khoi dong cung stack (log tho, debug ky thuat) -
khac muc dich voi mission_logger_node (log nghiep vu doi chieu CSDL GCS).
"""

import os
import time

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

BRINGUP = get_package_share_directory('drone_bringup')
CONFIG = os.path.join(BRINGUP, 'config')
# Uu tien ban da nap qua day (giao uoc GCS 8.7, P31) neu co, khong thi dung ban dong bo trong repo.
_TAGS_OVERRIDE = os.path.expanduser('~/.config/drone_ros2_jazzy/tags_override.yaml')
TAGS_YAML = _TAGS_OVERRIDE if os.path.isfile(_TAGS_OVERRIDE) else os.path.join(CONFIG, 'tags.yaml')

# Ghi MOI topic (10-10: phan tich duoc moi van de cua chuyen bay - lenh gui FC, co OB/arm, can RC,
# IMU, EKF, flow, laser, tag, GPS, log moi node /rosout, tai Pi /system/stats...) TRU luong anh
# /camera/*: anh tho 30 FPS ~7,7 MB/s (09-17 da lam day the SD 229 GB) va image_transport con phat
# them ban nen zstd/jpeg/theora ~6 MB/s. Anh de xem lai lay tu camera_log_node (/log/camera, 2 Hz).
# Cat file moi 10 phut + nen zstd theo chunk; scripts/prune_logs.sh giu tong <= 10 GB.
BAG_CMD = ['ros2', 'bag', 'record', '--all-topics', '--exclude-regex', '^/camera/.*',
           '--max-bag-duration', '600', '--storage-preset-profile', 'zstd_fast']


def generate_launch_description():
    return LaunchDescription([
        IncludeLaunchDescription(PythonLaunchDescriptionSource(
            os.path.join(BRINGUP, 'launch', 'control.launch.py'))),

        Node(package='drone_mission', executable='fc_command_bridge_node',
             name='fc_command_bridge_node',
             parameters=[os.path.join(CONFIG, 'mission.yaml')], output='screen'),

        Node(package='drone_mission', executable='mission_manager_node',
             name='mission_manager_node',
             parameters=[os.path.join(CONFIG, 'mission.yaml'), TAGS_YAML],
             output='screen'),

        Node(package='drone_mission', executable='gripper_controller_node',
             name='gripper_controller_node',
             parameters=[os.path.join(CONFIG, 'mission.yaml')], output='screen'),

        # tags.yaml BAT BUOC: thieu no thi known_tags khong khai bao, tagmap_crc = 0, va GCS
        # KHOA chuc nang nap ke hoach (giao uoc GCS muc 8.6). Loi nay chi lo ra tren drone that.
        Node(package='drone_comms', executable='telemetry_aggregator_node',
             name='telemetry_aggregator_node',
             parameters=[os.path.join(CONFIG, 'comms.yaml'), TAGS_YAML], output='screen'),

        Node(package='drone_comms', executable='gcs_link_node', name='gcs_link_node',
             parameters=[os.path.join(CONFIG, 'comms.yaml')], output='screen'),

        Node(package='drone_safety', executable='failsafe_monitor_node',
             name='failsafe_monitor_node',
             parameters=[os.path.join(CONFIG, 'safety.yaml')], output='screen'),

        Node(package='drone_safety', executable='mission_logger_node',
             name='mission_logger_node',
             parameters=[os.path.join(CONFIG, 'safety.yaml')], output='screen'),

        # Ghi kem bag: tai / nhiet / ha xung cua Pi (1 Hz) va anh camera thua (2 Hz JPEG).
        Node(package='drone_safety', executable='system_monitor_node',
             name='system_monitor_node', output='screen'),
        Node(package='drone_perception', executable='camera_log_node',
             name='camera_log_node', output='screen'),

        # Cau noi WebSocket de xem truc tiep toan bo he thong tren Foxglove
        # (may khac ket noi toi ws://<ip-drone>:8765).
        Node(package='foxglove_bridge', executable='foxglove_bridge',
             name='foxglove_bridge',
             parameters=[{'port': 8765, 'address': '0.0.0.0'}], output='screen'),

        # Moi lan chay mot thu muc rieng: ten co dinh thi tu lan khoi dong thu hai (tu chay luc
        # boot) ros2 bag chet vi "Output folder already exists".
        ExecuteProcess(cmd=[*BAG_CMD, '-o',
                            os.path.expanduser(time.strftime('~/drone_logs/bag_%Y%m%d_%H%M%S'))],
                       output='screen'),
    ])
