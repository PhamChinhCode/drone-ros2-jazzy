"""Giai doan 3-5 - toan bo he thong: nhiem vu, giao tiep GCS, an toan va ghi log.

rosbag2 chay kem bang ExecuteProcess de tu khoi dong cung stack (log tho, debug ky thuat) -
khac muc dich voi mission_logger_node (log nghiep vu doi chieu CSDL GCS).
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

BRINGUP = get_package_share_directory('drone_bringup')
CONFIG = os.path.join(BRINGUP, 'config')

BAG_TOPICS = [
    '/camera/image_raw', '/apriltag/detections', '/optical_flow/velocity',
    '/odometry/filtered', '/mavros/state', '/mavros/battery',
    '/mission/state', '/failsafe_event', '/gripper/status',
]


def generate_launch_description():
    return LaunchDescription([
        IncludeLaunchDescription(PythonLaunchDescriptionSource(
            os.path.join(BRINGUP, 'launch', 'control.launch.py'))),

        Node(package='drone_mission', executable='fc_command_bridge_node',
             name='fc_command_bridge_node',
             parameters=[os.path.join(CONFIG, 'mission.yaml')], output='screen'),

        Node(package='drone_mission', executable='mission_manager_node',
             name='mission_manager_node',
             parameters=[os.path.join(CONFIG, 'mission.yaml')], output='screen'),

        Node(package='drone_mission', executable='gripper_controller_node',
             name='gripper_controller_node',
             parameters=[os.path.join(CONFIG, 'mission.yaml')], output='screen'),

        Node(package='drone_comms', executable='telemetry_aggregator_node',
             name='telemetry_aggregator_node',
             parameters=[os.path.join(CONFIG, 'comms.yaml')], output='screen'),

        Node(package='drone_comms', executable='gcs_link_node', name='gcs_link_node',
             parameters=[os.path.join(CONFIG, 'comms.yaml')], output='screen'),

        Node(package='drone_safety', executable='failsafe_monitor_node',
             name='failsafe_monitor_node',
             parameters=[os.path.join(CONFIG, 'safety.yaml')], output='screen'),

        Node(package='drone_safety', executable='mission_logger_node',
             name='mission_logger_node',
             parameters=[os.path.join(CONFIG, 'safety.yaml')], output='screen'),

        # Cau noi WebSocket de xem truc tiep toan bo he thong tren Foxglove
        # (may khac ket noi toi ws://<ip-drone>:8765).
        Node(package='foxglove_bridge', executable='foxglove_bridge',
             name='foxglove_bridge',
             parameters=[{'port': 8765, 'address': '0.0.0.0'}], output='screen'),

        ExecuteProcess(cmd=['ros2', 'bag', 'record', '-o',
                            os.path.expanduser('~/drone_logs/bag'), *BAG_TOPICS],
                       output='screen'),
    ])
