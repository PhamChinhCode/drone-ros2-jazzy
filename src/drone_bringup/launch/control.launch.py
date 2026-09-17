"""Giai doan 2 - them lop dieu khien. Muc tieu: ha canh chinh xac tai mot diem co dinh.

CANH BAO: gain PID trong control.yaml dang la 0.0. Tune trong Gazebo truoc khi chay that.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

BRINGUP = get_package_share_directory('drone_bringup')
CONFIG = os.path.join(BRINGUP, 'config')
# Uu tien ban da nap qua day (giao uoc GCS 8.7, P31) neu co, khong thi dung ban dong bo trong repo.
_TAGS_OVERRIDE = os.path.expanduser('~/.config/drone_ros2_jazzy/tags_override.yaml')
TAGS_YAML = _TAGS_OVERRIDE if os.path.isfile(_TAGS_OVERRIDE) else os.path.join(CONFIG, 'tags.yaml')


def generate_launch_description():
    return LaunchDescription([
        IncludeLaunchDescription(PythonLaunchDescriptionSource(
            os.path.join(BRINGUP, 'launch', 'estimation.launch.py'))),

        Node(package='drone_control', executable='landing_target_bridge_node',
             name='landing_target_bridge_node',
             parameters=[os.path.join(CONFIG, 'control.yaml'), TAGS_YAML],
             output='screen'),

        Node(package='drone_control', executable='position_controller_node',
             name='position_controller_node',
             parameters=[os.path.join(CONFIG, 'control.yaml')], output='screen'),
    ])
