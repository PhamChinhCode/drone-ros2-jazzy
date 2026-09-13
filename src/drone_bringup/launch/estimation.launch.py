"""Giai doan 1b - them MAVROS va EKF len tren chuoi cam nhan. Muc tieu: position hold khong troi.

Thu tu kiem thu (muc 2.1 tai lieu huong dan): chi IMU truoc -> them optical flow -> them marker,
moi lan them mot nguon thi kiem `/odometry/filtered` khong co buoc nhay dot ngot.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

BRINGUP = get_package_share_directory('drone_bringup')
CONFIG = os.path.join(BRINGUP, 'config')


def generate_launch_description():
    return LaunchDescription([
        IncludeLaunchDescription(PythonLaunchDescriptionSource(
            os.path.join(BRINGUP, 'launch', 'perception.launch.py'))),

        # KHONG dat name=: no remap ten MOI node con trong process (router, tung plugin) thanh
        # 'mavros' -> plugin de topic cua nhau va crash. Chi dat namespace, giong mavros/node.launch.
        Node(package='mavros', executable='mavros_node', namespace='mavros',
             parameters=[os.path.join(CONFIG, 'mavros.yaml')], output='screen'),

        # Vi tri lap camera tren khung drone - DO THAT roi sua sau day.
        # Sai transform nay gay trieu chung "thay dung marker nhung bay lech tam".
        Node(package='tf2_ros', executable='static_transform_publisher',
             name='base_to_camera',
             arguments=['0', '0', '-0.05', '0', '1.5708', '0', 'base_link', 'camera_link']),

        # Noi hai quy uoc truc (REP 103): camera_link la x-tien, z-len; con
        # camera_optical_frame la x-phai, y-xuong, z-theo huong nhin. camera.yaml dat
        # camera_frame_id='camera_optical_frame' nen apriltag phat pose trong khung do -
        # thieu phep bien doi nay thi camera_optical_frame KHONG noi vao cay TF va moi
        # lookupTransform cua marker_pose_republisher_node / landing_target_bridge_node
        # deu that bai. Goc co dinh theo quy uoc, KHONG phai so do lap dat.
        Node(package='tf2_ros', executable='static_transform_publisher',
             name='camera_to_optical',
             arguments=['0', '0', '0', '-1.5708', '0', '-1.5708',
                        'camera_link', 'camera_optical_frame']),

        Node(package='drone_estimation', executable='marker_pose_republisher_node',
             name='marker_pose_republisher_node',
             parameters=[os.path.join(CONFIG, 'estimation.yaml')], output='screen'),

        Node(package='robot_localization', executable='ekf_node', name='ekf_filter_node',
             parameters=[os.path.join(CONFIG, 'ekf.yaml')], output='screen'),

        Node(package='drone_estimation', executable='ekf_health_node', name='ekf_health_node',
             parameters=[os.path.join(CONFIG, 'estimation.yaml')], output='screen'),
    ])
