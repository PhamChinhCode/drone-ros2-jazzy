"""Giai doan 1b - them MAVROS va EKF len tren chuoi cam nhan. Muc tieu: position hold khong troi.

Thu tu kiem thu (muc 2.1 tai lieu huong dan): chi IMU truoc -> them optical flow -> them marker,
moi lan them mot nguon thi kiem `/odometry/filtered` khong co buoc nhay dot ngot.
"""

import os
import shlex

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import yaml

BRINGUP = get_package_share_directory('drone_bringup')
CONFIG = os.path.join(BRINGUP, 'config')
MAVROS_PLUGINS_YAML = os.path.join(CONFIG, 'mavros_plugins.yaml')
# Uu tien ban da nap qua day (giao uoc GCS 8.7, P31) neu co, khong thi dung ban dong bo trong repo.
_TAGS_OVERRIDE = os.path.expanduser('~/.config/drone_ros2_jazzy/tags_override.yaml')
TAGS_YAML = _TAGS_OVERRIDE if os.path.isfile(_TAGS_OVERRIDE) else os.path.join(CONFIG, 'tags.yaml')


def _plugin_param_script():
    """Lenh shell 'ros2 param set' tung tham so trong mavros_plugins.yaml, lap den khi node len.

    Khong dung 'ros2 param load': tren Pi 5 no crash (assert PyUnicode_Check) voi tham so chuoi.
    Chuoi dump YAML mot dong co nhay kep de 'ros2 param set' doc lai dung kieu string.
    """
    with open(MAVROS_PLUGINS_YAML) as f:
        nodes = yaml.safe_load(f)
    cmds = []
    for node, section in nodes.items():
        for name, value in section['ros__parameters'].items():
            style = '"' if isinstance(value, str) else None
            text = yaml.safe_dump(value, default_style=style, width=1 << 20)
            text = shlex.quote(text.strip().removesuffix('...').strip())
            cmds.append(f'until ros2 param set {node} {name} {text}; do sleep 1; done')
    return '; '.join(cmds)


def generate_launch_description():
    return LaunchDescription([
        IncludeLaunchDescription(PythonLaunchDescriptionSource(
            os.path.join(BRINGUP, 'launch', 'perception.launch.py'))),

        # KHONG dat name=: no remap ten MOI node con trong process (router, tung plugin) thanh
        # 'mavros' -> plugin de topic cua nhau va crash.
        # KHONG dat namespace= (MAVROS 2.15): plugin nam duoi ten DAY DU cua node UAS, namespace
        # 'mavros' bien no thanh /mavros/mavros -> moi topic doi sang /mavros/mavros/... va khong
        # node Pi nao nghe/gui duoc (2026-10-08). De mac dinh: UAS = /mavros, topic /mavros/<...>.
        Node(package='mavros', executable='mavros_node',
             parameters=[os.path.join(CONFIG, 'mavros.yaml'), MAVROS_PLUGINS_YAML],
             output='screen'),

        # MAVROS 2.15.1 khong dua --params-file toi node plugin (mavlink/mavros#2294) -> dat lai
        # tham so plugin khi node da len. Thieu: IMU covariance mac dinh, laser khong phat.
        ExecuteProcess(cmd=[_plugin_param_script()], shell=True, output='screen'),

        # Vi tri lap camera tren khung drone - DO THAT roi sua sau day.
        # Sai transform nay gay trieu chung "thay dung marker nhung bay lech tam".
        # Do 09-14: truoc tam 6 cm, cao ngang cam bien ToF (lay z = 0), mat camera nghieng 20 do so
        # voi mat ban (theo ban ve), nhin ve phia truoc -> truc nhin lech 20 do khoi phuong thang
        # dung: pitch = 90 - 20 = 70 do. Tag do sau hieu chinh 640x400 cho 17-23 do. Tham so:
        # x y z yaw pitch roll.
        Node(package='tf2_ros', executable='static_transform_publisher',
             name='base_to_camera',
             arguments=['0.06', '0', '0', '0', '1.2217', '0', 'base_link', 'camera_link']),

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
             parameters=[os.path.join(CONFIG, 'estimation.yaml'), TAGS_YAML],
             output='screen'),

        # Van toc FC da loc mau khong hop le -> /fc/velocity_xy, /fc/velocity_z cho EKF.
        Node(package='drone_estimation', executable='fc_velocity_node', name='fc_velocity_node',
             output='screen'),

        # GPS (giao uoc FC 1.8) -> khung ban do tag theo goc trong tags.yaml -> /gps/pose_odom.
        Node(package='drone_estimation', executable='gps_odom_node', name='gps_odom_node',
             parameters=[TAGS_YAML], output='screen'),

        # Laser doc truc than -> do cao thang dung (bu nghieng IMU) -> /range/vertical.
        Node(package='drone_estimation', executable='range_vertical_node',
             name='range_vertical_node', output='screen'),

        Node(package='robot_localization', executable='ekf_node', name='ekf_filter_node',
             parameters=[os.path.join(CONFIG, 'ekf.yaml')], output='screen'),

        Node(package='drone_estimation', executable='ekf_health_node', name='ekf_health_node',
             parameters=[os.path.join(CONFIG, 'estimation.yaml')], output='screen'),
    ])
