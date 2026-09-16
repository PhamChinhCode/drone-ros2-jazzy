"""Khoi launch dung chung cho sim_tune.launch.py va sim_mission.launch.py."""

import os

from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

WORLD = os.path.join(get_package_share_directory('drone_sim'), 'worlds', 'drone_tune.sdf')
CONFIG = os.path.join(get_package_share_directory('drone_bringup'), 'config')
CONTROL_YAML = os.path.join(CONFIG, 'control.yaml')
TAGS_YAML = os.path.join(CONFIG, 'tags.yaml')
GZ_LAUNCH = os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')
SIM_TIME = {'use_sim_time': True}


def gazebo(gui, render_engine):
    """-r: chay ngay; -s: chi server, khong cua so.

    render_engine 'ogre' (Ogre1): Ogre2 mac dinh nhap nhay tren may ao / GPU yeu."""
    def include(flags, condition):
        args = [flags, ' --render-engine ', render_engine, f' {WORLD}']
        return IncludeLaunchDescription(PythonLaunchDescriptionSource(GZ_LAUNCH),
                                        launch_arguments=[('gz_args', args)],
                                        condition=condition)
    return [include('-r', IfCondition(gui)),
            include('-r -s', UnlessCondition(gui))]


def gz_bridge():
    """Gazebo <-> ROS. ']' = ROS -> Gazebo, '[' = Gazebo -> ROS."""
    return Node(package='ros_gz_bridge', executable='parameter_bridge', name='gz_bridge',
                arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
                           '/X3/gazebo/command/twist@geometry_msgs/msg/Twist]gz.msgs.Twist',
                           '/X3/enable@std_msgs/msg/Bool]gz.msgs.Boolean',
                           '/model/X3/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry'],
                output='screen')


def sim_fc_bridge(auto_arm, heartbeat, delay, noise):
    return Node(package='drone_sim', executable='sim_fc_bridge_node', name='sim_fc_bridge_node',
                parameters=[{'auto_arm': auto_arm, 'publish_mission_heartbeat': heartbeat,
                             'odom_delay_s': ParameterValue(delay, value_type=float),
                             'odom_noise_m': ParameterValue(noise, value_type=float)}, SIM_TIME],
                output='screen')


def controller_node():
    """Cung control.yaml voi drone that - gain tune xong chep lai vao do."""
    return Node(package='drone_control', executable='position_controller_node',
                name='position_controller_node', parameters=[CONTROL_YAML, SIM_TIME],
                output='screen')


def sim_tag(noise, dropout):
    """Gia lap phat hien tag tu ground truth - thay ca camera + apriltag_ros."""
    return Node(package='drone_sim', executable='sim_tag_node', name='sim_tag_node',
                parameters=[TAGS_YAML,
                            {'noise_m': ParameterValue(noise, value_type=float),
                             'dropout_prob': ParameterValue(dropout, value_type=float)},
                            SIM_TIME],
                output='screen')


def landing_bridge():
    """Node that, khong sua gi - nhan /apriltag/detections + TF, ra /landing_target/pose."""
    return Node(package='drone_control', executable='landing_target_bridge_node',
                name='landing_target_bridge_node',
                parameters=[CONTROL_YAML, TAGS_YAML, SIM_TIME], output='screen')


def gripper_node():
    """Node that o che do simulate: gia lap hanh trinh servo + cong tac xac nhan."""
    return Node(package='drone_mission', executable='gripper_controller_node',
                name='gripper_controller_node',
                parameters=[os.path.join(CONFIG, 'mission.yaml'), {'simulate': True}, SIM_TIME],
                output='screen')


def marker_republisher():
    """Node that, khong sua gi: /apriltag/detections + TF -> /marker/pose_odom.

    Can cho ekf_health_node biet odom da NEO theo bang tag (EkfHealth.anchored) - dieu kien de
    FSM chot "nha" cho RTH. Giao uoc GCS <-> Pi muc 5.2b.
    """
    return Node(package='drone_estimation', executable='marker_pose_republisher_node',
                name='marker_pose_republisher_node',
                parameters=[os.path.join(CONFIG, 'estimation.yaml'), TAGS_YAML, SIM_TIME],
                output='screen')


def telemetry_aggregator():
    """Node that. tags.yaml BAT BUOC: thieu no thi tagmap_crc = 0 va GCS khoa nap ke hoach."""
    return Node(package='drone_comms', executable='telemetry_aggregator_node',
                name='telemetry_aggregator_node',
                parameters=[os.path.join(CONFIG, 'comms.yaml'), TAGS_YAML, SIM_TIME],
                output='screen')


def gcs_link():
    """Node that, lay nguyen comms.yaml - ke ca gcs_host, nen GCS o may khac tren LAN van nhan duoc.

    Dia chi do chi la du phong: Pi uu tien dia chi HOC duoc tu goi hop le gan nhat (giao uoc 2.1).
    """
    return Node(package='drone_comms', executable='gcs_link_node', name='gcs_link_node',
                parameters=[os.path.join(CONFIG, 'comms.yaml'),
                            # Mo phong chay tren loopback cua CHINH may nay - mang kin theo dung
                            # nghia den nhat, nen tat chu ky de khong bat moi nguoi tao khoa truoc
                            # khi chay thu. Tu 2026-09-16 comms.yaml CUNG dang de false theo yeu cau
                            # nguoi van hanh, nen dong nay tam thoi thua - giu lai de mo phong van
                            # tat chu ky ngay khi ai do bat lai comms.yaml cho drone that.
                            {'signing_required': False},
                            SIM_TIME], output='screen')
