"""Khoi launch dung chung cho sim_tune.launch.py va sim_mission.launch.py."""

import os

from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

WORLD = os.path.join(get_package_share_directory('drone_sim'), 'worlds', 'drone_tune.sdf')
CONTROL_YAML = os.path.join(get_package_share_directory('drone_bringup'), 'config', 'control.yaml')
GZ_LAUNCH = os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')
SIM_TIME = {'use_sim_time': True}


def gazebo(gui):
    """-r: chay ngay; -s: chi server, khong cua so."""
    def include(args, condition):
        return IncludeLaunchDescription(PythonLaunchDescriptionSource(GZ_LAUNCH),
                                        launch_arguments={'gz_args': args}.items(),
                                        condition=condition)
    return [include(f'-r {WORLD}', IfCondition(gui)),
            include(f'-r -s {WORLD}', UnlessCondition(gui))]


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
