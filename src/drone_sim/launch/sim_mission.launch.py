"""Thu bay waypoint qua nhiem vu that trong Gazebo (sau khi da tune cruise.*).

  ros2 launch drone_sim sim_mission.launch.py
  ros2 run drone_mission send_mission_plan \
      $(ros2 pkg prefix drone_sim)/share/drone_sim/config/missions/sim_tag1.yaml
  ros2 service call /mission_manager_node/start std_srvs/srv/Trigger

Chay node Pi that: fc_command_bridge_node, mission_manager_node, position_controller_node,
ekf_health_node, failsafe_monitor_node. CHUA gia lap camera/tag: MARKER_SEARCH se het gio, thu lai
Phat hien tag hai cach:
  camera:=true (mac dinh) - camera Gazebo duoi bung X3 (dung vi tri/goc lap + K that) chup bai co
    anh AprilTag that, apriltag_ros THAT giai ma; than nghieng thi anh nghieng theo;
  camera:=false - sim_tag_node tinh tu ground truth (nhanh, khong can GPU), them nhieu/mat khung:
    ros2 launch drone_sim sim_mission.launch.py camera:=false tag_noise_m:=0.02 tag_dropout:=0.1
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

from drone_sim.sim_launch import (TAGS_YAML, camera_pipeline, controller_node, gazebo, gcs_link,
                                  gripper_node,
                                  gz_bridge, landing_bridge, marker_republisher,
                                  sim_fc_bridge, sim_tag, telemetry_aggregator)

CONFIG = os.path.join(get_package_share_directory('drone_bringup'), 'config')
SIM_TIME = {'use_sim_time': True}


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('render_engine', default_value='ogre'),
        DeclareLaunchArgument('odom_delay_s', default_value='0.0'),
        DeclareLaunchArgument('odom_noise_m', default_value='0.0'),
        DeclareLaunchArgument('takeoff_alt_m', default_value='2.0'),
        DeclareLaunchArgument('tag_noise_m', default_value='0.0'),
        DeclareLaunchArgument('tag_dropout', default_value='0.0'),
        DeclareLaunchArgument('camera', default_value='true'),
        # Sai so van toc FC gia lap (m/s, m/s, do/s) - thu giu vi tri / giu huong cua Pi.
        DeclareLaunchArgument('drift_vx', default_value='0.0'),
        DeclareLaunchArgument('drift_vy', default_value='0.0'),
        DeclareLaunchArgument('drift_yaw_dps', default_value='0.0'),
        *gazebo(LaunchConfiguration('gui'), LaunchConfiguration('render_engine')),
        gz_bridge(),
        sim_fc_bridge(auto_arm=False, heartbeat=False,
                      delay=LaunchConfiguration('odom_delay_s'),
                      noise=LaunchConfiguration('odom_noise_m'),
                      drift=(LaunchConfiguration('drift_vx'), LaunchConfiguration('drift_vy'),
                             LaunchConfiguration('drift_yaw_dps'))),
        controller_node(),
        sim_tag(LaunchConfiguration('tag_noise_m'), LaunchConfiguration('tag_dropout'),
                UnlessCondition(LaunchConfiguration('camera'))),
        *camera_pipeline(IfCondition(LaunchConfiguration('camera'))),
        landing_bridge(),
        marker_republisher(),
        gripper_node(),
        telemetry_aggregator(),
        gcs_link(),
        Node(package='drone_mission', executable='fc_command_bridge_node',
             name='fc_command_bridge_node',
             parameters=[os.path.join(CONFIG, 'mission.yaml'), SIM_TIME], output='screen'),
        Node(package='drone_mission', executable='mission_manager_node',
             name='mission_manager_node',
             parameters=[os.path.join(CONFIG, 'mission.yaml'), TAGS_YAML,
                         {'takeoff_alt_m': ParameterValue(LaunchConfiguration('takeoff_alt_m'),
                                                          value_type=float),
                          # X3 quay yaw duoc: thu WP5 + PAD_ALIGN truoc khi yaw FC that dat WP0.
                          'yaw_control': True},
                         SIM_TIME],
             output='screen'),
        Node(package='drone_estimation', executable='ekf_health_node', name='ekf_health_node',
             parameters=[os.path.join(CONFIG, 'estimation.yaml'), SIM_TIME], output='screen'),
        Node(package='drone_safety', executable='failsafe_monitor_node',
             name='failsafe_monitor_node',
             parameters=[os.path.join(CONFIG, 'safety.yaml'), SIM_TIME], output='screen'),
    ])
