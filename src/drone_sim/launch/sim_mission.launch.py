"""Thu bay waypoint qua nhiem vu that trong Gazebo (sau khi da tune cruise.*).

  ros2 launch drone_sim sim_mission.launch.py
  ros2 run drone_mission send_mission_plan \
      $(ros2 pkg prefix drone_sim)/share/drone_sim/config/missions/sim_tag1.yaml
  ros2 service call /mission_manager_node/start std_srvs/srv/Trigger

Chay node Pi that: fc_command_bridge_node, mission_manager_node, position_controller_node,
ekf_health_node, failsafe_monitor_node. CHUA gia lap camera/tag: MARKER_SEARCH se het gio, thu lai
drone:=v2 (mac dinh) - khung Version2.0 (models/drone_v2 sinh tu drone_v2.urdf), servo camera that
  trong Gazebo, hop hang o bai A; drone:=x3 - X3 cua Gazebo Fuel nhu truoc (tune gain).
Phat hien tag hai cach:
  camera:=true (mac dinh) - camera Gazebo duoi bung drone (dung vi tri/goc lap + K that) chup bai co
    anh AprilTag that, apriltag_ros THAT giai ma; than nghieng thi anh nghieng theo (x3: camera co
    dinh 20 do, khong theo servo);
  camera:=false - sim_tag_node tinh tu ground truth (nhanh, khong can GPU), them nhieu/mat khung:
    ros2 launch drone_sim sim_mission.launch.py camera:=false tag_noise_m:=0.02 tag_dropout:=0.1
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

from drone_sim.sim_launch import (DRONES, SHARE, TAGS_YAML, camera_pipeline, camera_tilt,
                                  camera_view, controller_node, gazebo, gcs_link, gripper_node,
                                  gui_follow, range_vertical,
                                  gz_bridge, landing_bridge, marker_republisher,
                                  sim_fc_bridge, sim_tag, telemetry_aggregator)

CONFIG = os.path.join(get_package_share_directory('drone_bringup'), 'config')
SIM_TIME = {'use_sim_time': True}


def theo_drone(context):
    """Phan phu thuoc loai drone (world, camera, laser, nguong doi goc camera toi hop)."""
    drone = LaunchConfiguration('drone').perform(context)
    if drone not in DRONES:
        raise RuntimeError(f'drone:={drone} - chi co {sorted(DRONES)}')
    d = DRONES[drone]
    return [
        *gazebo(LaunchConfiguration('gui'), LaunchConfiguration('render_engine'),
                os.path.join(SHARE, 'worlds', d['world'])),
        sim_fc_bridge(auto_arm=False, heartbeat=False,
                      delay=LaunchConfiguration('odom_delay_s'),
                      noise=LaunchConfiguration('odom_noise_m'),
                      drift=(LaunchConfiguration('drift_vx'), LaunchConfiguration('drift_vy'),
                             LaunchConfiguration('drift_yaw_dps')),
                      vel_lag=(LaunchConfiguration('vel_delay_s'), LaunchConfiguration('vel_tau_s')),
                      drone=drone),
        camera_tilt(drone),
        *([range_vertical()] if d['range_node'] else []),
        gui_follow(d['model'], IfCondition(LaunchConfiguration('gui'))),
        Node(package='drone_mission', executable='mission_manager_node',
             name='mission_manager_node',
             parameters=[os.path.join(CONFIG, 'mission.yaml'), TAGS_YAML,
                         {'takeoff_alt_m': ParameterValue(LaunchConfiguration('takeoff_alt_m'),
                                                          value_type=float),
                          # Mo phong quay yaw duoc: thu WP5 + ALIGN_HEADING nhu drone that.
                          'yaw_control': True,
                          'box_cam_switch_m': d['box_cam_switch_m']},
                         SIM_TIME],
             output='screen'),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('drone', default_value='v2'),
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('render_engine', default_value='ogre'),
        DeclareLaunchArgument('odom_delay_s', default_value='0.0'),
        DeclareLaunchArgument('odom_noise_m', default_value='0.0'),
        DeclareLaunchArgument('takeoff_alt_m', default_value='2.0'),
        DeclareLaunchArgument('tag_noise_m', default_value='0.0'),
        DeclareLaunchArgument('tag_dropout', default_value='0.0'),
        DeclareLaunchArgument('camera', default_value='true'),
        # Cua so xem anh camera (goc servo, trang thai, tag) - chi co anh khi camera:=true.
        DeclareLaunchArgument('camera_view', default_value='true'),
        # Sai so van toc FC gia lap (m/s, m/s, do/s) - thu giu vi tri / giu huong cua Pi.
        DeclareLaunchArgument('drift_vx', default_value='0.0'),
        DeclareLaunchArgument('drift_vy', default_value='0.0'),
        DeclareLaunchArgument('drift_yaw_dps', default_value='0.0'),
        # Dap ung van toc ngang cham cua FC that (s): tre thuan + quan tinh bac 1.
        DeclareLaunchArgument('vel_delay_s', default_value='0.0'),
        DeclareLaunchArgument('vel_tau_s', default_value='0.0'),
        OpaqueFunction(function=theo_drone),
        gz_bridge(),
        controller_node(),
        sim_tag(LaunchConfiguration('tag_noise_m'), LaunchConfiguration('tag_dropout'),
                UnlessCondition(LaunchConfiguration('camera'))),
        *camera_pipeline(IfCondition(LaunchConfiguration('camera'))),
        camera_view(IfCondition(LaunchConfiguration('camera_view'))),
        landing_bridge(),
        marker_republisher(),
        gripper_node(),
        telemetry_aggregator(),
        gcs_link(),
        Node(package='drone_mission', executable='fc_command_bridge_node',
             name='fc_command_bridge_node',
             parameters=[os.path.join(CONFIG, 'mission.yaml'), SIM_TIME], output='screen'),
        Node(package='drone_estimation', executable='ekf_health_node', name='ekf_health_node',
             parameters=[os.path.join(CONFIG, 'estimation.yaml'), SIM_TIME], output='screen'),
        Node(package='drone_safety', executable='failsafe_monitor_node',
             name='failsafe_monitor_node',
             parameters=[os.path.join(CONFIG, 'safety.yaml'), SIM_TIME], output='screen'),
    ])
