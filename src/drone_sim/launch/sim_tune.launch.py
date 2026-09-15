"""Tune cruise.* trong Gazebo: drone X3 + FC gia + position_controller_node, KHONG co nhiem vu.

  ros2 launch drone_sim sim_tune.launch.py                      # co cua so Gazebo
  ros2 launch drone_sim sim_tune.launch.py gui:=false           # chi server (PC yeu)
  ros2 launch drone_sim sim_tune.launch.py odom_delay_s:=0.1 odom_noise_m:=0.03

FC gia tu arm va tu phat /mission/state (watchdog P6). Doi gain luc dang chay:
  ros2 param set /position_controller_node cruise.z.kp 0.8
Thu buoc: ros2 run drone_sim step_test --axis z --step 2.0
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

from drone_sim.sim_launch import controller_node, gazebo, gz_bridge, sim_fc_bridge


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('odom_delay_s', default_value='0.0'),
        DeclareLaunchArgument('odom_noise_m', default_value='0.0'),
        *gazebo(LaunchConfiguration('gui')),
        gz_bridge(),
        sim_fc_bridge(auto_arm=True, heartbeat=True,
                      delay=LaunchConfiguration('odom_delay_s'),
                      noise=LaunchConfiguration('odom_noise_m')),
        controller_node(),
    ])
