"""Khoi launch dung chung cho sim_tune.launch.py va sim_mission.launch.py."""

import os

from ament_index_python.packages import get_package_share_directory
from launch.actions import (AppendEnvironmentVariable, ExecuteProcess, IncludeLaunchDescription,
                            TimerAction)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode
from launch_ros.parameter_descriptions import ParameterValue
import yaml

from drone_sim.gripper_sim import joint_targets

SHARE = get_package_share_directory('drone_sim')
WORLD = os.path.join(SHARE, 'worlds', 'drone_tune.sdf')
MODELS = os.path.join(SHARE, 'models')
CONFIG = os.path.join(get_package_share_directory('drone_bringup'), 'config')
CONTROL_YAML = os.path.join(CONFIG, 'control.yaml')
# Ban do bai rieng cua mo phong (home + 2 bai, co huong -> co tag nho), khop worlds/drone_tune.sdf.
TAGS_YAML = os.path.join(get_package_share_directory('drone_sim'), 'config', 'sim_tags.yaml')
GZ_LAUNCH = os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')
SIM_TIME = {'use_sim_time': True}
# Vi tri camera duoi bung X3 - PHAI khop <sensor camera_down> trong worlds/drone_tune.sdf. x nhu
# drone that; z xem chu thich world. Goc nghieng do camera_tilt_node phat theo servo.
CAM_XYZ = ('0.09', '0', '-0.06')
# Joint kep (drone_v2.urdf) va hop (hop_<ten> / hang_<ten> trong worlds/drone_v2.sdf).
GRIPPER_JOINTS = tuple(joint_targets(0.0))
BOX_NAMES = ('a',)

# Hai drone mo phong (sim_mission.launch.py drone:=...). Moi so lieu phai khop world tuong ung.
DRONES = {
    # X3 Gazebo Fuel, camera gan cung (khong co servo trong Gazebo - camera:=true chi dung o 20 do).
    'x3': dict(world='drone_tune.sdf', model='X3', pivot=[float(v) for v in CAM_XYZ], lens=[0.0, 0.0, 0.0],
               ground_range_m=0.17, camera_joint_topic='', tof_xyz=[0.0], boxes=[0.0],
               range_node=False, vel_ki=0.0, vel_ki_z=0.0, gripper=False,
               # Camera thap hon base_link 6 cm: tag hop ra khoi khung tu ToF ~0,30 m.
               box_cam_switch_m=0.32),
    # Khung Version2.0 (models/drone_v2 <- drone_v2.urdf): truc servo + ong kinh theo URDF, laser 0,146 m
    # khi nam dat (chan 0,155 m, mat MTF-01 0,009 m duoi mat FC = goc base_link), servo camera that.
    'v2': dict(world='drone_v2.sdf', model='drone_v2', pivot=[0.09525, -0.018, -0.05218], lens=[0.0, 0.017, -0.0418],
               ground_range_m=0.146, camera_joint_topic='/X3/camera_tilt/cmd',
               # Laser theo tia (sim_tof): MTF-01 0,10 m sau tam, 0,009 m duoi FC (URDF); hop hang 12 cm
               # [x, y, yaw] - PHAI khop hop_a trong worlds/drone_v2.sdf (bai B trong de tha). Hop phia
               # "tren" bai A, xoay 120 do so voi bai (10-27): drone phai quay mui 30 do moi kep duoc.
               tof_xyz=[-0.10, 0.0, -0.009], boxes=[10.0, 0.30, 3.6652],
               # range_vertical_node THAT (bu nghieng + bu hop hang) thay laser da xu ly cua FC gia.
               range_node=True,
               # Khau I van toc ngang nhu FC that (Gazebo khong co): khu troi tinh do trong tam lech.
               # 1,0 -> 3,0 (10-25, gio giat 4 m/s): truoc khi neo Pi chua giu vi tri, ki 1 de gio day
               # 4 m luc cat canh -> mat tag nha. FC that: vong van toc ki 3 (chinh 10-10).
               vel_ki=3.0,
               # Khau I van toc dung (FC that giu do cao): drone mang hop 120 g khong chim (sim_fc_bridge).
               # 2,0 lam treo gap nhap nho +-3 cm chu ky ~2 s (cong voi vong ToF 5 Hz cua FSM) - 10-25.
               vel_ki_z=0.7,
               # Kep MG90 that trong Gazebo (sim_gripper_node): ham theo co cau 4 khau, nhac hop
               # 120 g.
               gripper=True,
               # Doi camera -20 -> -36 do o cung ToF 0,32 nhu drone that (mission.yaml). Che do tag tong
               # hop (camera:=false): sim_tag_node bo tag gan hon min_range 0,15 m - doan cuoi mat tag.
               box_cam_switch_m=0.32),
}


def camera_tilt_table():
    """Bang goc <-> xung cua camera_tilt_node (mission.yaml) - FC gia doi xung ra goc joint."""
    with open(os.path.join(CONFIG, 'mission.yaml'), encoding='utf-8') as f:
        p = yaml.safe_load(f)['camera_tilt_node']['ros__parameters']
    return [float(v) for v in p['tilt_deg']], [int(v) for v in p['pulse_us']]


def gazebo(gui, render_engine, world=WORLD):
    """-r: chay ngay; -s: chi server, khong cua so.

    render_engine 'ogre' (Ogre1): Ogre2 mac dinh nhap nhay tren may ao / GPU yeu."""
    def include(flags, condition):
        args = [flags, ' --render-engine ', render_engine, f' {world}']
        return IncludeLaunchDescription(PythonLaunchDescriptionSource(GZ_LAUNCH),
                                        launch_arguments=[('gz_args', args)],
                                        condition=condition)
    # model://drone_v2 trong worlds/drone_v2.sdf.
    return [AppendEnvironmentVariable('GZ_SIM_RESOURCE_PATH', MODELS),
            include('-r', IfCondition(gui)),
            include('-r -s', UnlessCondition(gui))]


def gz_bridge():
    """Gazebo <-> ROS. ']' = ROS -> Gazebo, '[' = Gazebo -> ROS."""
    return Node(package='ros_gz_bridge', executable='parameter_bridge', name='gz_bridge',
                arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
                           '/X3/gazebo/command/twist@geometry_msgs/msg/Twist]gz.msgs.Twist',
                           '/X3/enable@std_msgs/msg/Bool]gz.msgs.Boolean',
                           '/model/X3/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry',
                           # Goc servo camera (rad) -> JointPositionController (chi world drone_v2).
                           '/X3/camera_tilt/cmd@std_msgs/msg/Float64]gz.msgs.Double',
                           # Kep (chi world drone_v2): goc 5 joint + tach hop (sim_gripper_node).
                           *[f'/X3/gripper/{j}/cmd@std_msgs/msg/Float64]gz.msgs.Double'
                             for j in GRIPPER_JOINTS],
                           *[f'/X3/gripper/detach_{n}@std_msgs/msg/Empty]gz.msgs.Empty'
                             for n in BOX_NAMES]],
                output='screen')


def sim_wind(mps, dir_deg, gust, seed):
    """Gio giat (WindEffects, chi world drone_v2) - mac dinh lang."""
    return Node(package='drone_sim', executable='sim_wind_node', name='sim_wind_node',
                parameters=[{'wind_mps': ParameterValue(mps, value_type=float),
                             'wind_dir_deg': ParameterValue(dir_deg, value_type=float),
                             'wind_gust': ParameterValue(gust, value_type=float),
                             'wind_seed': ParameterValue(seed, value_type=int)}, SIM_TIME],
                output='screen')


def sim_gripper(boxes):
    """Kep MG90 trong Gazebo: /gripper/command -> 5 joint kep + gan/tach hop (drone_v2.sdf)."""
    return Node(package='drone_sim', executable='sim_gripper_node', name='sim_gripper_node',
                parameters=[{'boxes': boxes, 'box_names': list(BOX_NAMES)}, SIM_TIME],
                output='screen')


def sim_fc_bridge(auto_arm, heartbeat, delay, noise, drift=None, vel_lag=None, drone='x3'):
    """drift: (vx, vy, yaw_rate_dps) - sai so van toc FC gia lap; vel_lag: (tre thuan s, tau s) cua
    van toc ngang (xem sim_fc_bridge_node); drone: khoa cua DRONES."""
    d = DRONES[drone]
    tilt_deg, pulse_us = camera_tilt_table()
    p = {'auto_arm': auto_arm, 'publish_mission_heartbeat': heartbeat,
         'odom_delay_s': ParameterValue(delay, value_type=float),
         'odom_noise_m': ParameterValue(noise, value_type=float),
         'ground_range_m': d['ground_range_m'], 'camera_joint_topic': d['camera_joint_topic'],
         'tof_xyz': d['tof_xyz'], 'boxes': d['boxes'],
         'publish_range_vertical': not d['range_node'], 'vel_ki': d['vel_ki'],
         'vel_ki_z': d['vel_ki_z'],
         'tilt_deg': tilt_deg, 'pulse_us': pulse_us}
    if drift is not None:
        for k, v in zip(('drift_vx_mps', 'drift_vy_mps', 'drift_yaw_dps'), drift):
            p[k] = ParameterValue(v, value_type=float)
    if vel_lag is not None:
        for k, v in zip(('vel_delay_s', 'vel_tau_s'), vel_lag):
            p[k] = ParameterValue(v, value_type=float)
    return Node(package='drone_sim', executable='sim_fc_bridge_node', name='sim_fc_bridge_node',
                parameters=[p, SIM_TIME],
                output='screen')


def controller_node():
    """Cung control.yaml voi drone that - gain tune xong chep lai vao do."""
    return Node(package='drone_control', executable='position_controller_node',
                name='position_controller_node',
                # X3 quay yaw duoc va khong co rui ro dau: bat giu huong (drone that: sau WP5 ban).
                parameters=[CONTROL_YAML, {'heading_hold': True}, SIM_TIME],
                output='screen')


def camera_pipeline(condition):
    """Camera Gazebo -> rectify -> apriltag_ros THAT (cung apriltag.yaml voi Pi), thay sim_tag_node.

    Giong perception.launch.py cua drone that: cung node, cung tham so, cung ten topic/khung. K cua
    anh render = K that, D = 0 (anh pinhole) nen rectify gan nhu dong nhat - giu de chuoi giong that.
    """
    return [
        Node(package='ros_gz_bridge', executable='parameter_bridge', name='gz_camera_bridge',
             arguments=['/camera/image_raw@sensor_msgs/msg/Image[gz.msgs.Image',
                        '/camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo'],
             parameters=[SIM_TIME], condition=condition, output='screen'),
        Node(package='tf2_ros', executable='static_transform_publisher', name='camera_to_optical',
             arguments=['0', '0', '0', '-1.5708', '0', '-1.5708', 'camera_link',
                        'camera_optical_frame'],
             parameters=[SIM_TIME], condition=condition),
        ComposableNodeContainer(
            name='perception_container', namespace='', package='rclcpp_components',
            executable='component_container_isolated', condition=condition, output='screen',
            parameters=[SIM_TIME],
            composable_node_descriptions=[
                ComposableNode(package='image_proc', plugin='image_proc::RectifyNode',
                               name='image_rectify', namespace='camera',
                               remappings=[('image', 'image_raw')], parameters=[SIM_TIME]),
                ComposableNode(package='apriltag_ros', plugin='AprilTagNode',
                               name='marker_detector_node',
                               parameters=[os.path.join(CONFIG, 'apriltag.yaml'), SIM_TIME],
                               remappings=[('image_rect', '/camera/image_rect'),
                                           ('camera_info', '/camera/camera_info'),
                                           ('detections', '/apriltag/detections')]),
            ]),
        # Optical flow THAT tren anh camera Gazebo (cung perception.yaml voi Pi) - de kiem dau va do
        # lon van toc truoc khi tin no tren drone that. Chi phat /optical_flow/velocity, EKF sim
        # khong dung (sim_fc_bridge phat odom ground truth).
        Node(package='drone_perception', executable='optical_flow_node', name='optical_flow_node',
             parameters=[os.path.join(CONFIG, 'perception.yaml'), SIM_TIME], condition=condition,
             output='screen'),
    ]


def range_vertical(condition=None):
    """Node that: laser tho /mavros/mtf01p + IMU -> /range/vertical (bu nghieng, bu hop hang)."""
    return Node(package='drone_estimation', executable='range_vertical_node',
                name='range_vertical_node', parameters=[SIM_TIME], condition=condition,
                output='screen')


def camera_tilt(drone='x3'):
    """Node that: servo camera qua fc_command_bridge (sim_fc nhan lenh 183) + TF base_link ->
    camera_link theo goc servo. Truc servo + ong kinh = camera Gazebo cua drone do (DRONES)."""
    d = DRONES[drone]
    return Node(package='drone_mission', executable='camera_tilt_node', name='camera_tilt_node',
                parameters=[os.path.join(CONFIG, 'mission.yaml'),
                            {'camera_xyz': d['pivot'], 'lens_xyz': d['lens']}, SIM_TIME],
                output='screen')


def gui_follow(model, condition=None):
    """Goc nhin Gazebo GUI bam theo drone (sau, cheo trai, cao hon) - doi GUI len roi moi goi."""
    def svc(name, typ, req):
        return ExecuteProcess(cmd=['gz', 'service', '-s', name, '--reqtype', typ, '--reptype',
                                   'gz.msgs.Boolean', '--timeout', '3000', '--req', req])
    return TimerAction(period=10.0, condition=condition, actions=[
        svc('/gui/follow', 'gz.msgs.StringMsg', f'data: "{model}"'),
        svc('/gui/follow/offset', 'gz.msgs.Vector3d', 'x: -1.5, y: -1.5, z: 1.2')])


def camera_view(condition=None):
    """Cua so anh camera + goc servo + tag dang thay (can camera:=true - anh render that)."""
    return Node(package='drone_sim', executable='sim_camera_view_node', name='sim_camera_view_node',
                parameters=[SIM_TIME], condition=condition, output='screen')


def sim_tag(noise, dropout, condition=None):
    """Gia lap phat hien tag tu ground truth - thay ca camera + apriltag_ros."""
    return Node(package='drone_sim', executable='sim_tag_node', name='sim_tag_node',
                parameters=[TAGS_YAML,
                            {'noise_m': ParameterValue(noise, value_type=float),
                             'dropout_prob': ParameterValue(dropout, value_type=float)},
                            SIM_TIME],
                condition=condition, output='screen')


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
