"""Giai doan 1a - chuoi cam nhan: camera -> rectify -> apriltag -> chat luong bam + optical flow.

Kiem thu truoc khi di tiep:
  1. `ros2 topic hz /camera/image_raw` ra ~60 Hz (khop --vblank 1779);
  2. KIEM THONG KE PIXEL (topic hz KHONG bat duoc loi anh toan so 0):
     anh dung phai co std vai chuc, max gan 255. min=max=0 -> sai format,
     chay lai camera_v4l2_setup.sh; mean~16 std<2 -> thieu anh sang;
  3. KIEM camera_info CO INTRINSICS: `ros2 topic echo /camera/camera_info --once`
     phai co k[0] != 0 va d[] du 5 he so. Neu k toan 0 / d rong thi camera_info_url
     dang tro nham file khac do phan giai - camera_info_manager bo im lang, node CHI
     log INFO. Bo qua thi rectify vo nghia va apriltag khong giai duoc PnP;
  4. `ros2 topic echo /apriltag/detections` cho khoang cach khop thuoc do tay
     (sai lech >5-10% thuong do sai `size` tag hoac chua hieu chinh camera).
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

CONFIG = os.path.join(get_package_share_directory('drone_bringup'), 'config')


def generate_launch_description():
    return LaunchDescription([
        # v4l2_camera doc thang /dev/video0, KHONG dung camera_ros/libcamera
        # (docs/CAMERA.md muc 9: hai loi khong sua duoc tu ung dung tren Ubuntu 24.04).
        #
        # BAT BUOC chay TRUOC launch nay, va lai sau MOI LAN REBOOT:
        #   /home/pc/ros2_ws/scripts/camera_v4l2_setup.sh --width 640 --height 400 --vblank 1779 --exposure 300 --gain 32
        # Exposure/gain do 09-14 trong phong: 800/120 lam 80 % pixel bao hoa, vien trang tag chay va
        # apriltag KHONG bat duoc tag; 300/32 cho mean ~60, bat tag on dinh. Ngoai troi phai do lai.
        # Bo qua buoc do thi topic van ra dung nhip nhung MOI KHUNG HINH TOAN SO 0
        # (subdev con o Y10_1X10 trong khi node xin GREY 8-bit) - `ros2 topic hz` KHONG
        # phat hien duoc loi nay, phai kiem thong ke pixel.
        #
        # --vblank 1779 = 60 FPS. KHONG dung 110 (246 FPS) du sensor chay duoc:
        # do thuc tren may nay, 246 FPS lam Pi 4 bao hoa (load 9.4, idle 1%) va apriltag
        # chi ghep duoc 15 cap image/camera_info moi 10s -> WARN "do not appear to be
        # synchronized" lien tuc. O 60 FPS: 373/375 khung khop stamp, log sach, apriltag
        # xu ly du 60 Hz. FPS = pixel_rate / ((400+vblank) * (640+890)), pixel_rate=200e6.
        # Dung namespace CHU KHONG dung remappings: remap chi doi topic goc, cac topic
        # cua image_transport (compressed/theora/zstd) van giu ten cu -> chui ra
        # /image_raw/compressed thay vi /camera/image_raw/compressed. Lech namespace thi
        # Foxglove (va cac cong cu khac) khong tu ghep duoc anh voi camera_info.
        Node(package='v4l2_camera', executable='v4l2_camera_node', name='camera_node',
             namespace='camera',
             parameters=[os.path.join(CONFIG, 'camera.yaml')],
             output='screen'),

        # Khu meo ong kinh truoc khi phat hien marker: marker o ria khung hinh bi meo nhieu nhat.
        # Cung ly do namespace nhu camera_node: de /camera/image_rect/compressed nam dung cho.
        # Trong namespace 'camera' thi camera_info va image_rect da dung san, chi con
        # phai tro 'image' sang 'image_raw'.
        Node(package='image_proc', executable='rectify_node', name='image_rectify',
             namespace='camera',
             remappings=[('image', 'image_raw')],
             output='screen'),

        Node(package='apriltag_ros', executable='apriltag_node', name='marker_detector_node',
             parameters=[os.path.join(CONFIG, 'apriltag.yaml')],
             remappings=[('image_rect', '/camera/image_rect'),
                         ('camera_info', '/camera/camera_info'),
                         ('detections', '/apriltag/detections')],
             output='screen'),

        Node(package='drone_perception', executable='marker_quality_node',
             name='marker_quality_node',
             parameters=[os.path.join(CONFIG, 'perception.yaml')], output='screen'),

        # Optical flow doc anh THO (khong rectify): sai so meo anh nho so voi sai so optical flow.
        # OV9281 la sensor mono nen anh da la mono8 san - khong ton buoc cvtColor.
        Node(package='drone_perception', executable='optical_flow_node',
             name='optical_flow_node',
             parameters=[os.path.join(CONFIG, 'perception.yaml')], output='screen'),
    ])
