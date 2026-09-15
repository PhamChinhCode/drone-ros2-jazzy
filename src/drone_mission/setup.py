from setuptools import find_packages, setup

package_name = 'drone_mission'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='pc',
    maintainer_email='phamvanchinh203@gmail.com',
    description='Lop hanh vi nhiem vu: may trang thai, gripper, cau noi lenh FC.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'mission_manager_node = drone_mission.mission_manager_node:main',
            'gripper_controller_node = drone_mission.gripper_controller_node:main',
            'fc_command_bridge_node = drone_mission.fc_command_bridge_node:main',
            'send_mission_plan = drone_mission.send_mission_plan:main',
        ],
    },
)
