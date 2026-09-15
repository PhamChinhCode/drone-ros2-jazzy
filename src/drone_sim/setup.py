from glob import glob

from setuptools import find_packages, setup

package_name = 'drone_sim'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml', 'README.md']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/worlds', glob('worlds/*.sdf')),
        ('share/' + package_name + '/config/missions', glob('config/missions/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='pc',
    maintainer_email='phamvanchinh203@gmail.com',
    description='Mo phong Gazebo de tune vong vi tri Pi: FC gia, cau MAVROS, thu dap ung buoc.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'sim_fc_bridge_node = drone_sim.sim_fc_bridge_node:main',
            'step_test = drone_sim.step_test:main',
        ],
    },
)
