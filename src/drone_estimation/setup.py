from setuptools import find_packages, setup

package_name = 'drone_estimation'

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
    description='Lop uoc luong: chuyen pose marker sang khung odom va co suc khoe EKF.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'marker_pose_republisher_node = drone_estimation.marker_pose_republisher_node:main',
            'ekf_health_node = drone_estimation.ekf_health_node:main',
            'fc_velocity_node = drone_estimation.fc_velocity_node:main',
        ],
    },
)
