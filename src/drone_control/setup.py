from setuptools import find_packages, setup

package_name = 'drone_control'

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
    description='Lop dieu khien: cau noi landing target va vong PID vi tri.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'landing_target_bridge_node = drone_control.landing_target_bridge_node:main',
            'position_controller_node = drone_control.position_controller_node:main',
        ],
    },
)
