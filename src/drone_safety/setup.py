from setuptools import find_packages, setup

package_name = 'drone_safety'

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
    description='Lop an toan va ghi log: giam sat failsafe tap trung va log nghiep vu.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'failsafe_monitor_node = drone_safety.failsafe_monitor_node:main',
            'mission_logger_node = drone_safety.mission_logger_node:main',
        ],
    },
)
