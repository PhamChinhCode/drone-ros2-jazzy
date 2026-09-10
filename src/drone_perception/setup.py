from setuptools import find_packages, setup

package_name = 'drone_perception'

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
    description='Lop cam nhan: optical flow va tom tat chat luong bam marker.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'optical_flow_node = drone_perception.optical_flow_node:main',
            'marker_quality_node = drone_perception.marker_quality_node:main',
        ],
    },
)
