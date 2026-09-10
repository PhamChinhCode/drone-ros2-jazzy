from setuptools import find_packages, setup

package_name = 'drone_comms'

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
    description='Lop giao tiep: duong lien lac GCS rieng va gop telemetry.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'gcs_link_node = drone_comms.gcs_link_node:main',
            'telemetry_aggregator_node = drone_comms.telemetry_aggregator_node:main',
        ],
    },
)
