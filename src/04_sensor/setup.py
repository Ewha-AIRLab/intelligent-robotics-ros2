from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'sensor'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'urdf'),   glob('urdf/*.urdf')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.rviz')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*.sdf')),
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*.world')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Daeun Song',
    maintainer_email='songd@ewha.ac.kr',
    description='LiDAR and RGB camera simulation with Gazebo Fortress',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'lidar_subscriber = sensor.lidar_subscriber:main',
            'camera_subscriber = sensor.camera_subscriber:main',
        ],
    },
)
