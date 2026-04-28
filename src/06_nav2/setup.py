from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'nav2'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'config'), glob('config/*.rviz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Daeun Song',
    maintainer_email='songd@ewha.ac.kr',
    description='Autonomous navigation with Nav2 on a simulated differential-drive robot',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'waypoint_navigator = nav2.waypoint_navigator:main',
        ],
    },
)
