from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'slam'

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
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*.sdf')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Daeun Song',
    maintainer_email='songd@ewha.ac.kr',
    description='Online SLAM demonstration using slam_toolbox with a simulated differential-drive robot',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [],
    },
)
