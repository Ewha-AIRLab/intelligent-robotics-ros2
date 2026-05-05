"""
multi_robot.launch.py — spawn two independent robots in the same world,
each with its own namespaced Nav2 stack.

Usage:
  ros2 launch multi_robot multi_robot.launch.py map:=/path/to/map.yaml
"""

import os
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                            OpaqueFunction)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def launch_setup(context, *args, **kwargs):
    pkg_share   = get_package_share_directory('multi_robot')
    slam_share  = get_package_share_directory('slam')
    ros_gz_share = get_package_share_directory('ros_gz_sim')

    robot_nav_launch = os.path.join(pkg_share, 'launch', 'robot_nav.launch.py')
    world_path = os.path.join(slam_share, 'worlds', 'slam_world.sdf')
    map_yaml   = LaunchConfiguration('map').perform(context)
    rviz_config = os.path.join(pkg_share, 'config', 'multi_robot.rviz')

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_share, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments=[('gz_args', f'-r {world_path}')],
    )

    # Bridge the clock once here — robot_nav.launch.py must NOT bridge it
    clock_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='clock_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
        output='screen',
    )

    robot1 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(robot_nav_launch),
        launch_arguments={
            'namespace':   'robot1',
            'x':           '-2.0',
            'y':           '0.0',
            'yaw':         '0.0',
            'map':         map_yaml,
            'nav2_delay':  '5.0',
        }.items(),
    )

    robot2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(robot_nav_launch),
        launch_arguments={
            'namespace':   'robot2',
            'x':           '2.0',
            'y':           '0.0',
            'yaw':         '3.14159',
            'map':         map_yaml,
            'nav2_delay':  '15.0',
        }.items(),
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config] if os.path.exists(rviz_config) else [],
        parameters=[{'use_sim_time': True}],
        output='screen',
    )

    return [gazebo, clock_bridge, robot1, robot2, rviz]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'map',
            default_value=os.path.expanduser('~/my_map.yaml'),
            description='Absolute path to the shared nav2 map YAML file'),
        OpaqueFunction(function=launch_setup),
    ])
