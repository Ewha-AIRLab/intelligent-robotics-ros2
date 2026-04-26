import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_share = get_package_share_directory('sensor')
    ros_gz_sim_share = get_package_share_directory('ros_gz_sim')

    urdf_path   = os.path.join(pkg_share, 'urdf', 'sensor_robot.urdf')
    rviz_config = os.path.join(pkg_share, 'config', 'sensor_robot.rviz')
    bridge_yaml = os.path.join(pkg_share, 'config', 'bridge.yaml')
    default_world = os.path.join(pkg_share, 'worlds', 'sensor_world.sdf')

    with open(urdf_path, 'r') as f:
        robot_description = f.read()

    return LaunchDescription([
        DeclareLaunchArgument('world',       default_value=default_world),
        DeclareLaunchArgument('launch_rviz', default_value='true'),
        DeclareLaunchArgument('spawn_x',     default_value='0'),
        DeclareLaunchArgument('spawn_y',     default_value='0'),
        DeclareLaunchArgument('spawn_yaw',   default_value='0'),

        # Gazebo Fortress
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(ros_gz_sim_share, 'launch', 'gz_sim.launch.py')
            ),
            launch_arguments=[('gz_args', ['-r ', LaunchConfiguration('world')])],
        ),

        # Bridge: all topics defined in config/bridge.yaml
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            parameters=[{'config_file': bridge_yaml}],
            output='screen',
        ),

        # Publish robot description
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': robot_description, 'use_sim_time': True}],
            output='screen',
        ),

        # Spawn robot
        Node(
            package='ros_gz_sim',
            executable='create',
            arguments=[
                '-topic', 'robot_description', '-name', 'sensor_robot',
                '-x', LaunchConfiguration('spawn_x'),
                '-y', LaunchConfiguration('spawn_y'),
                '-z', '0.11',
                '-Y', LaunchConfiguration('spawn_yaw'),
            ],
            output='screen',
        ),

        # RViz
        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', rviz_config],
            parameters=[{'use_sim_time': True}],
            condition=IfCondition(LaunchConfiguration('launch_rviz')),
            output='screen',
        ),
    ])
