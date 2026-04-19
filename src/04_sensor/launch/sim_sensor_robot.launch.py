import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_share = get_package_share_directory('sensor')
    ros_gz_sim_share = get_package_share_directory('ros_gz_sim')

    urdf_path   = os.path.join(pkg_share, 'urdf', 'sensor_robot.urdf')
    world_path  = os.path.join(pkg_share, 'worlds', 'sensor_world.sdf')
    rviz_config = os.path.join(pkg_share, 'config', 'sensor_robot.rviz')
    bridge_yaml = os.path.join(pkg_share, 'config', 'bridge.yaml')

    with open(urdf_path, 'r') as f:
        robot_description = f.read()

    return LaunchDescription([
        # Gazebo Fortress (sensor_world.sdf includes the Sensors plugin for gpu_lidar + camera)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(ros_gz_sim_share, 'launch', 'gz_sim.launch.py')
            ),
            launch_arguments=[('gz_args', f'-r {world_path}')],
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
            arguments=['-topic', 'robot_description', '-name', 'sensor_robot', '-z', '0.11'],
            output='screen',
        ),

        # RViz
        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', rviz_config],
            parameters=[{'use_sim_time': True}],
            output='screen',
        ),
    ])
