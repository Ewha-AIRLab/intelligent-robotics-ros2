import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_share = get_package_share_directory('sensor')
    gazebo_ros_share = get_package_share_directory('gazebo_ros')

    urdf_path  = os.path.join(pkg_share, 'urdf', 'sensor_robot_classic.urdf')
    world_path = os.path.join(pkg_share, 'worlds', 'sensor_world_classic.world')
    rviz_config = os.path.join(pkg_share, 'config', 'sensor_robot.rviz')

    with open(urdf_path, 'r') as f:
        robot_description = f.read()

    return LaunchDescription([
        # Gazebo Classic (gzserver + gzclient)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(gazebo_ros_share, 'launch', 'gazebo.launch.py')
            ),
            launch_arguments={'world': world_path}.items(),
        ),

        # Publish robot description
        # Note: Classic plugins publish directly to ROS — no bridge needed
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': robot_description, 'use_sim_time': True}],
            output='screen',
        ),

        # Spawn robot into Gazebo Classic
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=[
                '-topic', 'robot_description',
                '-entity', 'sensor_robot_classic',
                '-z', '0.11',
            ],
            output='screen',
        ),

        # RViz — same config as the Fortress version
        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', rviz_config],
            parameters=[{'use_sim_time': True}],
            output='screen',
        ),
    ])
