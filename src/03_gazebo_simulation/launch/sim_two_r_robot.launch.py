import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_share = get_package_share_directory('gazebo_simulation')
    ros_gz_sim_share = get_package_share_directory('ros_gz_sim')

    urdf_path = os.path.join(pkg_share, 'urdf', 'two_r_robot.urdf')
    with open(urdf_path, 'r') as f:
        robot_description = f.read().replace('$(find gazebo_simulation)', pkg_share)

    rviz_config = os.path.join(pkg_share, 'config', 'display_robot.rviz')

    return LaunchDescription([
        # Gazebo Fortress
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(ros_gz_sim_share, 'launch', 'gz_sim.launch.py')
            ),
            launch_arguments=[('gz_args', '-r empty.sdf --physics-engine ignition-physics5-bullet-plugin')],
        ),

        # Bridge /clock
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
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
            arguments=['-topic', 'robot_description', '-name', '2r_robot'],
            output='screen',
        ),

        # Controllers
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
            output='screen',
        ),
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=['joint_trajectory_controller', '--controller-manager', '/controller_manager'],
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

        # rqt joint trajectory controller GUI
        Node(
            package='rqt_joint_trajectory_controller',
            executable='rqt_joint_trajectory_controller',
            arguments=['--force-discover'],
            output='screen',
        ),
    ])
