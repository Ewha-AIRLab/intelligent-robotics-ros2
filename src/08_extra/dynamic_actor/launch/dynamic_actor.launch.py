import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory, get_package_prefix


def generate_launch_description():
    pkg_share = get_package_share_directory('dynamic_actor')
    pkg_prefix = get_package_prefix('dynamic_actor')
    ros_gz_sim_share = get_package_share_directory('ros_gz_sim')

    world_path = os.path.join(pkg_share, 'worlds', 'actor_world.sdf')

    existing_resource_path = os.environ.get('GZ_SIM_RESOURCE_PATH', '')
    resource_path = pkg_share + (os.pathsep + existing_resource_path if existing_resource_path else '')

    # Let Gazebo find the ActorPosePublisher plugin .so
    plugin_path = os.path.join(pkg_prefix, 'lib')
    existing_plugin_path = os.environ.get('IGN_GAZEBO_SYSTEM_PLUGIN_PATH', '')
    ign_plugin_path = plugin_path + (os.pathsep + existing_plugin_path if existing_plugin_path else '')

    return LaunchDescription([
        SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH', resource_path),
        SetEnvironmentVariable('IGN_GAZEBO_SYSTEM_PLUGIN_PATH', ign_plugin_path),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(ros_gz_sim_share, 'launch', 'gz_sim.launch.py')
            ),
            launch_arguments=[('gz_args', f'-r {world_path}')],
        ),

        # Bridge Gazebo LiDAR → ROS 2
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            arguments=[
                '/scan@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan',
            ],
            output='screen',
        ),

        # Static TF for the lidar sensor frame
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['0', '0', '0.5', '0', '0', '0', 'world', 'lidar_test/link/lidar'],
        ),
    ])
