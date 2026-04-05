from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='basics',
            executable='talker',
            name='talker',
            output='screen',
        ),
        Node(
            package='basics',
            executable='listener',
            name='listener',
            output='screen',
        ),
    ])