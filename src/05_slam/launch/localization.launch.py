import os
import random
import math
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


# Safe spawn bounds — 1.5 m clear of outer walls (±5 m)
SPAWN_X_RANGE = (-3.0, 3.0)
SPAWN_Y_RANGE = (-3.0, 3.0)


def launch_setup(context, *args, **kwargs):
    slam_share   = get_package_share_directory('slam')
    sensor_share = get_package_share_directory('sensor')

    world_path   = os.path.join(slam_share, 'worlds', 'slam_world.sdf')
    amcl_params  = os.path.join(slam_share, 'config', 'amcl_params.yaml')
    rviz_config  = os.path.join(slam_share, 'config', 'localization.rviz')

    map_yaml = LaunchConfiguration('map').perform(context)

    # Random spawn pose inside the closed room
    x   = round(random.uniform(*SPAWN_X_RANGE), 3)
    y   = round(random.uniform(*SPAWN_Y_RANGE), 3)
    yaw = round(random.uniform(-math.pi, math.pi), 3)
    print(f'[localization.launch] Random spawn: x={x}, y={y}, yaw={yaw:.3f} rad')

    return [
        # ── 1. Simulation stack (Gazebo + bridge + RSP + spawn at random pose)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(sensor_share, 'launch', 'sim_sensor_robot.launch.py')
            ),
            launch_arguments={
                'world':       world_path,
                'launch_rviz': 'false',
                'spawn_x':     str(x),
                'spawn_y':     str(y),
                'spawn_yaw':   str(yaw),
            }.items(),
        ),

        # ── 2. Map server — serves the saved .pgm map on /map ───────────────
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            parameters=[{'yaml_filename': map_yaml, 'use_sim_time': True}],
            output='screen',
        ),

        # ── 3. AMCL — particle filter localisation ──────────────────────────
        Node(
            package='nav2_amcl',
            executable='amcl',
            name='amcl',
            parameters=[
                amcl_params,
                {'use_sim_time': True,
                 'initial_pose.x':   0.0,
                 'initial_pose.y':   0.0,
                 'initial_pose.yaw': 0.0},
            ],
            output='screen',
        ),

        # ── 4. Lifecycle manager — activates map_server and amcl ────────────
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_localization',
            parameters=[{
                'use_sim_time': True,
                'autostart':    True,
                'node_names':   ['map_server', 'amcl'],
            }],
            output='screen',
        ),

        # ── 5. RViz2 ────────────────────────────────────────────────────────
        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', rviz_config],
            parameters=[{'use_sim_time': True}],
            output='screen',
        ),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'map',
            default_value=os.path.expanduser('~/my_map.yaml'),
            description='Absolute path to a nav2 map YAML file',
        ),
        OpaqueFunction(function=launch_setup),
    ])
