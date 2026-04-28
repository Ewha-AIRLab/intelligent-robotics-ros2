import os
import random
import math
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


SPAWN_X_RANGE = (-3.0, 3.0)
SPAWN_Y_RANGE = (-3.0, 3.0)


def launch_setup(context, *args, **kwargs):
    nav2_share          = get_package_share_directory('nav2')
    sensor_share        = get_package_share_directory('sensor')
    slam_share          = get_package_share_directory('slam')
    bt_navigator_share  = get_package_share_directory('nav2_bt_navigator')

    world_path  = os.path.join(slam_share, 'worlds', 'slam_world.sdf')
    nav2_params = os.path.join(nav2_share,  'config', 'nav2_params.yaml')
    rviz_config = os.path.join(nav2_share,  'config', 'navigation.rviz')
    bt_xml      = os.path.join(bt_navigator_share, 'behavior_trees',
                               'navigate_to_pose_w_replanning_and_recovery.xml')

    map_yaml = LaunchConfiguration('map').perform(context)

    x   = round(random.uniform(*SPAWN_X_RANGE), 3)
    y   = round(random.uniform(*SPAWN_Y_RANGE), 3)
    yaw = round(random.uniform(-math.pi, math.pi), 3)
    print(f'[navigation.launch] Random spawn: x={x}, y={y}, yaw={yaw:.3f} rad')

    return [
        # ── 1. Robot bringup (Gazebo + bridge + RSP + spawn) ────────────────
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

        # ── 2. Map server ────────────────────────────────────────────────────
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            parameters=[nav2_params, {'yaml_filename': map_yaml, 'use_sim_time': True}],
            output='screen',
        ),

        # ── 3. AMCL — initialised at the actual spawn pose ───────────────────
        Node(
            package='nav2_amcl',
            executable='amcl',
            name='amcl',
            parameters=[
                nav2_params,
                {'use_sim_time': True,
                 'initial_pose.x':   x,
                 'initial_pose.y':   y,
                 'initial_pose.yaw': yaw},
            ],
            output='screen',
        ),

        # ── 4. Lifecycle manager — localization ──────────────────────────────
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

        # ── 5. Planner server ────────────────────────────────────────────────
        Node(
            package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            parameters=[nav2_params, {'use_sim_time': True}],
            output='screen',
        ),

        # ── 6. Controller server ─────────────────────────────────────────────
        Node(
            package='nav2_controller',
            executable='controller_server',
            name='controller_server',
            parameters=[nav2_params, {'use_sim_time': True}],
            output='screen',
        ),

        # ── 7. Behavior server ───────────────────────────────────────────────
        Node(
            package='nav2_behaviors',
            executable='behavior_server',
            name='behavior_server',
            parameters=[nav2_params, {'use_sim_time': True}],
            output='screen',
        ),

        # ── 8. BT navigator ──────────────────────────────────────────────────
        Node(
            package='nav2_bt_navigator',
            executable='bt_navigator',
            name='bt_navigator',
            parameters=[nav2_params, {'use_sim_time': True,
                                      'default_nav_to_pose_bt_xml': bt_xml}],
            output='screen',
        ),

        # ── 9. Lifecycle manager — navigation ────────────────────────────────
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_navigation',
            parameters=[{
                'use_sim_time': True,
                'autostart':    True,
                'node_names':   ['planner_server', 'controller_server',
                                 'behavior_server', 'bt_navigator'],
            }],
            output='screen',
        ),

        # ── 10. RViz2 ────────────────────────────────────────────────────────
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
