"""
robot_nav.launch.py — spawn one namespaced robot + full Nav2 stack.

Arguments:
  namespace   Robot namespace, e.g. "robot1" (default: robot)
  x, y, yaw  Spawn pose in the world (default: 0 0 0)
  map         Absolute path to a nav2 map YAML file
"""

import os
import xacro
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, GroupAction,
                            OpaqueFunction, TimerAction)
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace
from ament_index_python.packages import get_package_share_directory
from nav2_common.launch import RewrittenYaml


def launch_setup(context, *args, **kwargs):
    ns      = LaunchConfiguration('namespace').perform(context)
    x       = LaunchConfiguration('x').perform(context)
    y       = LaunchConfiguration('y').perform(context)
    yaw     = LaunchConfiguration('yaw').perform(context)
    map_yaml = LaunchConfiguration('map').perform(context)

    pkg_share = get_package_share_directory('multi_robot')
    xacro_path  = os.path.join(pkg_share, 'urdf', 'multi_robot.urdf.xacro')
    params_path = os.path.join(pkg_share, 'config', 'nav2_params.yaml')
    bt_xml = os.path.join(
        get_package_share_directory('nav2_bt_navigator'),
        'behavior_trees',
        'navigate_to_pose_w_replanning_and_recovery.xml')

    # Process xacro with the robot's namespace
    robot_description = xacro.process_file(
        xacro_path, mappings={'namespace': ns}).toxml()

    # Rewrite Nav2 params: substitute generic frame names with namespaced ones
    nav2_params = RewrittenYaml(
        source_file=params_path,
        root_key=ns,
        param_rewrites={
            'base_frame_id':    f'{ns}/base_link',
            'robot_base_frame': f'{ns}/base_link',
            'odom_frame_id':    f'{ns}/odom',
            'global_frame':     f'{ns}/odom',   # local costmap + behavior server
            'odom_topic':       f'/{ns}/odom',
            'scan_topic':       f'/{ns}/scan',
        },
        convert_types=True)

    # ── Robot state publisher (publishes {ns}/base_link, {ns}/odom frames) ──
    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        namespace=ns,
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': True,
            'frame_prefix': f'{ns}/',
        }],
        output='screen',
    )

    # ── Gazebo spawn ─────────────────────────────────────────────────────────
    spawn = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-topic', f'/{ns}/robot_description',
            '-name',  ns,
            '-x', x, '-y', y, '-z', '0.11', '-Y', yaw,
        ],
        output='screen',
    )

    # ── Bridge: namespaced Gazebo ↔ ROS topics ───────────────────────────────
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name=f'bridge_{ns}',
        arguments=[
            f'/{ns}/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
            f'/{ns}/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
            f'/{ns}/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
            '/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
        ],
        output='screen',
    )

    # ── Nav2 nodes — all pushed under /{ns}/ namespace ───────────────────────
    nav2_nodes = GroupAction([
        PushRosNamespace(ns),

        Node(package='nav2_map_server', executable='map_server',
             name='map_server',
             parameters=[nav2_params, {'yaml_filename': map_yaml,
                                       'use_sim_time': True}],
             output='screen'),

        Node(package='nav2_amcl', executable='amcl', name='amcl',
             parameters=[nav2_params, {'use_sim_time': True}],
             output='screen'),

        Node(package='nav2_lifecycle_manager',
             executable='lifecycle_manager',
             name='lifecycle_manager_localization',
             parameters=[{'use_sim_time': True, 'autostart': True,
                          'node_names': ['map_server', 'amcl']}],
             output='screen'),

        Node(package='nav2_planner', executable='planner_server',
             name='planner_server',
             parameters=[nav2_params, {'use_sim_time': True}],
             output='screen'),

        Node(package='nav2_controller', executable='controller_server',
             name='controller_server',
             parameters=[nav2_params, {'use_sim_time': True}],
             output='screen'),

        Node(package='nav2_behaviors', executable='behavior_server',
             name='behavior_server',
             parameters=[nav2_params, {'use_sim_time': True}],
             output='screen'),

        Node(package='nav2_bt_navigator', executable='bt_navigator',
             name='bt_navigator',
             parameters=[nav2_params, {'use_sim_time': True,
                                       'default_nav_to_pose_bt_xml': bt_xml}],
             output='screen'),

        Node(package='nav2_lifecycle_manager',
             executable='lifecycle_manager',
             name='lifecycle_manager_navigation',
             parameters=[{'use_sim_time': True, 'autostart': True,
                          'node_names': ['planner_server', 'controller_server',
                                         'behavior_server', 'bt_navigator']}],
             output='screen'),
    ])

    return [rsp, bridge, spawn, TimerAction(period=5.0, actions=[nav2_nodes])]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('namespace', default_value='robot'),
        DeclareLaunchArgument('x',         default_value='0.0'),
        DeclareLaunchArgument('y',         default_value='0.0'),
        DeclareLaunchArgument('yaw',       default_value='0.0'),
        DeclareLaunchArgument(
            'map',
            default_value=os.path.expanduser('~/my_map.yaml'),
            description='Absolute path to a nav2 map YAML file'),
        OpaqueFunction(function=launch_setup),
    ])
