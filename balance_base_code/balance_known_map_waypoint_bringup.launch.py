#!/usr/bin/env python3

"""Bring up the complete known-map waypoint navigation system.

Startup sequence:
    0 s: balance robot base
    2 s: N10P LiDAR
    5 s: map server and AMCL
    9 s: Nav2 navigation servers
   15 s: continuous waypoint loop (optional)

Before using automatic movement, make sure the AMCL initial pose matches the
robot's real pose in the saved map. Use RViz "2D Pose Estimate" when needed.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    LogInfo,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # ------------------------------------------------------------------
    # Launch arguments
    # ------------------------------------------------------------------
    default_map = PathJoinSubstitution([
        FindPackageShare('balance_mapping'),
        'maps',
        'balance_map_1.yaml',
    ])

    map_arg = DeclareLaunchArgument(
        'map',
        default_value=default_map,
        description='Full path to the saved map YAML file.',
    )

    start_waypoints_arg = DeclareLaunchArgument(
        'start_waypoints',
        default_value='true',
        description=(
            'Start the continuous waypoint loop. Set false when you want to '
            'bring up localization/Nav2 first and test manually.'
        ),
    )

    map_file = LaunchConfiguration('map')
    start_waypoints = LaunchConfiguration('start_waypoints')

    # ------------------------------------------------------------------
    # Locate existing project launch files
    # ------------------------------------------------------------------
    robot_base_launch_file = os.path.join(
        get_package_share_directory('balance_robot_base'),
        'launch',
        'balance_robot_base.launch.py',
    )

    lidar_launch_file = os.path.join(
        get_package_share_directory('lslidar_driver_n10p'),
        'launch',
        'lslidar_launch.py',
    )

    localization_launch_file = os.path.join(
        get_package_share_directory('balance_navigation'),
        'launch',
        'balance_amcl_localization.launch.py',
    )

    nav2_launch_file = os.path.join(
        get_package_share_directory('balance_navigation'),
        'launch',
        'balance_nav2_core.launch.py',
    )

    # ------------------------------------------------------------------
    # 0 seconds: robot base and static transforms
    # ------------------------------------------------------------------
    robot_base_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(robot_base_launch_file)
    )

    # ------------------------------------------------------------------
    # 2 seconds: LiDAR, publishing /scan
    # ------------------------------------------------------------------
    lidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(lidar_launch_file)
    )

    delayed_lidar_launch = TimerAction(
        period=2.0,
        actions=[
            LogInfo(msg='[bringup] Starting N10P LiDAR...'),
            lidar_launch,
        ],
    )

    # ------------------------------------------------------------------
    # 5 seconds: map server and AMCL
    # ------------------------------------------------------------------
    localization_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(localization_launch_file),
        launch_arguments={
            'map': map_file,
        }.items(),
    )

    delayed_localization_launch = TimerAction(
        period=5.0,
        actions=[
            LogInfo(msg='[bringup] Starting map server and AMCL...'),
            localization_launch,
        ],
    )

    # ------------------------------------------------------------------
    # 9 seconds: Nav2 planner, controller, BT navigator, behaviors,
    # waypoint follower, and lifecycle manager
    # ------------------------------------------------------------------
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(nav2_launch_file)
    )

    delayed_nav2_launch = TimerAction(
        period=9.0,
        actions=[
            LogInfo(msg='[bringup] Starting Nav2...'),
            nav2_launch,
        ],
    )

    # ------------------------------------------------------------------
    # 15 seconds: repeatedly execute the waypoint route
    #
    # Coordinates are absolute positions in the map frame. Replace the
    # example values below with safe free-space positions from your map.
    # ------------------------------------------------------------------
    continuous_waypoint_node = Node(
        package='balance_bringup',
        executable='continuous_waypoint_loop',
        name='continuous_waypoint_loop',
        output='screen',
        condition=IfCondition(start_waypoints),
        parameters=[{
            'frame_id': 'map',
            'repeat_delay': 1.0,
            'waypoints_xy': [
                -1.191, 0.172,   # waypoint 1: x, y
                #0.66, 0.10,   # waypoint 2: x, y
                #0.73, 0.49,
                -1.284, 1.491,   # waypoint 3: x, y
                -0.328, 0.896,   # waypoint 4: x, y
                -0.153, 1.482,   # waypoint 3: x, y
                -0.328, 0.896,
                -1.284, 1.491,   # waypoint 2: x, y
            ],
        }],
    )

    delayed_waypoint_launch = TimerAction(
        period=15.0,
        actions=[
            LogInfo(
                msg=(
                    '[bringup] Starting continuous waypoint loop. '
                    'Confirm that the AMCL initial pose is correct.'
                )
            ),
            continuous_waypoint_node,
        ],
    )

    return LaunchDescription([
        map_arg,
        start_waypoints_arg,
        robot_base_launch,
        delayed_lidar_launch,
        delayed_localization_launch,
        delayed_nav2_launch,
        delayed_waypoint_launch,
    ])
