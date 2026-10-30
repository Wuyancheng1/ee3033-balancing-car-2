#!/usr/bin/env python3

import os
# import sys

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription,
    LogInfo,
    TimerAction,
)
from launch.launch_description_sources import (
    PythonLaunchDescriptionSource,
)

from launch_ros.actions import Node

def generate_launch_description():
    """
    Startup sequence:

    1. Start the robot base immediately.
    2. Start the LiDAR after 2 seconds.
    3. Start scan_resampler.py after 4 seconds.
    4. Start SLAM Toolbox after 7 seconds.
    """

    # ---------------------------------------------------------
    # Locate launch files
    # ---------------------------------------------------------

    robot_base_launch_file = os.path.join(
        get_package_share_directory('balance_robot_base'),
        'launch',
        'balance_robot_base.launch.py'
    )

    lidar_launch_file = os.path.join(
        get_package_share_directory('lslidar_driver_n10p'),
        'launch',
        'lslidar_launch.py'
    )

    mapping_launch_file = os.path.join(
        get_package_share_directory('balance_mapping'),
        'launch',
        'balance_mapping_slamtoolbox_async.launch.py'
    )

    # Location of the standalone scan-resampler script
    scan_resampler_file = '/home/ros2_ws/scan_resampler.py'

    # ---------------------------------------------------------
    # Robot base: start immediately
    # ---------------------------------------------------------

    robot_base_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            robot_base_launch_file
        )
    )

    # ---------------------------------------------------------
    # LiDAR: start after 2 seconds
    # ---------------------------------------------------------

    lidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            lidar_launch_file
        )
    )

    delayed_lidar_launch = TimerAction(
        period=2.0,
        actions=[
            lidar_launch
        ]
    )

    # ---------------------------------------------------------
    # Scan resampler: start after 4 seconds
    #
    # This gives the LiDAR approximately 2 seconds to start
    # publishing /scan before the resampler starts.
    # ---------------------------------------------------------

    # scan_resampler_process = ExecuteProcess(
    #     cmd=[
    #         sys.executable,
    #         scan_resampler_file,
    #         '--ros-args',
    #         '-p', 'input_topic:=/scan',
    #         '-p', 'output_topic:=/scan_fixed',
    #         '-p', 'number_of_bins:=450',
    #     ],
    #     output='screen',
    # )

    # delayed_scan_resampler = TimerAction(
    #     period=4.0,
    #     actions=[
    #         scan_resampler_process
    #     ]
    # )


    scan_resampler_node = Node(
        package='balance_mapping',
        executable='scan_resampler.py',
        name='scan_resampler',
        output='screen',
        parameters=[{
            'input_topic': '/scan',
            'output_topic': '/scan_fixed',
            'number_of_bins': 450,
        }],
    )

    delayed_scan_resampler = TimerAction(
        period=4.0,
        actions=[
            LogInfo(
                msg='[bringup] Starting scan resampler...'
            ),
            scan_resampler_node,
        ],
    )

    # ---------------------------------------------------------
    # SLAM Toolbox: start after 7 seconds
    #
    # This gives /scan_fixed approximately 3 seconds to become
    # available before SLAM Toolbox starts.
    # ---------------------------------------------------------

    mapping_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            mapping_launch_file
        )
    )

    delayed_mapping_launch = TimerAction(
        period=7.0,
        actions=[
            mapping_launch
        ]
    )

    return LaunchDescription([
        robot_base_launch,
        delayed_lidar_launch,
        delayed_scan_resampler,
        delayed_mapping_launch,
    ])
