import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    nav_pkg_dir = get_package_share_directory('balance_navigation')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    
    arg_map = DeclareLaunchArgument('map', description='Full path to map yaml file to load')

    # AMCLAMCL localization 
    amcl_localization_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(nav2_bringup_dir, 'launch', 'localization_launch.py')),
        launch_arguments={
            'use_sim_time': 'false',
            'map': LaunchConfiguration('map'),
            'params_file': os.path.join(nav_pkg_dir, 'config', 'localization_params.yaml')
        }.items()
    )

    return LaunchDescription([
        arg_map,
        amcl_localization_launch
    ])