from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    nav2_share = get_package_share_directory('nav2_bringup')
    bringup_launch = os.path.join(nav2_share, 'launch', 'bringup_launch.py')
    default_params = os.path.join(
        get_package_share_directory('physical_ai_robot_description'),
        'config', 'nav2_mdh100.yaml')

    return LaunchDescription([
        DeclareLaunchArgument('map', description='Absolute path to a saved map YAML file'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('params_file', default_value=default_params),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(bringup_launch),
            launch_arguments={
                'map': LaunchConfiguration('map'),
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'params_file': LaunchConfiguration('params_file'),
                'autostart': 'true',
            }.items(),
        ),
    ])
