import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    nav2_launch_file_dir = os.path.join(get_package_share_directory('nav2_bringup'), 'launch')

    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    map_dir = LaunchConfiguration(
        'map',
        default=os.path.join(
            get_package_share_directory('turtlebot3_navigation2'),
            'map',
            'map.yaml'))

    param_dir = LaunchConfiguration(
        'params_file',
        default=os.path.join(
            get_package_share_directory('pechan_planner_server'),
            'config',
            'box_bot_1.yaml'))



    box_bot_1_config = os.path.join(get_package_share_directory(
        'pechan_localization_server'), 'config', 'box_bot_1_amcl_config_turtleworld.yaml')
    box_bot_2_config = os.path.join(get_package_share_directory(
        'pechan_localization_server'), 'config', 'box_bot_2_amcl_config_turtleworld.yaml')

    map_file = os.path.join(get_package_share_directory(
        'pechan_cartographer_slam'), 'config', 'turtleworld.yaml')

    rviz_config_dir = os.path.join(
        get_package_share_directory('pechan_cartographer_slam'),
        'rviz',
        'nav2_default_view.rviz')

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([nav2_launch_file_dir, '/bringup_launch.py']),
            launch_arguments={
                'map': map_dir,
                'use_sim_time': use_sim_time,
                'params_file': param_dir}.items(),
        ),
    ])
