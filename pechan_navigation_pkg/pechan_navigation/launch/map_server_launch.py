import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node, SetParameter
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

TURTLEBOT3_MODEL = os.environ['TURTLEBOT3_MODEL']

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    # map_file = os.path.join(get_package_share_directory(
    #     'pechan_cartographer_slam'), 'config', 'turtleworld.yaml')

    map_file = os.path.join(get_package_share_directory(
        'turtlebot3_navigation2'), 'map', 'map.yaml')

    map_dir = LaunchConfiguration(
        'map',
        default=os.path.join(
            get_package_share_directory('turtlebot3_navigation2'),
            'map',
            'map.yaml'))
    # map_dir = LaunchConfiguration(
    #     'map',
    #     default=os.path.join(
    #         get_package_share_directory('pechan_cartographer_slam'),
    #         'config',
    #         'map_config.yaml'))
    
    # param_dir = LaunchConfiguration(
    #     'params_file',
    #     default=os.path.join(
    #         get_package_share_directory('pechan_cartographer_slam'),
    #         'config',
    #         'map_config.yaml'))


    param_file_name = TURTLEBOT3_MODEL + '.yaml'
    param_dir = LaunchConfiguration(
        'params_file',
        default=os.path.join(
            get_package_share_directory('turtlebot3_navigation2'),
            'param',
            param_file_name))



    rviz_config_dir = os.path.join(
        get_package_share_directory('pechan_cartographer_slam'),
        'rviz',
        'nav2_default_view.rviz')
    

    map_server_config_path = os.path.join(
        get_package_share_directory('pechan_planner_server'),
        'config',
        'map_config.yaml'
    )


    map_server_cmd = Node(
        package='nav2_map_server',
        executable='map_server',
        output='screen',
        parameters=[{'yaml_filename': map_file}]
    )
    lifecycle_nodes = ['map_server']
    use_sim_time = True
    autostart = True

    start_lifecycle_manager_cmd = Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager',
            output='screen',
            emulate_tty=True,  # https://github.com/ros2/launch/issues/188
            parameters=[{'use_sim_time': use_sim_time},
                        {'autostart': autostart},
                        {'node_names': lifecycle_nodes}])
    


    return LaunchDescription([
        # IncludeLaunchDescription(
        #     PythonLaunchDescriptionSource([nav2_launch_file_dir, '/bringup_launch.py']),
        #     launch_arguments={
        #         'map': map_dir,
        #         'use_sim_time': use_sim_time,
        #         'params_file': param_dir}.items(),
        # ),

        # Node(
        #     package='rviz2',
        #     executable='rviz2',
        #     name='rviz2',
        #     arguments=['-d', rviz_config_dir],
        #     parameters=[{'use_sim_time': True}],
        #     output='screen'),

        # # Node(
        # #     package='nav2_lifecycle_manager',
        # #     executable='lifecycle_manager',
        # #     name='lifecycle_manager_localization',
        # #     output='screen',
        # #     parameters=[{'use_sim_time': True},
        # #                 {'autostart': True},
        # #                 {'node_names': lifecycle_nodes}])   
        map_server_cmd,
        start_lifecycle_manager_cmd
    ])
