import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    box_bot_1_config = os.path.join(get_package_share_directory(
        'pechan_localization_server'), 'config', 'box_bot_1_amcl_config_turtleworld.yaml')
    box_bot_2_config = os.path.join(get_package_share_directory(
        'pechan_localization_server'), 'config', 'box_bot_2_amcl_config_turtleworld.yaml')

    # map_file = os.path.join(get_package_share_directory(
    #     'pechan_cartographer_slam'), 'config', 'turtleworld.yaml')
    

    map_file = os.path.join(get_package_share_directory('sim_worlds'),
            'maps',
            'sh.yaml')

    return LaunchDescription([
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[{'use_sim_time': True},
                        {'topic_name': "map"},
                        {'frame_id': "map"},
                        {'yaml_filename': map_file}]
        ),

        Node(
            namespace='box_bot_1',
            package='nav2_amcl',
            executable='amcl',
            name='amcl',
            output='screen',
            parameters=[box_bot_1_config]
        ),

        Node(
            namespace='box_bot_2',
            package='nav2_amcl',
            executable='amcl',
            name='amcl',
            output='screen',
            parameters=[box_bot_2_config]
        ),

        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_localization',
            output='screen',
            parameters=[{'use_sim_time': True},
                        {'autostart': True},
                        {'bond_timeout': 0.0},
                        {'node_names': ['map_server', 'box_bot_1/amcl', 'box_bot_2/amcl']}]
        ),
        # Node(
        #     package='nav2_lifecycle_manager',
        #     executable='lifecycle_manager',
        #     name='lifecycle_manager_localization',
        #     output='screen',
        #     parameters=[{'use_sim_time': True},
        #                 {'autostart': True},
        #                 {'bond_timeout': 0.0},
        #                 {'node_names': ['map_server', 'box_bot_1/amcl']}]
        # ),

        # Node(
        #     package='rviz2',
        #     executable='rviz2',
        #     name='rviz2',
        #     arguments=['-d', rviz_config_dir],
        #     parameters=[{'use_sim_time': True}],
        #     output='screen')
    ])
