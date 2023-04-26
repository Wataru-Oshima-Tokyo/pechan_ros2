import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration

def generate_launch_description():

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    controller_yaml_box_bot_1 = os.path.join(get_package_share_directory(
        'pechan_planner_server'), 'config', 'controller_box_bot_1.yaml')
    bt_navigator_yaml_box_bot_1 = os.path.join(get_package_share_directory(
        'pechan_planner_server'), 'config', 'bt_navigator_box_bot_1.yaml')
    planner_yaml_box_bot_1 = os.path.join(get_package_share_directory(
        'pechan_planner_server'), 'config', 'planner_server_box_bot_1.yaml')
    recovery_yaml_box_bot_1 = os.path.join(get_package_share_directory(
        'pechan_planner_server'), 'config', 'recovery_box_bot_1.yaml')

    waypoints_yaml_box_bot_1 = os.path.join(get_package_share_directory(
        'pechan_planner_server'), 'config', 'waypoint_follower_box_bot_1.yaml')


    controller_yaml_box_bot_2 = os.path.join(get_package_share_directory(
        'pechan_planner_server'), 'config', 'controller_box_bot_2.yaml')
    bt_navigator_yaml_box_bot_2 = os.path.join(get_package_share_directory(
        'pechan_planner_server'), 'config', 'bt_navigator_box_bot_2.yaml')
    planner_yaml_box_bot_2 = os.path.join(get_package_share_directory(
        'pechan_planner_server'), 'config', 'planner_server_box_bot_2.yaml')
    recovery_yaml_box_bot_2 = os.path.join(get_package_share_directory(
        'pechan_planner_server'), 'config', 'recovery_box_bot_2.yaml')
    waypoints_yaml_box_bot_2 = os.path.join(get_package_share_directory(
        'pechan_planner_server'), 'config', 'waypoint_follower_box_bot_2.yaml')

    box_bot_1_rviz_config_dir = os.path.join(
        get_package_share_directory('pechan_cartographer_slam'),
        'rviz',
        'box_bot_1_view.rviz')

    box_bot_2_rviz_config_dir = os.path.join(
        get_package_share_directory('pechan_cartographer_slam'),
        'rviz',
        'box_bot_2_view.rviz')



    return LaunchDescription([

        # Nodes for box_bot_1

        Node(
            namespace='box_bot_1',
            package='nav2_controller',
            executable='controller_server',
            name='controller_server',
            output='screen',
            parameters=[controller_yaml_box_bot_1]),

        Node(
            namespace='box_bot_1',
            package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            output='screen',
            parameters=[planner_yaml_box_bot_1]),

        Node(
            namespace='box_bot_1',
            package='nav2_behaviors',
            executable='behavior_server',
            name='behavior_server',
            parameters=[recovery_yaml_box_bot_1],
            output='screen'),

        Node(
            namespace='box_bot_1',
            package='nav2_bt_navigator',
            executable='bt_navigator',
            name='bt_navigator',
            output='screen',
            parameters=[bt_navigator_yaml_box_bot_1]),

        Node(
            namespace='box_bot_1',
            package='nav2_waypoint_follower',
            executable='waypoint_follower',
            name='waypoint_follower',
            output='screen',
            parameters=[waypoints_yaml_box_bot_1]),        


        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='box_bot_1_lifecycle_manager_pathplanner',
            output='screen',
            parameters=[{'autostart': True},
                        {'bond_timeout': 0.0},
                        {'node_names': [
                            'box_bot_1/planner_server',
                            'box_bot_1/controller_server',
                            'box_bot_1/behavior_server',
                            'box_bot_1/bt_navigator',
                            'box_bot_1/waypoint_follower'
                        ]}]),




        # Nodes for box_bot_2

        Node(
            namespace='box_bot_2',
            package='nav2_controller',
            executable='controller_server',
            name='controller_server',
            output='screen',
            parameters=[controller_yaml_box_bot_2]),

        Node(
            namespace='box_bot_2',
            package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            output='screen',
            parameters=[planner_yaml_box_bot_2]),

        Node(
            namespace='box_bot_2',
            package='nav2_behaviors',
            executable='behavior_server',
            name='behavior_server',
            parameters=[recovery_yaml_box_bot_2],
            output='screen'),

        Node(
            namespace='box_bot_2',
            package='nav2_bt_navigator',
            executable='bt_navigator',
            name='bt_navigator',
            output='screen',
            parameters=[bt_navigator_yaml_box_bot_2]),

        Node(
            namespace='box_bot_2',
            package='nav2_waypoint_follower',
            executable='waypoint_follower',
            name='waypoint_follower',
            output='screen',
            parameters=[waypoints_yaml_box_bot_2]),     


        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='box_bot_2_lifecycle_manager_pathplanner',
            output='screen',
            parameters=[{'autostart': True},
                        {'bond_timeout': 0.0},
                        {'node_names': [
                            'box_bot_2/planner_server',
                            'box_bot_2/controller_server',
                            'box_bot_2/behavior_server',
                            'box_bot_2/bt_navigator',
                            'box_bot_2/waypoint_follower'
                        ]}]),
        # Node(
        #     namespace='box_bot_1',
        #     package='rviz2',
        #     executable='rviz2',
        #     name='rviz2',
        #     arguments=['-d', box_bot_1_rviz_config_dir],
        #     parameters=[{'use_sim_time': use_sim_time}],
        #     output='screen'),
        # Node(
        #     namespace='box_bot_2',
        #     package='rviz2',
        #     executable='rviz2',
        #     name='rviz2',
        #     arguments=['-d', box_bot_2_rviz_config_dir],
        #     parameters=[{'use_sim_time': use_sim_time}],
        #     output='screen')
    ])
