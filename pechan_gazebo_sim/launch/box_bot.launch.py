
#!/usr/bin/python3
# -*- coding: utf-8 -*-
import os

from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch import LaunchDescription
from launch.actions import OpaqueFunction
import launch_ros.actions
from functools import partial


def gen_robot_info(context):
    pose_1 = [float(LaunchConfiguration('x_spawn').perform(context)), float(LaunchConfiguration('y_spawn').perform(context))]

    robot_name = str(LaunchConfiguration('robot_name').perform(context))
    x_pos = pose_1[0]
    y_pos = pose_1[1]
    robot = {'name': robot_name, 'x_pose': x_pos,
             'y_pose': y_pos, 'z_pose': 0.1, 'Y_pose': 0.0}

    print("############### ROBOTS MULTI ARRAY="+str(robot))

    return robot


def launch_setup(context, *args, **kwargs):

    launch_file_dir = os.path.join(get_package_share_directory(
        'pechan_gazebo_sim'), 'launch')

    # Names and poses of the robots
    robot = gen_robot_info(context)

    # Create the launch description and populate
    ld = LaunchDescription()

    ld.add_action(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            launch_file_dir, 'spawn.launch.py')),
        launch_arguments={
            'x_spawn': TextSubstitution(text=str(robot['x_pose'])),
            'y_spawn': TextSubstitution(text=str(robot['y_pose'])),
            'yaw_spawn': TextSubstitution(text=str(robot['Y_pose'])),
            'entity_name': robot['name']
        }.items()))

    return [ld]


def generate_launch_description():
    x_spawn_arg = DeclareLaunchArgument('x_spawn', default_value='-2.0', description='X position of the robot')
    y_spawn_arg = DeclareLaunchArgument('y_spawn', default_value='-0.5', description='Y position of the robot')
    robot_name_arg = DeclareLaunchArgument('robot_name', default_value='box_bot_0', description='name of the robot')
    return LaunchDescription([
        robot_name_arg,
        x_spawn_arg,
        y_spawn_arg,
        OpaqueFunction(function=launch_setup)
    ])
