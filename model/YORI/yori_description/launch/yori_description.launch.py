#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    
    # path to rviz config file 
    rviz_display_config_file = os.path.join(
        get_package_share_directory('yori_description'),
        'rviz',
        'yori_rviz.rviz')

    # path to urdf file
    urdf_file = os.path.join(
        get_package_share_directory('yori_description'),
        'urdf',
        'yori_rviz.urdf')
    with open(urdf_file, 'r') as infp:
        robot_description_file = infp.read()
    
    robot_description = {"robot_description": robot_description_file}    

    launch_description = LaunchDescription()

    # robot state publisher node
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[robot_description],
        output='screen')

    # joint state publisher gui node
    joint_state_publisher_gui = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        output='screen')

    # rviz
    rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_display_config_file],
        output='screen')
    
    launch_description.add_action(robot_state_publisher)
    launch_description.add_action(joint_state_publisher_gui)
    launch_description.add_action(rviz2)

    return launch_description