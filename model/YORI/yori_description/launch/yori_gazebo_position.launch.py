#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, RegisterEventHandler, ExecuteProcess
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node

import xacro

def generate_launch_description():
    # Package Directories
    pkg_yori_description = get_package_share_directory('yori_description')

    # gazebo
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('gazebo_ros'), 'launch'), '/gazebo.launch.py']),
    )

    # read urdf
    robot_description_file = os.path.join(
        pkg_yori_description,
        'urdf',
        'yori_position.xacro.urdf')

    doc = xacro.parse(open(robot_description_file))
    xacro.process_doc(doc)
    robot_description = {'robot_description': doc.toxml()}

    # Robot state publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[robot_description],
        output='screen')

    # Spawn
    spawn = Node(package='gazebo_ros', executable='spawn_entity.py',
                arguments=[
                    '-topic', 'robot_description',
                    '-entity', 'yori',
                    ],
                output='screen',
                )
    
    load_joint_state_broadcaster = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'start',
             'joint_state_broadcaster'],
        output='screen'
    )

    load_joint_position_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'start',
             'position_controllers'],
        output='screen',
    )

    return LaunchDescription(
        [
            RegisterEventHandler(
                event_handler=OnProcessExit(
                    target_action=spawn,
                    on_exit=[load_joint_state_broadcaster],
                )
            ),
            RegisterEventHandler(
                event_handler=OnProcessExit(
                    target_action=load_joint_state_broadcaster,
                    on_exit=[load_joint_position_controller],
                )
            ),
            gazebo,
            robot_state_publisher,
            spawn, 
        ]
    )