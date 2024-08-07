#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, RegisterEventHandler, ExecuteProcess, DeclareLaunchArgument
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command, FindExecutable

from launch_ros.actions import Node

import xacro

def generate_launch_description():

    position_control_mode = 'position_mode'
    effort_control_mode = 'effort_mode'
    trajectory_control_mode = 'trajectory_mode'

    position_mode = LaunchConfiguration(position_control_mode)
    effort_mode = LaunchConfiguration(effort_control_mode)
    trajectory_mode = LaunchConfiguration(trajectory_control_mode)

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
        'yori.xacro.urdf')

    robot_description = Command(
        [FindExecutable(name='xacro'), '', robot_description_file, 'position_mode:=', position_mode,
        'effort_control_mode:=', effort_mode, 'trajectory_control_mode:=', trajectory_mode ]
    )


    # doc = xacro.parse(open(robot_description_file))
    # xacro.process_doc(doc)
    # robot_description = {'robot_description': doc.toxml()}

    # # read urdf
    # robot_description_file = os.path.join(
    #     pkg_yori_description,
    #     'urdf',
    #     'yori_gazebo_position.urdf')

    # with open(robot_description_file, 'r') as infp:
    #     robot_description_config = infp.read()

    # robot_description = {"robot_description": robot_description_config}

    # Robot state publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        # parameters=[robot_description],
        parameters=[{'robot_description': robot_description}],
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
             'position_controller'],
        output='screen'
    )

    # joint_position_publisher
    joint_position_publisher = Node(package='yori_description', 
                                    executable='joint_position_publisher_test',
                                    output='screen',
                                    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                position_control_mode,
                default_value='true',
                description='Position Control Mode'
            ),
            DeclareLaunchArgument(
                effort_control_mode,
                default_value='false',
                description='Torque Control Mode'
            ),
            DeclareLaunchArgument(
                trajectory_control_mode,
                default_value='true',
                description='Trajectory Control Mode (Position)'
            ),
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
            # joint_position_publisher    
        ]
    )