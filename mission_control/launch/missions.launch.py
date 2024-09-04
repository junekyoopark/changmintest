#!/usr/bin/env python

from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction, IncludeLaunchDescription, RegisterEventHandler, LogInfo
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    package_dir = get_package_share_directory('mission_control')
    # bash_script_path = os.path.join(package_dir, 'scripts', 'TerminatorScript.sh')
    pooch_package = os.path.join(get_package_share_directory('dontscrewthepooch'), 'dontscrewthepooch.launch.py')

    visualizer = Node(
        package='mission_control',
        namespace='mission_control',
        executable='visualizer',
        name='visualizer'
    )

    offboard_start = Node(
        package='mission_control',
        namespace='mission_control',
        executable='offboard_start',
        name='offboard_start',
        #prefix='gnome-terminal --'
    )

    missionNode = Node(
        package='mission_control',
        namespace='mission_control',
        executable='mission1',
        name='mission1',
        #prefix='gnome-terminal --',
    )

    launchRviz = Node(
        package='rviz2',
        namespace='',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', [os.path.join(package_dir, 'visualize.rviz')]]
    )

    event_handler = RegisterEventHandler(
        OnProcessExit(
            target_action=missionNode,
            on_exit=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(pooch_package)
                )
            ]
        )
    )

    return LaunchDescription([
        offboard_start,
        missionNode,
        launchRviz,
        visualizer
        #event_handler
        # Node(
        #     package='px4_offboard',
        #     namespace='px4_offboard',
        #     executable='fixed_frame_tf2',
        #     name='fixed_frame_tf2'
        # )
    ])