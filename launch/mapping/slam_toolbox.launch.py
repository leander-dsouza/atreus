#! /usr/bin/env python3
"""
Launch SLAM Toolbox for mapping with RViz support
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    """Launch Function"""

    pkg_dir = get_package_share_directory('atreus')
    slam_toolbox_pkg = get_package_share_directory('slam_toolbox')

    rviz_config_arg = DeclareLaunchArgument(
        'rviz_config', default_value='mapping.rviz',
        description='RViz config file'
    )

    sim_time_arg = DeclareLaunchArgument(
        'use_sim_time', default_value='True',
        description='Flag to enable use_sim_time'
    )

    slam_toolbox_launch_path = os.path.join(
        slam_toolbox_pkg,
        'launch',
        'online_async_launch.py'
    )

    slam_toolbox_params_path = os.path.join(
        pkg_dir, 'config', 'mapping',
        'slam_toolbox', 'slam_toolbox_params.yaml'
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', PathJoinSubstitution(
            [pkg_dir, 'config', 'rviz', LaunchConfiguration('rviz_config')])],
        parameters=[
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
        ]
    )

    slam_toolbox_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(slam_toolbox_launch_path),
        launch_arguments={
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'slam_params_file': slam_toolbox_params_path,
        }.items()
    )

    ld = LaunchDescription()

    ld.add_action(rviz_config_arg)
    ld.add_action(sim_time_arg)
    ld.add_action(rviz_node)
    ld.add_action(slam_toolbox_launch)

    return ld
