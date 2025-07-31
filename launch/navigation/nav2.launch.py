#!/usr/bin/env python3

# Copyright (c) 2025, Leander Stephen Desouza
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
# THE SOFTWARE.

"""Script to launch Gazebo, RViz, and Nav2 for the Atreus robot."""

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    """Generate the launch description for Atreus."""
    pkg_dir = get_package_share_directory('atreus')

    warehouse_map = os.path.join(
        pkg_dir, 'config', 'mapping', 'maps', 'warehouse_map', 'warehouse_map.yaml'
    )

    # Create the launch configuration variables
    map_yaml_file = LaunchConfiguration('map')
    use_sim_time = LaunchConfiguration('use_sim_time')
    params_file = LaunchConfiguration('params_file')
    rviz_config_file = LaunchConfiguration('rviz_config_file')

    # Declare the launch arguments
    declare_map_yaml_arg = DeclareLaunchArgument(
        'map',
        default_value=warehouse_map,
        description='Full path to map file to load',
    )
    declare_use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true',
    )

    declare_params_file_arg = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(pkg_dir, 'config', 'navigation', 'nav2_params.yaml'),
        description='Full path to the ROS2 parameters file to use',
    )

    declare_rviz_config_file_arg = DeclareLaunchArgument(
        'rviz_config_file',
        default_value=os.path.join(pkg_dir, 'config', 'rviz', 'navigation.rviz'),
        description='Full path to the RViz config file to use',
    )

    # Include the launch files
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_dir, 'launch', 'description', 'gazebo.launch.py'])
        ),
        launch_arguments={
            'two_d_lidar_enabled': 'True',
        }.items(),
    )

    nav2_rviz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare('nav2_bringup'), 'launch', 'rviz_launch.py']
            )
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'rviz_config': rviz_config_file,
        }.items(),
    )

    nav2_bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare('nav2_bringup'), 'launch', 'bringup_launch.py']
            )
        ),
        launch_arguments={
            'map': map_yaml_file,
            'use_sim_time': use_sim_time,
            'params_file': params_file,
        }.items(),
    )

    ld = LaunchDescription()

    ld.add_action(declare_map_yaml_arg)
    ld.add_action(declare_use_sim_time_arg)
    ld.add_action(declare_params_file_arg)
    ld.add_action(declare_rviz_config_file_arg)

    ld.add_action(gazebo_launch)
    ld.add_action(nav2_bringup_launch)
    ld.add_action(nav2_rviz_launch)

    return ld
