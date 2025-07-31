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

"""Script to launch RViz for the Atreus robot."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    """Generate the launch description for Atreus."""
    pkg_dir = get_package_share_directory('atreus')

    rviz_config_path = os.path.join(pkg_dir, 'config', 'rviz', 'urdf.rviz')

    # Create the launch configuration variables
    gazebo_enabled = LaunchConfiguration('gazebo_enabled')
    camera_enabled = LaunchConfiguration('camera_enabled')
    two_d_lidar_enabled = LaunchConfiguration('two_d_lidar_enabled')
    rviz_config = LaunchConfiguration('rviz_config')
    gui = LaunchConfiguration('gui')

    # Declare the launch arguments
    declare_use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time', default_value='True', description='Flag to enable use_sim_time'
    )
    declare_gazebo_enabled_arg = DeclareLaunchArgument(
        'gazebo_enabled',
        default_value='False',
        description='Flag to indicate if Gazebo is enabled',
    )
    declare_camera_enabled_arg = DeclareLaunchArgument(
        'camera_enabled', default_value='False', description='Flag to enable camera'
    )
    declare_two_d_lidar_enabled_arg = DeclareLaunchArgument(
        'two_d_lidar_enabled',
        default_value='False',
        description='Flag to enable 2D LiDAR',
    )
    declare_rviz_config_arg = DeclareLaunchArgument(
        'rviz_config',
        default_value=rviz_config_path,
        description='Full path to the RViz config file to use',
    )
    declare_gui_arg = DeclareLaunchArgument(
        'gui',
        default_value='True',
        description='Flag to enable joint_state_publisher_gui',
    )

    # Include the Nodes
    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        condition=IfCondition(
            PythonExpression(
                ["'", gui, "' == 'True' and '", gazebo_enabled, "' == 'False'"]
            )
        ),
    )
    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        condition=IfCondition(
            PythonExpression(
                ["'", gui, "' == 'False' and '", gazebo_enabled, "' == 'False'"]
            )
        ),
    )
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[
            {
                'robot_description': Command(
                    [
                        'xacro ',
                        os.path.join(pkg_dir, 'urdf', 'atreus.xacro'),
                        ' camera_enabled:=',
                        camera_enabled,
                        ' two_d_lidar_enabled:=',
                        two_d_lidar_enabled,
                    ]
                )
            }
        ],
        condition=UnlessCondition(gazebo_enabled),
    )
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        output='screen',
        arguments=['-d', rviz_config],
    )

    ld = LaunchDescription()

    ld.add_action(declare_use_sim_time_arg)
    ld.add_action(declare_gazebo_enabled_arg)
    ld.add_action(declare_camera_enabled_arg)
    ld.add_action(declare_two_d_lidar_enabled_arg)
    ld.add_action(declare_rviz_config_arg)
    ld.add_action(declare_gui_arg)

    ld.add_action(joint_state_publisher_gui_node)
    ld.add_action(joint_state_publisher_node)
    ld.add_action(robot_state_publisher_node)
    ld.add_action(rviz_node)

    return ld
