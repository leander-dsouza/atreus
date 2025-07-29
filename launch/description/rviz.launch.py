#! /usr/bin/env python
"""
Spawn Robot in RViz
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, LaunchConfiguration, PythonExpression


def generate_launch_description():
    """
    Launch Function
    """
    pkg_dir = get_package_share_directory("atreus")

    rviz_config_path = os.path.join(pkg_dir, "config", "rviz", "urdf.rviz")

    # Create the launch configuration variables
    gazebo_enabled = LaunchConfiguration("gazebo_enabled")
    camera_enabled = LaunchConfiguration("camera_enabled")
    two_d_lidar_enabled = LaunchConfiguration("two_d_lidar_enabled")
    rviz_config = LaunchConfiguration("rviz_config")
    gui = LaunchConfiguration("gui")

    # Declare the launch arguments
    declare_use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time", default_value="True", description="Flag to enable use_sim_time"
    )
    declare_gazebo_enabled_arg = DeclareLaunchArgument(
        "gazebo_enabled",
        default_value="False",
        description="Flag to indicate if Gazebo is enabled",
    )
    declare_camera_enabled_arg = DeclareLaunchArgument(
        "camera_enabled", default_value="False", description="Flag to enable camera"
    )
    declare_two_d_lidar_enabled_arg = DeclareLaunchArgument(
        "two_d_lidar_enabled",
        default_value="False",
        description="Flag to enable 2D LiDAR",
    )
    declare_rviz_config_arg = DeclareLaunchArgument(
        "rviz_config",
        default_value=rviz_config_path,
        description="Full path to the RViz config file to use",
    )
    declare_gui_arg = DeclareLaunchArgument(
        "gui",
        default_value="True",
        description="Flag to enable joint_state_publisher_gui",
    )

    # Include the Nodes
    joint_state_publisher_gui_node = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        name="joint_state_publisher_gui",
        condition=IfCondition(
            PythonExpression(
                ["'", gui, "' == 'True' and '", gazebo_enabled, "' == 'False'"]
            )
        ),
    )
    joint_state_publisher_node = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        name="joint_state_publisher",
        condition=IfCondition(
            PythonExpression(
                ["'", gui, "' == 'False' and '", gazebo_enabled, "' == 'False'"]
            )
        ),
    )
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[
            {
                "robot_description": Command(
                    [
                        "xacro ",
                        os.path.join(pkg_dir, "urdf", "atreus.xacro"),
                        " camera_enabled:=",
                        camera_enabled,
                        " two_d_lidar_enabled:=",
                        two_d_lidar_enabled,
                    ]
                )
            }
        ],
        condition=UnlessCondition(gazebo_enabled),
    )
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        output="screen",
        arguments=["-d", rviz_config],
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
