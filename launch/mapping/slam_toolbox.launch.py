#! /usr/bin/env python3
"""Script to launch Gazebo, RViz, and SLAM Toolbox for the Atreus robot."""

import os

from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution


def generate_launch_description():
    """Generate the launch description for Atreus."""
    pkg_dir = get_package_share_directory("atreus")

    rviz_config_path = os.path.join(pkg_dir, "config", "rviz", "mapping.rviz")

    slam_toolbox_params_path = os.path.join(
        pkg_dir, "config", "mapping", "slam_toolbox", "slam_toolbox_params.yaml"
    )

    # Create the launch configuration variables
    use_sim_time = LaunchConfiguration("use_sim_time")
    rviz_config = LaunchConfiguration("rviz_config")

    # Declare the launch arguments
    declare_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time", default_value="True", description="Flag to enable use_sim_time"
    )
    declare_rviz_config_arg = DeclareLaunchArgument(
        "rviz_config",
        default_value=rviz_config_path,
        description="Full path to the RViz config file to use",
    )

    # Include the Nodes
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", rviz_config],
        parameters=[{"use_sim_time": use_sim_time}],
    )

    # Include the Launch files
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_dir, "launch", "description", "gazebo.launch.py"])
        ),
        launch_arguments={
            "two_d_lidar_enabled": "True",
        }.items(),
    )

    slam_toolbox_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("slam_toolbox"), "launch", "online_async_launch.py"]
            )
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "slam_params_file": slam_toolbox_params_path,
        }.items(),
    )

    ld = LaunchDescription()

    ld.add_action(declare_sim_time_arg)
    ld.add_action(declare_rviz_config_arg)

    ld.add_action(rviz_node)

    ld.add_action(gazebo_launch)
    ld.add_action(slam_toolbox_launch)

    return ld
