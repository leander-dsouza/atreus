#! /usr/bin/env python3
"""
Bringup Slam Toolbox for Mapping
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node


def generate_launch_description():
    """
    Launch Function
    """

    # .................. Configurable Arguments .....................

    use_sim_time = True
    slam_toolbox_params = 'slam_toolbox_params.yaml'
    # ...............................................................

    pkg_dir = get_package_share_directory('atreus')


    return LaunchDescription([

        DeclareLaunchArgument("use_sim_time", \
            default_value=str(use_sim_time), \
                description="Use simulation/Gazebo clock"),

        DeclareLaunchArgument("slam_toolbox_params_file", \
            default_value=slam_toolbox_params, \
                description="Config for Slam Toolbox"),

        Node(
            package='slam_toolbox',
            executable='sync_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            parameters=[
                {'use_sim_time': \
                    LaunchConfiguration('use_sim_time')},

                [os.path.join(pkg_dir, 'config', 'mapping', 'slam_toolbox/'), \
                    LaunchConfiguration("slam_toolbox_params_file")]]
        ),

    ])
