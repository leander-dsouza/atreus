#! /usr/bin/env python3
"""
Bringup Waypoint Follower in the Nav_Stack
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
    waypoint_follower_params = 'waypoint_follower_params.yaml'

    # ...............................................................

    pkg_dir = get_package_share_directory('atreus')


    return LaunchDescription([

        DeclareLaunchArgument("use_sim_time", \
            default_value=str(use_sim_time), \
                description="Use simulation/Gazebo clock"),

        DeclareLaunchArgument("waypoint_follower_params_file", \
            default_value=waypoint_follower_params, \
                description="Waypoint Follower Params"),

        Node(
            package='nav2_waypoint_follower',
            executable='waypoint_follower',
            name='waypoint_follower',
            output='screen',
            parameters=[
                {'use_sim_time': \
                    LaunchConfiguration('use_sim_time')},

                [os.path.join(pkg_dir, 'config', 'navigation', 'waypoint_follower/'), \
                    LaunchConfiguration("waypoint_follower_params_file")]]
        ),

        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_waypoint_follower',
            output='screen',
            parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')},
                        {'autostart': True},
                        {'node_names': ['waypoint_follower']}]
        )

    ])
