#! /usr/bin/env python3
"""
Bringup the Recovery Behaviour in the Nav_Stack
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
    recoveries_server_params = 'recoveries_server_params.yaml'

    # ...............................................................

    pkg_dir = get_package_share_directory('atreus')


    return LaunchDescription([

        DeclareLaunchArgument("use_sim_time", \
            default_value=str(use_sim_time), \
                description="Use simulation/Gazebo clock"),

        DeclareLaunchArgument("recoveries_server_params_file", \
            default_value=recoveries_server_params, \
                description="Recovery Params"),

        Node(
            package='nav2_recoveries',
            executable='recoveries_server',
            name='recoveries_server',
            output='screen',
            parameters=[
                {'use_sim_time': \
                    LaunchConfiguration('use_sim_time')},

                [os.path.join(pkg_dir, 'config', 'navigation', 'recoveries_server/'), \
                    LaunchConfiguration("recoveries_server_params_file")]]
        ),

        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_recovery',
            output='screen',
            parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')},
                        {'autostart': True},
                        {'node_names': ['recoveries_server']}]
        )

    ])
