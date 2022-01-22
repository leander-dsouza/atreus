#! /usr/bin/env python3
"""
Bringup Local Planner and Costmap in the Nav_Stack
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch.conditions import LaunchConfigurationEquals
from launch_ros.actions import Node



def generate_launch_description():
    """
    Launch Function
    """

    # .................. Configurable Arguments .....................

    use_sim_time = True
    local_planner_method = 'dwb'
    # ...............................................................

    pkg_dir = get_package_share_directory('atreus')
    controller_server_params = '%s_controller_server_params.yaml' \
        % local_planner_method


    return LaunchDescription([

        DeclareLaunchArgument("use_sim_time", \
            default_value=str(use_sim_time), \
                description="Use simulation/Gazebo clock"),

        DeclareLaunchArgument("controller_server_params_file", \
            default_value=controller_server_params, \
                description="Controller Server Params"),

        DeclareLaunchArgument("local_planner_method", \
            default_value=local_planner_method, \
                description="Local Planning Method"),


        Node(
            package='nav2_controller',
            executable='controller_server',
            name='controller_server',
            output='screen',
            parameters=[
                {'use_sim_time': LaunchConfiguration('use_sim_time')},

                [os.path.join(pkg_dir, 'config', 'navigation', \
                    'controller_server', 'dwb/'), \
                        LaunchConfiguration("controller_server_params_file")]],

            condition=LaunchConfigurationEquals('local_planner_method', 'dwb')
        ),

        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_controller_server',
            output='screen',
            parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')},
                        {'autostart': True},
                        {'node_names': ['controller_server']}]
        )

    ])
