#! /usr/bin/env python3
"""
Bringup the Global Planner and Costmap in the Nav_Stack
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
    global_planner_method = 'navfn'
    # ...............................................................

    pkg_dir = get_package_share_directory('atreus')

    planner_server_params = \
        '%s_planner_server_params.yaml' % global_planner_method


    return LaunchDescription([

        DeclareLaunchArgument("use_sim_time", \
            default_value=str(use_sim_time), \
                description="Use simulation/Gazebo clock"),

        DeclareLaunchArgument("planner_server_params_file", \
            default_value=planner_server_params, \
                description="Planner Server Params"),

        DeclareLaunchArgument("global_planner_method", \
            default_value=global_planner_method, \
                description="Global Planning Method"),

        Node(
            package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            output='screen',
            parameters=[
                {'use_sim_time': \
                    LaunchConfiguration('use_sim_time')},

                [os.path.join(pkg_dir, 'config', 'navigation', \
                    'planner_server', '{}/'.format(global_planner_method)), \
                        LaunchConfiguration("planner_server_params_file")]],
        ),

        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_planner_server',
            output='screen',
            parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')},
                        {'autostart': True},
                        {'node_names': ['planner_server']}]
        )

    ])
