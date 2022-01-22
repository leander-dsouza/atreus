#! /usr/bin/env python3
"""
Bringup Behavior Tree Navigator in the Nav_Stack
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
    bt_navigator_yaml_params = 'bt_navigator_params.yaml'
    bt_navigator_xml_params = 'navigate_w_replanning_and_recovery.xml'
    # ...............................................................

    pkg_dir = get_package_share_directory('atreus')


    return LaunchDescription([

        DeclareLaunchArgument("use_sim_time", \
            default_value=str(use_sim_time), \
            description="Use simulation/Gazebo clock"),

        DeclareLaunchArgument("bt_navigator_xml_params_file", \
            default_value=bt_navigator_xml_params, \
            description="Behaviour Tree XML Params"),

        DeclareLaunchArgument("bt_navigator_yaml_params_file", \
            default_value=bt_navigator_yaml_params, \
            description="Behaviour Tree YAML Params"),

        Node(
            package='nav2_bt_navigator',
            executable='bt_navigator',
            name='bt_navigator',
            output='screen',
            parameters=[
                {'use_sim_time': \
                    LaunchConfiguration('use_sim_time')},

                {'default_bt_xml_filename': \
                    [os.path.join(get_package_share_directory("nav2_bt_navigator"), \
                    "behavior_trees/"), LaunchConfiguration("bt_navigator_xml_params_file")]},

                [os.path.join(pkg_dir, 'config', 'navigation', 'bt_navigator/'), \
                    LaunchConfiguration("bt_navigator_yaml_params_file")]]
        ),

        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_bt_navigator',
            output='screen',
            parameters=[
                {'use_sim_time': LaunchConfiguration('use_sim_time')},
            	{'autostart': True},
            	{'node_names': ['bt_navigator']}]
        )

    ])
