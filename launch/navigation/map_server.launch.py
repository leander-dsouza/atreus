#! /usr/bin/env python3
"""
Bringup the Map Server in the Nav_Stack
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
    map_file = 'warehouse_map'

    # ...............................................................

    pkg_dir = get_package_share_directory('atreus')


    return LaunchDescription([

        DeclareLaunchArgument("use_sim_time", \
            default_value=str(use_sim_time), \
                description="Use simulation/Gazebo clock"),

        DeclareLaunchArgument("map_file", \
            default_value=map_file, \
                description="Map file"),

        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[
                {'yaml_filename': \
                    [os.path.join(pkg_dir, 'config', 'mapping', 'maps/'),
                        LaunchConfiguration("map_file"), "/", \
                            LaunchConfiguration("map_file"), ".yaml"],

                'topic_name': 'map',
                'frame_id': 'map'}]
        ),

        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_map',
            output='screen',
            parameters=[
                {'use_sim_time': LaunchConfiguration('use_sim_time')},
                {'autostart': True},
                {'node_names': ['map_server']}]
        )

    ])
