#! /usr/bin/env python3
"""
Main Launch file
"""
from launch import LaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, ThisLaunchFileDir
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, LaunchConfigurationEquals


def generate_launch_description():
    """
    Launch Function
    """

    # .................. Configurable Arguments .....................

    use_sim_time = True

    world_name = 'empty.world'
    camera_enabled = False
    two_d_lidar_enabled = False
    rviz_enabled = False
    rviz_config = 'urdf.rviz'

    mapping_method = ''
    navigation_enabled = False

    # ...............................................................


    return LaunchDescription([

        DeclareLaunchArgument("use_sim_time", \
            default_value=str(use_sim_time), \
                description="Use simulation/Gazebo clock"),

        DeclareLaunchArgument("world_name", \
            default_value=world_name, \
                description="Choice of Gazebo World"),

        DeclareLaunchArgument("camera_enabled", \
            default_value=str(camera_enabled), \
                description="Camera Xacro Argument"),

        DeclareLaunchArgument("two_d_lidar_enabled", \
            default_value=str(two_d_lidar_enabled), \
                description="2D LiDAR Xacro Argument"),

        DeclareLaunchArgument("rviz_enabled", \
            default_value=str(rviz_enabled), \
                description="Start RViz"),

        DeclareLaunchArgument("rviz_config", \
            default_value=rviz_config, \
                description="RViz Config"),

        DeclareLaunchArgument("mapping_method", \
            default_value=mapping_method, \
                description="Mapping Method"),

        DeclareLaunchArgument("navigation_enabled", \
            default_value=str(navigation_enabled), \
                description="Start Nav2 Stack"),


        IncludeLaunchDescription(
            PythonLaunchDescriptionSource( \
                [ThisLaunchFileDir(), '/description', '/spawn_robot.launch.py']),
            launch_arguments={
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'world_name': LaunchConfiguration('world_name'),
                'camera_enabled': LaunchConfiguration('camera_enabled'),
                'two_d_lidar_enabled': LaunchConfiguration('two_d_lidar_enabled'),
                'rviz_enabled': LaunchConfiguration('rviz_enabled'),
                'rviz_config': LaunchConfiguration('rviz_config')
                }.items(),
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource( \
                [ThisLaunchFileDir(), '/../navigation', '/slam_toolbox.launch.py']),
            launch_arguments={
                'use_sim_time': LaunchConfiguration('use_sim_time')
                }.items(),
            condition=LaunchConfigurationEquals('mapping_method', 'slam_toolbox')
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource( \
                [ThisLaunchFileDir(), '/../navigation', '/navigation.launch.py']),
            launch_arguments={
                'use_sim_time': LaunchConfiguration('use_sim_time')
                }.items(),
            condition=IfCondition(LaunchConfiguration('navigation_enabled'))
        )

    ])
