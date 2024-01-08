#! /usr/bin/env python3
"""
Bring all the components of the Nav_Stack together
"""
from launch import LaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, ThisLaunchFileDir
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable


def generate_launch_description():
    """
    Launch Function
    """

    # .................. Configurable Arguments .....................

    use_sim_time = True

    # ...............................................................


    return LaunchDescription([

        # For enabling coloured logs
        SetEnvironmentVariable('RCUTILS_COLORIZED_OUTPUT', '1'),

        DeclareLaunchArgument("use_sim_time", \
            default_value=str(use_sim_time), \
                description="Use simulation/Gazebo clock"),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource( \
                [ThisLaunchFileDir(), '/map_server.launch.py']),
            launch_arguments={
                'use_sim_time': LaunchConfiguration('use_sim_time')
                }.items(),
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource( \
                [ThisLaunchFileDir(), '/amcl.launch.py']),
            launch_arguments={
                'use_sim_time': LaunchConfiguration('use_sim_time')
                }.items(),
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource( \
                [ThisLaunchFileDir(), '/controller_server.launch.py']),
            launch_arguments={
                'use_sim_time': LaunchConfiguration('use_sim_time')
                }.items(),
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource( \
                [ThisLaunchFileDir(), '/planner_server.launch.py']),
            launch_arguments={
                'use_sim_time': LaunchConfiguration('use_sim_time')
                }.items(),
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource( \
                [ThisLaunchFileDir(), '/behavior_server.launch.py']),
            launch_arguments={
                'use_sim_time': LaunchConfiguration('use_sim_time')
                }.items(),
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource( \
                [ThisLaunchFileDir(), '/bt_navigator.launch.py']),
            launch_arguments={
                'use_sim_time': LaunchConfiguration('use_sim_time')
                }.items(),
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource( \
                [ThisLaunchFileDir(), '/waypoint_follower.launch.py']),
            launch_arguments={
                'use_sim_time': LaunchConfiguration('use_sim_time')
                }.items(),
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource( \
                [ThisLaunchFileDir(), '/velocity_smoother.launch.py']),
            launch_arguments={
                'use_sim_time': LaunchConfiguration('use_sim_time')
                }.items(),
        ),

    ])
