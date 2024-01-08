#! /usr/bin/env python3
"""
Spawn Robot in Gazebo
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, ThisLaunchFileDir, Command
from launch.actions import ExecuteProcess, DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch_ros.actions import Node


def generate_launch_description():
    """
    Launch Function
    """

    # .................. Configurable Arguments .....................

    use_sim_time = True
    gui = False

    world_name = 'empty.world'
    camera_enabled = False
    two_d_lidar_enabled = False
    rviz_enabled = False
    rviz_config = 'urdf.rviz'

    # ...............................................................


    pkg_dir = get_package_share_directory('atreus')

    # os.environ["GAZEBO_MODEL_PATH"] = \
    #     os.path.join(pkg_dir, 'models')   # Add to model directory

    os.environ["GAZEBO_MODEL_PATH"] = \
        os.path.join(pkg_dir, 'models', 'warehouse_models')  # Use this in warehouse world


    return LaunchDescription([

        DeclareLaunchArgument('gui', \
            default_value=str(gui), \
                description='Flag to enable joint_state_publisher_gui'),

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

        ExecuteProcess(
            cmd=['gazebo', '--verbose', \
                [os.path.join(pkg_dir, 'worlds/'), \
                    LaunchConfiguration("world_name")],
            '-s', 'libgazebo_ros_init.so',
            '-s', 'libgazebo_ros_factory.so'],
            output='screen'),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': Command( \
                ['xacro ', os.path.join(pkg_dir, 'urdf/atreus.xacro'),
                ' camera_enabled:=',      LaunchConfiguration('camera_enabled'),
                ' two_d_lidar_enabled:=', LaunchConfiguration('two_d_lidar_enabled'),
                ])}]
        ),

        # Node(
        #     package='joint_state_publisher_gui',
        #     executable='joint_state_publisher_gui',
        #     name='joint_state_publisher_gui',
        #     condition=IfCondition(LaunchConfiguration('gui'))
        # ),


        # Node(
        #     package='joint_state_publisher',
        #     executable='joint_state_publisher',
        #     name='joint_state_publisher',
        #     condition=UnlessCondition(LaunchConfiguration('gui'))
        # ),

        Node(
            package='gazebo_ros', executable='spawn_entity.py',
            arguments=['-topic', 'robot_description',
                       '-entity', 'atreus',
                       '-x', '0',
                       '-y', '0',
                       '-z', '0.5'
                      ],
            output='screen'
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource( \
                [ThisLaunchFileDir(), '/rviz.launch.py']),
            launch_arguments={
                'gazebo_enabled': 'True',
                'rviz_config': LaunchConfiguration('rviz_config')
                }.items(),
            condition=IfCondition(LaunchConfiguration('rviz_enabled'))
        ),

        # print(ThisLaunchFileDir())

    ])
