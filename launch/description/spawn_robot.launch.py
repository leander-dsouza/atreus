#! /usr/bin/env python3
"""
Spawn Robot in Gazebo
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command, PythonExpression
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, AppendEnvironmentVariable
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
    camera_enabled = True
    two_d_lidar_enabled = True
    rviz_enabled = True
    rviz_config = 'urdf.rviz'

    # ...............................................................


    pkg_dir = get_package_share_directory('atreus')
    gz_sim_pkg = get_package_share_directory('ros_gz_sim')

    gz_spawn_entity_node = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-topic", "/robot_description",
            "-name", "atreus",
            "-x", "0",
            "-y", "0",
            "-z", "0.5",
        ]
    )

    gz_ros2_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            "/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist",
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
            "/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry",
            "/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V",
            "/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan",
            "/camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo",
            "/camera/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked",
            "/imu@sensor_msgs/msg/Imu[gz.msgs.IMU",
            "/world/default/model/atreus/joint_state@sensor_msgs/msg/JointState[gz.msgs.Model"
        ],
        remappings=[
            ('/cmd_vel', '/cmd_vel'),
            ('/odom', '/odom'),
            ('/tf', '/tf'),
            ('/scan', '/scan'),
            ('/camera/camera_info', '/camera/camera_info'),
            ('/camera/points', '/camera/points'),
            ('/imu', '/imu'),
            ('/world/default/model/atreus/joint_state', '/joint_states')
        ]
    )


    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': Command( \
            ['xacro ', os.path.join(pkg_dir, 'urdf/atreus.xacro'),
            ' camera_enabled:=',      LaunchConfiguration('camera_enabled'),
            ' two_d_lidar_enabled:=', LaunchConfiguration('two_d_lidar_enabled'),
            ])}]
    )

    gz_sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(gz_sim_pkg, "launch", "gz_sim.launch.py")),
        launch_arguments={
            "gz_args" : PythonExpression(
                ["'", os.path.join(pkg_dir, 'worlds', world_name), " -r'"])
        }.items()
    )

    rviz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_dir, "launch", "description", "rviz.launch.py")),
        launch_arguments={
            'gazebo_enabled': 'True',
            'rviz_config': LaunchConfiguration('rviz_config')
            }.items(),
        condition=IfCondition(LaunchConfiguration('rviz_enabled'))
    )

    return LaunchDescription([

        AppendEnvironmentVariable(
            name='GZ_SIM_RESOURCE_PATH',
                value=os.path.join(pkg_dir, "worlds")),

        AppendEnvironmentVariable(
            name='GZ_SIM_RESOURCE_PATH',
                value=os.path.join(pkg_dir, "models")),

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

        robot_state_publisher_node,
        gz_ros2_bridge,
        gz_spawn_entity_node,

        gz_sim_launch,
        rviz_launch,

    ])
