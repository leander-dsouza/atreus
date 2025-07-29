#! /usr/bin/env python3
"""
Spawn Robot in Gazebo
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command, PythonExpression, PathJoinSubstitution
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, AppendEnvironmentVariable
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """
    Launch Function
    """
    pkg_dir = get_package_share_directory('atreus')

    rviz_config_path = os.path.join(
        pkg_dir, 'config', 'rviz', 'urdf.rviz')
    world_path = os.path.join(pkg_dir, 'worlds', 'mapping.sdf')


     # Create the launch configuration variables
    camera_enabled = LaunchConfiguration('camera_enabled')
    two_d_lidar_enabled = LaunchConfiguration('two_d_lidar_enabled')
    rviz_enabled = LaunchConfiguration('rviz_enabled')
    rviz_config = LaunchConfiguration('rviz_config')

    # Declare the append environment variables
    append_env_var_gz_sim_resource_path = AppendEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=os.path.join(pkg_dir, "worlds") + ':' +
              os.path.join(pkg_dir, "models", "warehouse_models")
    )

    # Declare the launch arguments
    declare_use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time', default_value='True',
        description='Flag to enable use_sim_time'
    )
    declare_world_name_arg = DeclareLaunchArgument(
        'world_name', default_value=world_path,
        description='Choice of Gazebo World'
    )
    declare_camera_enabled_arg = DeclareLaunchArgument(
        'camera_enabled', default_value='False',
        description='Flag to enable camera'
    )
    declare_two_d_lidar_enabled_arg = DeclareLaunchArgument(
        'two_d_lidar_enabled', default_value='False',
        description='Flag to enable 2D LiDAR'
    )
    declare_rviz_enabled_arg = DeclareLaunchArgument(
        'rviz_enabled', default_value='False',
        description='Flag to enable RViz'
    )
    declare_rviz_config_arg = DeclareLaunchArgument(
        'rviz_config', default_value=rviz_config_path,
        description='Full path to the RViz config file to use'
    )

    # Include the Nodes
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
    gz_ros2_bridge_node = Node(
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
        parameters=[{'robot_description': Command(
            ['xacro ', os.path.join(pkg_dir, 'urdf', 'atreus.xacro'),
            ' camera_enabled:=', camera_enabled,
            ' two_d_lidar_enabled:=', two_d_lidar_enabled
            ])}]
    )

    gz_sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare('ros_gz_sim'), 'launch', 'gz_sim.launch.py'])),
        launch_arguments={
            "gz_args" : PythonExpression(
                ["'", world_path, " -r'"])
        }.items()
    )

    rviz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_dir, "launch", "description", "rviz.launch.py")),
        launch_arguments={
            'gazebo_enabled': 'True',
            'camera_enabled': camera_enabled,
            'two_d_lidar_enabled': two_d_lidar_enabled,
            'rviz_config': rviz_config,
            }.items(),
        condition=IfCondition(rviz_enabled)
    )

    ld = LaunchDescription()

    ld.add_action(append_env_var_gz_sim_resource_path)

    ld.add_action(declare_use_sim_time_arg)
    ld.add_action(declare_world_name_arg)
    ld.add_action(declare_camera_enabled_arg)
    ld.add_action(declare_two_d_lidar_enabled_arg)
    ld.add_action(declare_rviz_enabled_arg)
    ld.add_action(declare_rviz_config_arg)

    ld.add_action(robot_state_publisher_node)
    ld.add_action(gz_ros2_bridge_node)
    ld.add_action(gz_spawn_entity_node)

    ld.add_action(gz_sim_launch)
    ld.add_action(rviz_launch)

    return ld
