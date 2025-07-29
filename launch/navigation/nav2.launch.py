import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_dir = get_package_share_directory('atreus')

    warehouse_map = os.path.join(
        pkg_dir, 'config', 'mapping',
        'maps', 'warehouse_map', 'warehouse_map.yaml')


    # Create the launch configuration variables
    map_yaml_file = LaunchConfiguration('map')
    use_sim_time = LaunchConfiguration('use_sim_time')
    params_file = LaunchConfiguration('params_file')
    rviz_config_file = LaunchConfiguration('rviz_config_file')

    # Declare the launch arguments
    declare_map_yaml_arg = DeclareLaunchArgument(
        'map',
        default_value=warehouse_map,
        description='Full path to map file to load',
    )
    declare_use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true')

    declare_params_file_arg = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(pkg_dir, 'config', 'navigation', 'nav2_params.yaml'),
        description='Full path to the ROS2 parameters file to use')

    declare_rviz_config_file_arg = DeclareLaunchArgument(
        'rviz_config_file',
        default_value=os.path.join(pkg_dir, 'config', 'rviz', 'navigation.rviz'),
        description='Full path to the RViz config file to use')


    # Include the launch files
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [pkg_dir, 'launch', 'description', 'gazebo.launch.py'])),
        launch_arguments={
            'two_d_lidar_enabled': 'True',
        }.items(),
    )

    nav2_rviz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
                        PathJoinSubstitution(
                [FindPackageShare('nav2_bringup'), 'launch', 'rviz_launch.py'])),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'rviz_config': rviz_config_file,
        }.items(),
    )

    nav2_bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare('nav2_bringup'), 'launch', 'bringup_launch.py'])),
        launch_arguments={
            'map': map_yaml_file,
            'use_sim_time': use_sim_time,
            'params_file': params_file,
        }.items())


    ld = LaunchDescription()

    ld.add_action(declare_map_yaml_arg)
    ld.add_action(declare_use_sim_time_arg)
    ld.add_action(declare_params_file_arg)
    ld.add_action(declare_rviz_config_file_arg)

    ld.add_action(gazebo_launch)
    ld.add_action(nav2_bringup_launch)
    ld.add_action(nav2_rviz_launch)

    return ld
