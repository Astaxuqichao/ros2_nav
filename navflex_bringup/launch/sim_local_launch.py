import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    map_file = LaunchConfiguration('map_file')
    rviz_config_file = LaunchConfiguration('rviz_config_file')
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')

    navflex_bringup_dir = get_package_share_directory('navflex_bringup')

    rviz_launch_path = os.path.join(
        get_package_share_directory('nav2_bringup'), 'launch', 'rviz_launch.py')
    omni_launch_path = os.path.join(
        get_package_share_directory('omni_fake_node'), 'launch', 'omni_fake_node.launch.py')
    sim_lidar_launch_path = os.path.join(
        get_package_share_directory('simulation_lidar'), 'launch', 'simulation_lidar.launch.py')

    return LaunchDescription([
        DeclareLaunchArgument(
            'map_file',
            default_value=os.path.join(navflex_bringup_dir, 'maps', 'map1.yaml'),
            description='Path to the map file'),
        DeclareLaunchArgument(
            'rviz_config_file',
            default_value=os.path.join(
                navflex_bringup_dir, 'rviz', 'nav2_default_view.rviz'),
            description='Full path to the RViz config file'),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation clock'),
        DeclareLaunchArgument(
            'autostart',
            default_value='true',
            description='Automatically configure and activate lifecycle nodes'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(rviz_launch_path),
            condition=IfCondition('True'),
            launch_arguments={
                'namespace': '',
                'use_namespace': 'False',
                'rviz_config': rviz_config_file,
            }.items()),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(omni_launch_path)),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(sim_lidar_launch_path)),

        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[
                {'yaml_filename': map_file},
                {'use_sim_time': use_sim_time},
            ]),
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_map',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'autostart': autostart,
                'node_names': ['map_server'],
            }]),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_transform_publisher',
            output='screen',
            arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom']),
        LogInfo(msg='Navflex local simulation launched.'),
    ])
