import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    navflex_bt_navigator_dir = get_package_share_directory('navflex_bt_navigator')
    default_nav_to_pose_bt_xml = os.path.join(
        navflex_bt_navigator_dir, 'behavior_trees', 'test_bt_navigator.xml')
    default_params_file = os.path.join(
        navflex_bt_navigator_dir, 'params', 'navflex_bt_navigator.yaml')

    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    bt_xml = LaunchConfiguration('default_nav_to_pose_bt_xml')
    params_file = LaunchConfiguration('params_file')
    log_level = LaunchConfiguration('log_level')

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time', default_value='false',
        description='Use simulation time')

    declare_autostart = DeclareLaunchArgument(
        'autostart', default_value='true',
        description='Automatically activate the lifecycle-managed nodes')

    declare_params_file = DeclareLaunchArgument(
        'params_file', default_value=default_params_file,
        description='Full path to the ROS2 parameters file to use for bt_navigator')

    declare_bt_xml = DeclareLaunchArgument(
        'default_nav_to_pose_bt_xml', default_value=default_nav_to_pose_bt_xml,
        description='Full path to the behavior tree XML file')

    declare_log_level = DeclareLaunchArgument(
        'log_level', default_value='info',
        description='Log level')

    stdout_linebuf_envvar = SetEnvironmentVariable(
        'RCUTILS_LOGGING_BUFFERED_STREAM', '1')

    bt_navigator_node = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        output='screen',
        parameters=[
            params_file,
            {'default_nav_to_pose_bt_xml': bt_xml}
        ],
        arguments=['--ros-args', '--log-level', log_level],
    )

    lifecycle_manager_node = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_bt_navigator',
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time},
            {'autostart': autostart},
            {'node_names': ['bt_navigator']}
        ],
        arguments=['--ros-args', '--log-level', log_level],
    )

    ld = LaunchDescription()
    ld.add_action(stdout_linebuf_envvar)
    ld.add_action(declare_use_sim_time)
    ld.add_action(declare_autostart)
    ld.add_action(declare_params_file)
    ld.add_action(declare_bt_xml)
    ld.add_action(declare_log_level)
    ld.add_action(bt_navigator_node)
    ld.add_action(lifecycle_manager_node)

    return ld
