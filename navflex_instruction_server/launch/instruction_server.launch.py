from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs):
    del args, kwargs
    params_file = LaunchConfiguration('params_file').perform(context)
    parameters = [{
        'global_frame': LaunchConfiguration('global_frame'),
        'planner_id': LaunchConfiguration('planner_id'),
        'controller_id': LaunchConfiguration('controller_id'),
        'xy_goal_tolerance': LaunchConfiguration('xy_goal_tolerance'),
        'yaw_goal_tolerance': LaunchConfiguration('yaw_goal_tolerance'),
        'behavior_id': LaunchConfiguration('behavior_id'),
        'action_timeout': LaunchConfiguration('action_timeout'),
        'feedback_publish_period': LaunchConfiguration('feedback_publish_period'),
    }]
    if params_file:
        parameters.append(params_file)

    return [Node(
        package='navflex_instruction_server',
        executable='navflex_instruction_server_node.py',
        name='navflex_instruction_server',
        output='screen',
        parameters=parameters,
    )]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('global_frame', default_value='map'),
        DeclareLaunchArgument('planner_id', default_value='GridBased'),
        DeclareLaunchArgument('controller_id', default_value='FollowPath'),
        DeclareLaunchArgument('xy_goal_tolerance', default_value='0.0'),
        DeclareLaunchArgument('yaw_goal_tolerance', default_value='0.0'),
        DeclareLaunchArgument('behavior_id', default_value='cmd_behavior'),
        DeclareLaunchArgument('action_timeout', default_value='60.0'),
        DeclareLaunchArgument('feedback_publish_period', default_value='0.5'),
        DeclareLaunchArgument('params_file', default_value=''),
        OpaqueFunction(function=launch_setup),
    ])
