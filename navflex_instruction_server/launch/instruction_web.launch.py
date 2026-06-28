from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs):
    del args, kwargs
    start_instruction_server = LaunchConfiguration('start_instruction_server').perform(context).lower()
    actions = []

    if start_instruction_server in ['1', 'true', 'yes', 'on']:
        params_file = LaunchConfiguration('params_file').perform(context)
        instruction_params = [{
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
            instruction_params.append(params_file)
        actions.append(Node(
            package='navflex_instruction_server',
            executable='navflex_instruction_server_node.py',
            name='navflex_instruction_server',
            output='screen',
            parameters=instruction_params,
        ))

    actions.append(Node(
        package='navflex_instruction_server',
        executable='navflex_instruction_web.py',
        name='navflex_instruction_web',
        output='screen',
        parameters=[{
            'host': LaunchConfiguration('host'),
            'port': LaunchConfiguration('port'),
            'service_timeout': LaunchConfiguration('service_timeout'),
            'llm_enabled': LaunchConfiguration('llm_enabled'),
            'llm_backend': LaunchConfiguration('llm_backend'),
            'llm_model': LaunchConfiguration('llm_model'),
            'codex_command': LaunchConfiguration('codex_command'),
            'codex_args': LaunchConfiguration('codex_args'),
            'openai_api_url': LaunchConfiguration('openai_api_url'),
            'openai_api_key': LaunchConfiguration('openai_api_key'),
            'openai_api_key_env': LaunchConfiguration('openai_api_key_env'),
            'openai_proxy': LaunchConfiguration('openai_proxy'),
            'openai_proxy_env': LaunchConfiguration('openai_proxy_env'),
            'llm_timeout': LaunchConfiguration('llm_timeout'),
            'auto_execute_chat': LaunchConfiguration('auto_execute_chat'),
        }],
    ))
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('start_instruction_server', default_value='true'),
        DeclareLaunchArgument('host', default_value='0.0.0.0'),
        DeclareLaunchArgument('port', default_value='8080'),
        DeclareLaunchArgument('service_timeout', default_value='120.0'),
        DeclareLaunchArgument('llm_enabled', default_value='true'),
        DeclareLaunchArgument('llm_backend', default_value='openai'),
        DeclareLaunchArgument('llm_model', default_value='gpt-5.5'),
        DeclareLaunchArgument('codex_command', default_value='codex'),
        DeclareLaunchArgument('codex_args', default_value='["exec", "--skip-git-repo-check", "-"]'),
        DeclareLaunchArgument('openai_api_url', default_value='https://api.openai.com/v1/responses'),
        DeclareLaunchArgument('openai_api_key', default_value=''),
        DeclareLaunchArgument('openai_api_key_env', default_value='OPENAI_API_KEY'),
        DeclareLaunchArgument('openai_proxy', default_value=''),
        DeclareLaunchArgument('openai_proxy_env', default_value=''),
        DeclareLaunchArgument('llm_timeout', default_value='20.0'),
        DeclareLaunchArgument('auto_execute_chat', default_value='true'),
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
