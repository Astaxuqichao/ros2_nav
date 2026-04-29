"""Launch file for omni_fake_node."""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """Generate launch description for omni_fake_node."""
    
    omni_fake_node = Node(
        package='omni_fake_node',
        executable='omni_fake_node',
        name='omni_fake_node',
        output='screen',
        parameters=[
            {'update_rate': 50.0},
            {'cmd_vel_timeout': 0.5},
            {'odom_frame': 'odom'},
            {'base_link_frame': 'base_link'},
        ]
    )

    ld = LaunchDescription()
    ld.add_action(omni_fake_node)
    return ld
