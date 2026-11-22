"""Launch file for all 11 NENUPHARES BONNEVAL V1 ROS 2 Jazzy nodes."""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    """Launch the 11-node L2 software stack."""
    executables = [
        'camera_driver',
        'vio',
        'perception_geom',
        'clearance',
        'mission_fsm',
        'corridor_monitor',
        'formation',
        'energy',
        'mavlink_bridge',
        'blackbox',
        'system_monitor',
    ]
    return LaunchDescription([
        Node(
            package='nenuphar_nodes',
            executable=exe,
            name=exe,
            output='screen',
        )
        for exe in executables
    ])
