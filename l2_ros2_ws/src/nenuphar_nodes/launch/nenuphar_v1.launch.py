"""Launch file for all 11 NENUPHARES BONNEVAL V1 ROS 2 Jazzy nodes."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description() -> LaunchDescription:
    """Launch the 11-node L2 software stack."""
    enable_stub_arg = DeclareLaunchArgument(
        'enable_stub_stream',
        default_value='false',
        description='Enable 15 Hz stub depth/stereo + IMU streams on camera_driver.',
    )
    enable_stub_param = ParameterValue(
        LaunchConfiguration('enable_stub_stream'), value_type=bool
    )

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
    nodes = [
        Node(
            package='nenuphar_nodes',
            executable=exe,
            name=exe,
            output='screen',
            parameters=(
                [{'enable_stub_stream': enable_stub_param}]
                if exe == 'camera_driver'
                else []
            ),
        )
        for exe in executables
    ]
    return LaunchDescription([enable_stub_arg, *nodes])
