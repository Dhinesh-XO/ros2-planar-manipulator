"""Start the controller and optional GUI with explicit runtime configuration."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('record_telemetry', default_value='false'),
        DeclareLaunchArgument('telemetry_directory', default_value='artifacts/telemetry'),
        DeclareLaunchArgument('control_mode', default_value='position'),
        DeclareLaunchArgument('publish_rate_hz', default_value='50.0'),
        DeclareLaunchArgument('trajectory_duration', default_value='4.0'),
        Node(
            package="planar_arm_control",
            executable="controller_node",
            name="controller_node",
            output="screen",
            parameters=[{
                'control_mode': LaunchConfiguration('control_mode'),
                'publish_rate_hz': ParameterValue(LaunchConfiguration('publish_rate_hz'), value_type=float),
                'trajectory_duration': ParameterValue(LaunchConfiguration('trajectory_duration'), value_type=float),
            }],
        ),
        Node(
            package='planar_arm_control', executable='telemetry_recorder',
            output='screen', condition=IfCondition(LaunchConfiguration('record_telemetry')),
            parameters=[{'output_directory': LaunchConfiguration('telemetry_directory')}],
        ),
        Node(
            package="planar_arm_control",
            executable="gui_node",
            name="gui_node",
            output="screen",
            condition=IfCondition(LaunchConfiguration('gui')),
        ),
    ])
