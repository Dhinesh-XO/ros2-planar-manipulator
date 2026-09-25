"""Same controller and GUI, using bridged physical state and position commands."""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory('planar_arm_control')
    world = os.path.join(share, 'worlds', 'workcell.sdf')
    config = os.path.join(share, 'config', 'gazebo_bridge.yaml')
    return LaunchDescription([
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('gazebo_gui', default_value='false'),
        DeclareLaunchArgument('record_telemetry', default_value='true'),
        DeclareLaunchArgument('telemetry_directory', default_value='artifacts/gazebo_telemetry'),
        SetEnvironmentVariable('GZ_PARTITION', 'kineshia_'+os.environ.get('ROS_DOMAIN_ID', '67')),
        ExecuteProcess(cmd=['gz', 'sim', '-s', '-r', '--force-version', '8', world],
                       output='screen'),
        ExecuteProcess(cmd=['gz', 'sim', '-g', '--force-version', '8'], output='screen',
                       condition=IfCondition(LaunchConfiguration('gazebo_gui'))),
        Node(package='ros_gz_bridge', executable='parameter_bridge', output='screen',
             parameters=[{'config_file': config}]),
        Node(package='planar_arm_control', executable='controller_node', output='screen',
             parameters=[{'backend': 'gazebo', 'trajectory_duration': 3.0}]),
        Node(package='planar_arm_control', executable='gui_node', output='screen',
             condition=IfCondition(LaunchConfiguration('gui'))),
        Node(package='planar_arm_control', executable='telemetry_recorder', output='screen',
             condition=IfCondition(LaunchConfiguration('record_telemetry')),
             parameters=[{'output_directory': LaunchConfiguration('telemetry_directory')}]),
    ])
