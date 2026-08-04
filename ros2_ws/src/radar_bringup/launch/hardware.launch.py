"""Phase 1 hardware pipeline: TI IWR6843ISK-ODS -> tracker -> RViz.

    ros2 launch radar_bringup hardware.launch.py
    ros2 launch radar_bringup hardware.launch.py cli_port:=/dev/ttyACM2 \
        data_port:=/dev/ttyACM3 cfg_file:=/path/to/custom.cfg
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory('radar_bringup')
    params = os.path.join(share, 'config', 'hardware_params.yaml')
    rviz_config = os.path.join(share, 'rviz', 'echo_pigeon.rviz')

    return LaunchDescription([
        DeclareLaunchArgument('rviz', default_value='true',
                              description='Start RViz'),
        DeclareLaunchArgument('cli_port', default_value='/dev/ttyACM0',
                              description='TI CLI/config UART'),
        DeclareLaunchArgument('data_port', default_value='/dev/ttyACM1',
                              description='TI data UART'),
        DeclareLaunchArgument('cfg_file', default_value='',
                              description='Chirp cfg; empty = package default'),

        Node(package='ti_radar_driver', executable='radar_driver_node',
             parameters=[params, {
                 'cli_port': LaunchConfiguration('cli_port'),
                 'data_port': LaunchConfiguration('data_port'),
                 'cfg_file': LaunchConfiguration('cfg_file'),
             }]),
        Node(package='radar_preprocessor', executable='preprocessor_node',
             parameters=[params]),
        Node(package='target_tracker', executable='tracker_node',
             parameters=[params]),
        Node(package='track_visualizer', executable='visualizer_node',
             parameters=[params]),
        Node(package='data_logger', executable='logger_node',
             parameters=[params]),

        Node(package='tf2_ros', executable='static_transform_publisher',
             arguments=['--frame-id', 'map', '--child-frame-id', 'radar_link']),
        Node(package='rviz2', executable='rviz2',
             arguments=['-d', rviz_config],
             condition=IfCondition(LaunchConfiguration('rviz'))),
    ])
