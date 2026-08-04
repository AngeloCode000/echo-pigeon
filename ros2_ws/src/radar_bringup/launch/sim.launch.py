"""Phase 0 simulation pipeline: synthetic drone -> tracker -> RViz.

    ros2 launch radar_bringup sim.launch.py
    ros2 launch radar_bringup sim.launch.py trajectory:=circle rviz:=false
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
    params = os.path.join(share, 'config', 'sim_params.yaml')
    rviz_config = os.path.join(share, 'rviz', 'echo_pigeon.rviz')

    trajectory = LaunchConfiguration('trajectory')
    return LaunchDescription([
        DeclareLaunchArgument('rviz', default_value='true',
                              description='Start RViz'),
        DeclareLaunchArgument('trajectory', default_value='figure_eight',
                              description="'circle' or 'figure_eight'"),

        Node(package='radar_simulator', executable='target_generator_node',
             parameters=[params, {'trajectory_type': trajectory}]),
        Node(package='radar_preprocessor', executable='preprocessor_node',
             parameters=[params]),
        Node(package='target_tracker', executable='tracker_node',
             parameters=[params]),
        Node(package='track_visualizer', executable='visualizer_node',
             parameters=[params]),
        Node(package='data_logger', executable='logger_node',
             parameters=[params]),

        # RViz has no TF tree of its own; pin radar_link to map identity.
        Node(package='tf2_ros', executable='static_transform_publisher',
             arguments=['--frame-id', 'map', '--child-frame-id', 'radar_link']),
        Node(package='rviz2', executable='rviz2',
             arguments=['-d', rviz_config],
             condition=IfCondition(LaunchConfiguration('rviz'))),
    ])
