#!/usr/bin/env python3
"""
Bag Replay Launch File

Replays a ROS2 bag file with robot_state_publisher for SLAM testing.
Publishes robot description so the robot model appears in RViz during replay.

Usage:
    ros2 launch replay_bag.launch.py bag_path:=/shared/bags/my_bag_folder

Arguments:
    bag_path - Full path to the bag directory (required)
    rate - Playback speed multiplier (default: 1.0)
           1.0 = real-time, 0.5 = half speed, 2.0 = double speed

Example:
    ros2 launch replay_bag.launch.py bag_path:=/shared/bags/exercise5_baseline_20251106_172711

Author: MUSI-nav2 Project
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Get URDF file
    bringup_dir = get_package_share_directory('nav2_bringup')
    urdf_file = os.path.join(bringup_dir, 'urdf', 'turtlebot3_waffle.urdf')

    # Read URDF
    with open(urdf_file, 'r') as f:
        robot_description = f.read()

    # Launch arguments
    declare_bag_path = DeclareLaunchArgument(
        'bag_path',
        description='Full path to the bag directory to replay'
    )

    declare_rate = DeclareLaunchArgument(
        'rate',
        default_value='1.0',
        description='Playback speed (1.0 = real-time, 0.5 = half speed)'
    )

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation clock from bag'
    )

    # Get configurations
    bag_path = LaunchConfiguration('bag_path')
    rate = LaunchConfiguration('rate')
    use_sim_time = LaunchConfiguration('use_sim_time')

    # Robot state publisher (for robot description only, not TF)
    # The bag already has TF data, we just need the robot_description for visualization
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_description': robot_description,
            'publish_frequency': 0.0  # Don't publish TF (bag has it)
        }]
    )

    # Bag playback process
    bag_playback = ExecuteProcess(
        cmd=['ros2', 'bag', 'play', bag_path, '--clock', '--rate', rate],
        output='screen'
    )

    return LaunchDescription([
        declare_bag_path,
        declare_rate,
        declare_use_sim_time,
        robot_state_publisher,
        bag_playback,
    ])
