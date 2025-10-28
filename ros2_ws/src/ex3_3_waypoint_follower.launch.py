#!/usr/bin/env python3
"""
Exercise 3 - Step 5: Run Waypoint Follower with Drift Tracking

Runs the custom waypoint follower script that navigates a square path
and tracks drift between odometry and map frames.

Run this AFTER Nav2 is active and initial pose is set.

Usage:
    ros2 launch ex3_5_waypoint_follower.launch.py

Optional arguments:
    delay:=5.0  (wait N seconds before starting, default: 5.0)

Expected: Robot navigates 4 waypoints, displays drift analysis at end
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction, ExecuteProcess
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    # Launch arguments
    declare_delay = DeclareLaunchArgument(
        'delay',
        default_value='5.0',
        description='Seconds to wait before starting waypoint follower (for Nav2 initialization)'
    )

    delay = LaunchConfiguration('delay')

    # Waypoint follower script
    waypoint_follower = ExecuteProcess(
        cmd=['python3', '/ros2_ws/src/nav2_waypoint_follower.py'],
        output='screen',
        shell=False
    )

    # Delayed start to ensure Nav2 is ready
    delayed_waypoint_follower = TimerAction(
        period=delay,
        actions=[waypoint_follower]
    )

    return LaunchDescription([
        declare_delay,
        delayed_waypoint_follower,
    ])
