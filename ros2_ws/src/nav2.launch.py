#!/usr/bin/env python3
"""
Exercise 3 - Step 3: Launch Nav2 Navigation Stack

Starts the full Nav2 stack with map server, AMCL localization, planners, and controllers.
Run this AFTER robot is spawned (ex3_2_spawn_robot.launch.py).

Usage:
    ros2 launch ex3_3_nav2.launch.py

Optional arguments:
    map:=/path/to/map.yaml    (default: landmark_map.yaml)
    params:=/path/to/params   (default: nav2_params_ex3.yaml)

Expected: Nav2 servers start, AMCL initializes, ready for navigation goals
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    # Package directory
    bringup_dir = get_package_share_directory('nav2_bringup')
    launch_dir = os.path.join(bringup_dir, 'launch')

    # Default paths
    default_map = '/shared/maps/landmark_map.yaml'
    default_params = '/shared/configs/nav2_params.yaml'

    # Launch arguments
    declare_map = DeclareLaunchArgument(
        'map',
        default_value=default_map,
        description='Full path to map yaml file to load'
    )

    declare_params = DeclareLaunchArgument(
        'params_file',
        default_value=default_params,
        description='Full path to Nav2 parameter file'
    )

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock'
    )

    declare_autostart = DeclareLaunchArgument(
        'autostart',
        default_value='true',
        description='Automatically startup the nav2 stack'
    )

    # Get configurations
    map_yaml = LaunchConfiguration('map')
    params_file = LaunchConfiguration('params_file')
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')

    # Nav2 bringup (includes map_server, amcl, controller, planner, bt_navigator, etc.)
    nav2_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_dir, 'bringup_launch.py')
        ),
        launch_arguments={
            'map': map_yaml,
            'use_sim_time': use_sim_time,
            'params_file': params_file,
            'autostart': autostart
        }.items()
    )

    return LaunchDescription([
        declare_map,
        declare_params,
        declare_use_sim_time,
        declare_autostart,
        nav2_bringup,
    ])
