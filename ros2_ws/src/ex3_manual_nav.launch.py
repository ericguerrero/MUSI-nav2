#!/usr/bin/env python3
"""
Launch file for Manual Nav2 Navigation (for testing waypoint placement)

Launches Gazebo + Nav2 + RViz without automatic waypoint follower.
Use RViz "Nav2 Goal" button to manually test navigation to different points.

Usage:
    ros2 launch ex3_manual_nav.launch.py
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Get package directories
    bringup_dir = get_package_share_directory('nav2_bringup')
    gazebo_dir = get_package_share_directory('turtlebot3_gazebo')
    launch_dir = os.path.join(bringup_dir, 'launch')

    # Map and params
    map_yaml_file = '/shared/maps/empty_10x10.yaml'
    params_file = '/shared/configs/nav2_params_ex3.yaml'

    # Empty world (open environment, no obstacles)
    world = os.path.join(gazebo_dir, 'worlds', 'empty_world.world')

    # URDF for robot_state_publisher
    urdf = os.path.join(bringup_dir, 'urdf', 'turtlebot3_waffle.urdf')
    with open(urdf, 'r') as infp:
        robot_description = infp.read()

    # Launch configuration variables
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    autostart = LaunchConfiguration('autostart', default='true')

    # Robot spawn pose (origin in empty world)
    x_pose = LaunchConfiguration('x_pose', default='0.0')
    y_pose = LaunchConfiguration('y_pose', default='0.0')

    # Declare launch arguments
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock'
    )

    declare_autostart_cmd = DeclareLaunchArgument(
        'autostart',
        default_value='true',
        description='Automatically startup nav2 stack'
    )

    # 1. Start Gazebo server (headless)
    start_gazebo_server_cmd = ExecuteProcess(
        cmd=['gzserver', '-s', 'libgazebo_ros_init.so',
             '-s', 'libgazebo_ros_factory.so', world],
        cwd=[launch_dir],
        output='screen'
    )

    # 2. Robot state publisher
    start_robot_state_publisher_cmd = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_description': robot_description
        }]
    )

    # 3. Spawn TurtleBot3 in Gazebo
    start_gazebo_spawner_cmd = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        output='screen',
        arguments=[
            '-entity', 'turtlebot3_waffle',
            '-file', os.path.join(gazebo_dir, 'models', 'turtlebot3_waffle', 'model.sdf'),
            '-x', x_pose,
            '-y', y_pose,
            '-z', '0.01'
        ]
    )

    # 4. Nav2 bringup (full navigation stack)
    bringup_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_dir, 'bringup_launch.py')
        ),
        launch_arguments={
            'map': map_yaml_file,
            'use_sim_time': use_sim_time,
            'params_file': params_file,
            'autostart': autostart
        }.items()
    )

    # 5. RViz with Nav2 default configuration
    rviz_config_file = os.path.join(bringup_dir, 'rviz', 'nav2_default_view.rviz')
    rviz_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_dir, 'rviz_launch.py')
        ),
        launch_arguments={
            'rviz_config': rviz_config_file
        }.items()
    )

    # Create launch description
    ld = LaunchDescription()

    # Add launch arguments
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_autostart_cmd)

    # Add nodes (order matters for initialization)
    ld.add_action(start_gazebo_server_cmd)
    ld.add_action(start_robot_state_publisher_cmd)
    ld.add_action(start_gazebo_spawner_cmd)
    ld.add_action(bringup_cmd)
    ld.add_action(rviz_cmd)

    return ld
