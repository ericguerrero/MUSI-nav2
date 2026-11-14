#!/usr/bin/env python3
"""
MUSI-nav2 ROSbot 2R simulation.

Starts Gazebo Fortress with the Husarion ROSbot 2R: drive controller, EKF,
LiDAR and IMU, the same stack and topics as the lab robot. This file only
chooses the world and translates arguments; the robot itself comes from
rosbot_gazebo, pinned to the commit the robot's driver was built from.

Usage:
    ros2 launch /ros2_ws/src/rosbot_simulation.launch.py

Optional arguments:
    world:=<name|path>   husarion_world (default), husarion_office,
                         empty_with_plugins, sonoma_raceway, or an absolute
                         path to an .sdf file
    headless:=true       no Gazebo window, saves CPU
    x:=0.0 y:=-2.0 yaw:=0.0   spawn pose
    mecanum:=true        mecanum wheels and mecanum_drive_controller

The simulated camera is off. Set ROSBOT_CAMERA=true before launching to add it.

Topics: /cmd_vel in; /odometry/filtered, /scan, /imu_broadcaster/imu,
/joint_states, /tf out.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

DEFAULT_WORLD = "husarion_world"


class WorldNotFoundError(ValueError):
    """The requested world is neither a known name nor an existing .sdf file."""


class InvalidFlagError(ValueError):
    """A boolean launch argument was neither true nor false."""


def resolve_world(world, worlds_dir):
    """Return the .sdf path for a world name in worlds_dir, or an absolute path."""
    if os.path.isabs(world):
        if os.path.isfile(world):
            return world
        raise WorldNotFoundError(f"World file not found: {world}")
    path = os.path.join(worlds_dir, f"{world}.sdf")
    if os.path.isfile(path):
        return path
    available = sorted(f[: -len(".sdf")] for f in os.listdir(worlds_dir) if f.endswith(".sdf"))
    raise WorldNotFoundError(
        f"Unknown world '{world}'. Available: {', '.join(available)}, or an absolute .sdf path"
    )


def gz_flag(value):
    """Husarion's launch files accept only 'True' and 'False' for booleans."""
    lowered = value.strip().lower()
    if lowered not in ("true", "false"):
        raise InvalidFlagError(f"Expected true or false, got '{value}'")
    return "True" if lowered == "true" else "False"


def include_rosbot_simulation(context):
    """Resolve arguments at launch time and include rosbot_gazebo's launch file."""
    worlds_dir = os.path.join(get_package_share_directory("husarion_gz_worlds"), "worlds")

    def arg(name):
        return LaunchConfiguration(name).perform(context)

    simulation_launch = os.path.join(
        get_package_share_directory("rosbot_gazebo"), "launch", "simulation.launch.py"
    )
    return [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(simulation_launch),
            launch_arguments={
                "gz_world": resolve_world(arg("world"), worlds_dir),
                "gz_headless_mode": gz_flag(arg("headless")),
                "mecanum": gz_flag(arg("mecanum")),
                "x": arg("x"),
                "y": arg("y"),
                "yaw": arg("yaw"),
            }.items(),
        )
    ]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("world", default_value=DEFAULT_WORLD,
                                  description="World name in husarion_gz_worlds, or an absolute .sdf path"),
            DeclareLaunchArgument("headless", default_value="false",
                                  description="Run Gazebo without its window"),
            DeclareLaunchArgument("mecanum", default_value="false",
                                  description="Use mecanum wheels and mecanum_drive_controller"),
            DeclareLaunchArgument("x", default_value="0.0", description="Spawn x in metres"),
            DeclareLaunchArgument("y", default_value="-2.0", description="Spawn y in metres"),
            DeclareLaunchArgument("yaw", default_value="0.0", description="Spawn yaw in radians"),
            OpaqueFunction(function=include_rosbot_simulation),
        ]
    )
