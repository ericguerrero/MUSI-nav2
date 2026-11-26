#!/usr/bin/env python3
"""
MUSI-nav2 Ackermann (car-like) robot simulation.

Starts Gazebo Fortress with a small car: rear wheels driven, front wheels
steered, driven through ros2_control by ackermann_steering_controller. It cannot rotate on the spot and cannot turn
tighter than a 0.50 m radius.

Usage:
    ros2 launch /ros2_ws/src/ackermann_simulation.launch.py

Optional arguments:
    world:=<name|path>        empty (default, a flat floor and nothing else), or
                              a world of rosbot_simulation.launch.py:
                              husarion_world, husarion_office,
                              empty_with_plugins, sonoma_raceway, or an
                              absolute path to an .sdf file
    headless:=true            no Gazebo window, saves CPU
    x:=0.0 y:=0.0 yaw:=0.0    spawn pose

Topics: /cmd_vel in; /ackermann_steering_controller/odometry (wheel odometry),
/groundtruth/odom (Gazebo's own pose), /joint_states, /tf out.
"""

import importlib.util
import os

import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    AppendEnvironmentVariable,
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

ROBOT_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "robots", "ackermann")
XACRO_FILE = os.path.join(ROBOT_DIR, "ackermann.urdf.xacro")
CONTROLLERS_FILE = os.path.join(ROBOT_DIR, "ackermann_controllers.yaml")
ROSBOT_LAUNCH_FILE = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                  "rosbot_simulation.launch.py")
# Gazebo's own flat world. It needs nothing from the ROSbot packages.
EMPTY_WORLD = "empty"
EMPTY_WORLD_FILE = "empty.sdf"
MODEL_NAME = "ackermann"
CONTROLLERS = ("joint_state_broadcaster", "ackermann_steering_controller")
BRIDGED_TOPICS = (
    "/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock",
    "/groundtruth/odom@nav_msgs/msg/Odometry[ignition.msgs.Odometry",
)


class InvalidFlagError(ValueError):
    """A boolean launch argument was neither true nor false."""


def parse_flag(value):
    """Return the boolean for 'true' or 'false', in any letter case."""
    lowered = value.strip().lower()
    if lowered not in ("true", "false"):
        raise InvalidFlagError(f"Expected true or false, got '{value}'")
    return lowered == "true"


def rosbot_launch_module():
    """rosbot_simulation.launch.py as a module, so both robots resolve world
    names with the same code. The file name holds dots, so it is loaded by path."""
    spec = importlib.util.spec_from_file_location("rosbot_simulation_launch", ROSBOT_LAUNCH_FILE)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def resolve_world(world, worlds_dir):
    """The file Gazebo loads: its own empty world, or a ROSbot world by name or path."""
    if world == EMPTY_WORLD:
        return EMPTY_WORLD_FILE
    return rosbot_launch_module().resolve_world(world, worlds_dir)


def gazebo_args(headless, world_file=EMPTY_WORLD_FILE):
    """Arguments for Gazebo: running from the start, server only when headless."""
    return f"-r -v 1 {'-s ' if headless else ''}{world_file}"


def world_actions(world):
    """(world file, actions needed before Gazebo starts) for a world argument."""
    if world == EMPTY_WORLD:
        return EMPTY_WORLD_FILE, []
    worlds_share = get_package_share_directory("husarion_gz_worlds")
    world_file = resolve_world(world, os.path.join(worlds_share, "worlds"))
    # The Husarion worlds refer to their models as model://<name>.
    models = AppendEnvironmentVariable("IGN_GAZEBO_RESOURCE_PATH",
                                       os.path.join(worlds_share, "models"))
    return world_file, [models]


def robot_description():
    """URDF text with the controller file path the Gazebo plugin loads."""
    return xacro.process_file(
        XACRO_FILE, mappings={"controllers_file": CONTROLLERS_FILE}
    ).toxml()


def start_simulation(context):
    """Resolve arguments at launch time and start Gazebo, the robot and its controllers."""

    def arg(name):
        return LaunchConfiguration(name).perform(context)

    sim_time = {"use_sim_time": True}
    # ros_gz_sim's launch file puts the ROS library directory on Gazebo's plugin
    # path; started directly, Gazebo does not find ign_ros2_control.
    gz_sim_launch = os.path.join(
        get_package_share_directory("ros_gz_sim"), "launch", "gz_sim.launch.py"
    )
    world_file, actions = world_actions(arg("world"))
    actions += [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(gz_sim_launch),
            launch_arguments={
                "gz_args": gazebo_args(parse_flag(arg("headless")), world_file),
                "on_exit_shutdown": "true",
            }.items(),
        ),
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            parameters=[{"robot_description": robot_description()}, sim_time],
        ),
        Node(
            package="ros_gz_sim",
            executable="create",
            arguments=["-name", MODEL_NAME, "-topic", "robot_description",
                       "-x", arg("x"), "-y", arg("y"), "-Y", arg("yaw")],
        ),
        Node(
            package="ros_gz_bridge",
            executable="parameter_bridge",
            arguments=list(BRIDGED_TOPICS),
            parameters=[sim_time],
        ),
    ]
    actions += [
        Node(package="controller_manager", executable="spawner", arguments=[name],
             parameters=[sim_time])
        for name in CONTROLLERS
    ]
    return actions


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("world", default_value=EMPTY_WORLD,
                                  description="empty, a world name of "
                                              "rosbot_simulation.launch.py, or an absolute .sdf path"),
            DeclareLaunchArgument("headless", default_value="false",
                                  description="Run Gazebo without its window"),
            DeclareLaunchArgument("x", default_value="0.0", description="Spawn x in metres"),
            DeclareLaunchArgument("y", default_value="0.0", description="Spawn y in metres"),
            DeclareLaunchArgument("yaw", default_value="0.0", description="Spawn yaw in radians"),
            OpaqueFunction(function=start_simulation),
        ]
    )
