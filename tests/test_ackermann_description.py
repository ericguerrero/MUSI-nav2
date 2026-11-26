"""Unit tests for the Ackermann robot description and its launch helpers.

The geometry is written twice, in the xacro (what Gazebo simulates) and in the
controller YAML (what the controller assumes). When they differ the robot still
drives, with wrong odometry, so the agreement is checked here.

Runs inside the nav2_student:humble container. See tests/run_unit_tests.sh.
"""

import importlib.util
import math
import xml.etree.ElementTree as ET

import pytest
import yaml

LAUNCH_FILE = "/ros2_ws/src/ackermann_simulation.launch.py"
MIN_TURN_RADIUS_M = 0.50

# The file name holds dots, so it cannot be imported by module name.
_spec = importlib.util.spec_from_file_location("ackermann_simulation_launch", LAUNCH_FILE)
assert _spec is not None and _spec.loader is not None
launch = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(launch)


@pytest.fixture(scope="module")
def urdf():
    return ET.fromstring(launch.robot_description())


@pytest.fixture(scope="module")
def controller():
    with open(launch.CONTROLLERS_FILE) as f:
        return yaml.safe_load(f)["ackermann_steering_controller"]["ros__parameters"]


def joint(urdf, name):
    found = urdf.find(f"joint[@name='{name}']")
    assert found is not None, f"joint {name} missing from the URDF"
    return found


def origin(urdf, name):
    return [float(v) for v in joint(urdf, name).find("origin").get("xyz").split()]


def limits(urdf, name):
    limit = joint(urdf, name).find("limit")
    return float(limit.get("lower")), float(limit.get("upper"))


def test_controller_wheelbase_and_track_match_the_urdf(urdf, controller):
    front_left = origin(urdf, "front_left_steering_joint")
    front_right = origin(urdf, "front_right_steering_joint")
    rear_left = origin(urdf, "rear_left_wheel_joint")
    rear_right = origin(urdf, "rear_right_wheel_joint")
    assert controller["wheelbase"] == pytest.approx(front_left[0] - rear_left[0])
    assert controller["front_wheel_track"] == pytest.approx(front_left[1] - front_right[1])
    assert controller["rear_wheel_track"] == pytest.approx(rear_left[1] - rear_right[1])


def test_controller_wheel_radius_matches_the_urdf(urdf, controller):
    radius = float(urdf.find("link[@name='rear_left_wheel_link']/collision/geometry/cylinder")
                   .get("radius"))
    assert controller["rear_wheels_radius"] == pytest.approx(radius)
    assert controller["front_wheels_radius"] == pytest.approx(radius)


def test_controller_joints_exist_right_wheel_first(urdf, controller):
    names = controller["rear_wheels_names"] + controller["front_wheels_names"]
    for name in names:
        joint(urdf, name)
    assert ["right" in n for n in names] == [True, False, True, False]


def test_rear_axle_is_under_base_link(urdf):
    assert origin(urdf, "rear_left_wheel_joint")[0] == 0.0


def test_full_lock_points_both_front_wheels_at_the_minimum_radius(urdf, controller):
    wheelbase = controller["wheelbase"]
    half_track = controller["front_wheel_track"] / 2
    inner = math.atan(wheelbase / (MIN_TURN_RADIUS_M - half_track))
    outer = math.atan(wheelbase / (MIN_TURN_RADIUS_M + half_track))
    assert limits(urdf, "front_left_steering_joint") == pytest.approx((-outer, inner))
    assert limits(urdf, "front_right_steering_joint") == pytest.approx((-inner, outer))


def test_gazebo_plugin_loads_the_controller_file_and_maps_cmd_vel(urdf):
    plugin = urdf.find("gazebo/plugin[@filename='ign_ros2_control-system']")
    assert plugin.find("parameters").text == launch.CONTROLLERS_FILE
    assert plugin.find("ros/remapping").text == (
        "ackermann_steering_controller/reference_unstamped:=cmd_vel"
    )


def test_controller_takes_plain_twist(controller):
    assert controller["use_stamped_vel"] is False


@pytest.mark.parametrize("value, expected", [("true", True), ("False", False), (" TRUE ", True)])
def test_flag_is_parsed_in_any_case(value, expected):
    assert launch.parse_flag(value) is expected


def test_flag_other_than_true_or_false_is_rejected():
    with pytest.raises(launch.InvalidFlagError):
        launch.parse_flag("yes")


def test_headless_runs_the_server_only():
    assert "-s" in launch.gazebo_args(True).split()
    assert "-s" not in launch.gazebo_args(False).split()
    assert "-r" in launch.gazebo_args(False).split()
