"""Unit tests for the drive comparison core: presets, the geometry behind the
table, the Gazebo pose parser, wheel odometry and simulation time."""

import math
import sys

sys.path.insert(0, "/ros2_ws/src")

import pytest

from drive_test_core import (
    MANOEUVRES,
    PRESETS,
    DiffDriveOdometry,
    DriveTestError,
    Manoeuvre,
    SimClock,
    describe_command,
    ideal_motion,
    parse_ign_model_pose,
    preset_for,
    relative_motion,
    table_header,
    table_row,
    yaw_from_quaternion,
)

# The slowest robot in the lab; a command above its limits would be clipped
# and the rows of different robots would no longer compare.
WAFFLE_MAX_LINEAR = 0.26
WAFFLE_MAX_ANGULAR = 1.82

IGN_MODEL_OUTPUT = """
Requesting state for world [empty]...

Model: [10]
  - Name: rosbot
  - Pose [ XYZ (m) ] [ RPY (rad) ]:
    [-0.000268 -0.980000 -0.000000]
    [0.000000 -0.000000 1.570796]
"""


def by_name(name):
    return next(m for m in MANOEUVRES if m.name == name)


# --- manoeuvres and presets ---

def test_manoeuvres_run_in_the_order_of_the_worksheet():
    assert [m.name for m in MANOEUVRES] == ["straight", "spin", "arc", "sideways"]


@pytest.mark.parametrize("manoeuvre", MANOEUVRES, ids=lambda m: m.name)
def test_every_command_is_inside_the_slowest_robots_limits(manoeuvre):
    assert math.hypot(manoeuvre.vx, manoeuvre.vy) <= WAFFLE_MAX_LINEAR
    assert abs(manoeuvre.wz) <= WAFFLE_MAX_ANGULAR
    assert manoeuvre.duration > 0


@pytest.mark.parametrize("manoeuvre", MANOEUVRES, ids=lambda m: m.name)
def test_every_turn_is_below_half_a_revolution(manoeuvre):
    # The true heading is read only at the ends; 180 degrees or more would wrap.
    assert abs(ideal_motion(manoeuvre)[2]) < math.radians(150)


@pytest.mark.parametrize("robot", ["turtlebot3", "rosbot", "rosbot_mecanum", "ackermann"])
def test_each_lab_robot_has_a_complete_preset(robot):
    preset = preset_for(robot)
    assert preset["cmd_vel"].startswith("/")
    assert preset["truth"] in ("model_states", "ign_model", "odometry")
    assert preset["model"]
    # With truth on a topic, "model" is that topic.
    assert (preset["truth"] == "odometry") == preset["model"].startswith("/")
    # Exactly one source of wheel odometry.
    assert (preset["odom"] is None) != (preset["wheels"] is None)


def test_unknown_robot_is_refused_and_the_choices_are_named():
    with pytest.raises(DriveTestError, match="ackermann, rosbot, rosbot_mecanum, turtlebot3"):
        preset_for("tank")


def test_changing_a_returned_preset_leaves_the_table_alone():
    preset_for("rosbot")["model"] = "other"
    assert PRESETS["rosbot"]["model"] == "rosbot"


# --- commanded motion ---

def test_straight_command_moves_forward_only():
    assert ideal_motion(by_name("straight")) == pytest.approx((1.0, 0.0, 0.0))


def test_spin_command_turns_without_moving():
    forward, left, turn = ideal_motion(by_name("spin"))
    assert (forward, left) == pytest.approx((0.0, 0.0))
    assert math.degrees(turn) == pytest.approx(114.59, abs=0.01)


def test_sideways_command_moves_left_only():
    assert ideal_motion(by_name("sideways")) == pytest.approx((0.0, 1.0, 0.0))


def test_arc_command_ends_on_a_circle_of_radius_v_over_w():
    arc = Manoeuvre("arc", 0.2, 0.0, 0.5, 4.0)
    radius = 0.2 / 0.5
    forward, left, turn = ideal_motion(arc)
    assert turn == pytest.approx(2.0)
    assert forward == pytest.approx(radius * math.sin(2.0))
    assert left == pytest.approx(radius * (1.0 - math.cos(2.0)))


def test_quarter_circle_to_the_left():
    forward, left, turn = ideal_motion(Manoeuvre("q", 1.0, 0.0, math.pi / 2, 1.0))
    radius = 1.0 / (math.pi / 2)
    assert (forward, left, turn) == pytest.approx((radius, radius, math.pi / 2))


def test_sideways_speed_while_turning_follows_a_circle_too():
    # Moving left while turning left circles a centre behind the robot.
    forward, left, turn = ideal_motion(Manoeuvre("q", 0.0, 1.0, math.pi / 2, 1.0))
    radius = 1.0 / (math.pi / 2)
    assert (forward, left, turn) == pytest.approx((-radius, radius, math.pi / 2))


# --- measured motion ---

def test_motion_is_expressed_in_the_starting_frame():
    # Facing +y, a move along +y is forward and a move along -x is to the left.
    start = (1.0, 1.0, math.pi / 2)
    assert relative_motion(start, (1.0, 3.0, math.pi / 2)) == pytest.approx((2.0, 0.0, 0.0))
    assert relative_motion(start, (0.0, 1.0, math.pi / 2)) == pytest.approx((0.0, 1.0, 0.0))


def test_same_pose_is_no_motion():
    assert relative_motion((3.0, -2.0, 1.0), (3.0, -2.0, 1.0)) == pytest.approx((0.0, 0.0, 0.0))


def test_turn_across_the_wrap_is_the_short_way():
    _, _, turn = relative_motion((0.0, 0.0, math.radians(170)), (0.0, 0.0, math.radians(-170)))
    assert math.degrees(turn) == pytest.approx(20.0)


def test_measured_motion_of_an_ideal_arc_equals_the_commanded_one():
    arc = by_name("arc")
    start = (2.0, -1.0, 0.7)
    radius = arc.vx / arc.wz
    end_yaw = start[2] + arc.wz * arc.duration
    end = (start[0] + radius * (math.sin(end_yaw) - math.sin(start[2])),
           start[1] - radius * (math.cos(end_yaw) - math.cos(start[2])),
           end_yaw)
    assert relative_motion(start, end) == pytest.approx(ideal_motion(arc))


def test_yaw_from_quaternion():
    assert yaw_from_quaternion(0.0, 0.0, 0.0, 1.0) == pytest.approx(0.0)
    half = math.radians(90) / 2
    assert yaw_from_quaternion(0.0, 0.0, math.sin(half), math.cos(half)) == pytest.approx(
        math.pi / 2)


# --- Gazebo Fortress pose ---

def test_pose_is_read_from_ign_model_output():
    assert parse_ign_model_pose(IGN_MODEL_OUTPUT) == pytest.approx(
        (-0.000268, -0.98, 1.570796))


def test_scientific_notation_is_read():
    text = "  - Pose [ XYZ (m) ] [ RPY (rad) ]:\n    [1e-05 -2.5e+00 0]\n    [0 0 -1.5e-01]\n"
    assert parse_ign_model_pose(text) == pytest.approx((1e-05, -2.5, -0.15))


@pytest.mark.parametrize("text", ["", "Requesting state for world [empty]...\n",
                                  "Model: [10]\n  - Name: rosbot\n"])
def test_output_without_a_pose_gives_none(text):
    assert parse_ign_model_pose(text) is None


# --- wheel odometry ---

RADIUS = 0.033
SEPARATION = 0.287


def test_first_reading_sets_the_origin():
    odometry = DiffDriveOdometry(RADIUS, SEPARATION)
    assert odometry.pose is None
    assert odometry.update(12.0, -7.0) == (0.0, 0.0, 0.0)


def test_equal_wheel_angles_drive_straight():
    odometry = DiffDriveOdometry(RADIUS, SEPARATION)
    odometry.update(0.0, 0.0)
    assert odometry.update(10.0, 10.0) == pytest.approx((10.0 * RADIUS, 0.0, 0.0))


def test_opposite_wheel_angles_turn_on_the_spot():
    odometry = DiffDriveOdometry(RADIUS, SEPARATION)
    odometry.update(0.0, 0.0)
    angle = (math.pi / 2) * (SEPARATION / 2) / RADIUS   # wheel angle for 90 degrees
    assert odometry.update(-angle, angle) == pytest.approx((0.0, 0.0, math.pi / 2))


def test_many_small_steps_follow_the_commanded_arc():
    arc = by_name("arc")
    odometry = DiffDriveOdometry(RADIUS, SEPARATION)
    left_rate = (arc.vx - arc.wz * SEPARATION / 2) / RADIUS
    right_rate = (arc.vx + arc.wz * SEPARATION / 2) / RADIUS
    steps = 120                                           # 30 Hz for 4 s
    for step in range(steps + 1):
        t = arc.duration * step / steps
        odometry.update(left_rate * t, right_rate * t)
    assert odometry.pose == pytest.approx(ideal_motion(arc), abs=1e-4)


def test_odometry_cannot_see_the_robot_being_pushed_sideways():
    odometry = DiffDriveOdometry(RADIUS, SEPARATION)
    odometry.update(1.0, 1.0)
    assert odometry.update(1.0, 1.0) == pytest.approx((0.0, 0.0, 0.0))


# --- simulation time ---

def test_no_time_before_the_first_clock_message():
    assert SimClock().now(100.0) is None


def test_time_advances_between_clock_messages():
    clock = SimClock()
    clock.update(sim_seconds=50.0, wall_seconds=100.0)
    assert clock.now(100.04) == pytest.approx(50.04)


def test_a_paused_simulation_does_not_run_on():
    clock = SimClock()
    clock.update(sim_seconds=50.0, wall_seconds=100.0)
    assert clock.now(160.0) == pytest.approx(50.0 + SimClock.MAX_EXTRAPOLATION_S)


def test_the_time_of_the_last_message_is_kept():
    clock = SimClock()
    assert clock.updated_at is None
    clock.update(sim_seconds=50.0, wall_seconds=100.0)
    assert clock.updated_at == 100.0


# --- table ---

def test_command_is_described_with_units():
    assert describe_command(by_name("arc")) == "x 0.2 m/s, z 0.5 rad/s for 4 s"
    assert describe_command(by_name("sideways")) == "y 0.2 m/s for 5 s"
    assert describe_command(Manoeuvre("rest", 0.0, 0.0, 0.0, 2.0)) == "stop for 2 s"


def test_row_holds_commanded_true_and_odometry_in_that_order():
    row = table_row(by_name("straight"), (0.9, 0.01, math.radians(-2.0)), (1.1, 0.0, 0.0))
    cells = [cell.split() for cell in row.split("|")]
    assert cells[0][0] == "straight"
    assert cells[1] == ["1.000", "0.000", "0.0"]
    assert cells[2] == ["0.900", "0.010", "-2.0"]
    assert cells[3] == ["1.100", "0.000", "0.0"]


def test_a_tiny_negative_value_is_not_printed_as_minus_zero():
    row = table_row(by_name("straight"), (-0.0001, -0.0, -0.0001), (0.0, 0.0, 0.0))
    assert "-0.0" not in row


def test_header_columns_line_up_with_a_row():
    header = table_header().splitlines()
    row = table_row(by_name("arc"), (0.0, 0.0, 0.0), (0.0, 0.0, 0.0))
    bars = [i for i, c in enumerate(row) if c == "|"]
    assert len(bars) == 3
    for line in header:
        assert [i for i, c in enumerate(line) if c == "|"] == bars
