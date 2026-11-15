"""Unit tests for the motion primitive core: parsing, speed profile, and the
closed loop against a simulated robot."""

import math
import sys
from dataclasses import dataclass

sys.path.insert(0, "/ros2_ws/src")

import pytest

from motion_primitives_core import (
    ANGLE_TOLERANCE,
    DISTANCE_TOLERANCE,
    Limits,
    Primitive,
    PrimitiveError,
    PrimitiveRunner,
    expected_duration,
    normalize_angle,
    parse_primitives,
    speed_toward,
)


COAST_SECONDS = 2.0


@dataclass
class Run:
    pose: tuple          # (x, y, yaw); yaw is NOT wrapped, so 270 degrees reads as 270
    runner: PrimitiveRunner
    peak_linear: float   # largest commanded speeds
    peak_angular: float
    elapsed: float
    max_progress: float  # largest distance from the start seen at any step


def simulate(primitive, limits, start=(0.0, 0.0, 0.0), lag=0.2, dt=0.05,
             max_time=60.0, blocked=False):
    """Drive a unicycle whose velocities follow the command with a first-order
    lag, as a real base does. The runner sees a wrapped yaw, as odometry gives."""
    runner = PrimitiveRunner(primitive, limits)
    x, y, yaw = start
    linear = angular = 0.0
    peak_linear = peak_angular = max_progress = elapsed = 0.0
    blend = dt / (lag + dt)
    while elapsed < max_time and not runner.done:
        command_linear, command_angular = runner.update((x, y, normalize_angle(yaw)), dt)
        peak_linear = max(peak_linear, abs(command_linear))
        peak_angular = max(peak_angular, abs(command_angular))
        linear += blend * (command_linear - linear)
        angular += blend * (command_angular - angular)
        if not blocked:
            x += linear * math.cos(yaw) * dt
            y += linear * math.sin(yaw) * dt
            yaw += angular * dt
        max_progress = max(max_progress, math.hypot(x - start[0], y - start[1]))
        elapsed += dt
    # A robot told to stop still coasts. The pose that counts is where it comes
    # to rest, not where it was when the runner said "done".
    for _ in range(int(COAST_SECONDS / dt)):
        linear -= blend * linear
        angular -= blend * angular
        if not blocked:
            x += linear * math.cos(yaw) * dt
            y += linear * math.sin(yaw) * dt
            yaw += angular * dt
    return Run((x, y, yaw), runner, peak_linear, peak_angular, elapsed, max_progress)


class TestParsePrimitives:
    def test_reads_pairs_of_word_and_amount(self):
        assert parse_primitives(["forward", "2.0", "turn", "90"]) == [
            Primitive("forward", 2.0),
            Primitive("turn", pytest.approx(math.pi / 2)),
        ]

    def test_a_negative_turn_is_to_the_right(self):
        assert parse_primitives(["turn", "-45"])[0].amount == pytest.approx(-math.pi / 4)

    @pytest.mark.parametrize("words", [
        [],
        ["forward"],
        ["jump", "1"],
        ["forward", "abc"],
        ["forward", "nan"],
        ["forward", "inf"],
        ["forward", "0"],
        ["forward", "1000"],
        ["turn", "100000"],
    ])
    def test_rejects_input_it_cannot_carry_out(self, words):
        with pytest.raises(PrimitiveError):
            parse_primitives(words)


class TestLimits:
    def test_refuses_a_speed_above_the_robot_limit(self):
        with pytest.raises(PrimitiveError):
            Limits(max_linear=2.0)

    def test_refuses_a_speed_that_is_not_positive(self):
        with pytest.raises(PrimitiveError):
            Limits(max_angular=0.0)


class TestSpeedToward:
    def test_ramps_up_from_rest_by_the_acceleration(self):
        assert speed_toward(2.0, 0.0, 0.2, 0.3, 0.05) == pytest.approx(0.015)

    def test_cruises_at_the_maximum(self):
        assert speed_toward(2.0, 0.2, 0.2, 0.3, 0.05) == pytest.approx(0.2)

    def test_brakes_so_as_to_stop_at_the_target(self):
        assert speed_toward(0.03, 0.2, 0.2, 0.3, 0.05) == pytest.approx(0.1342, abs=1e-3)

    def test_goes_backward_for_a_negative_remainder(self):
        assert speed_toward(-2.0, -0.2, 0.2, 0.3, 0.05) == pytest.approx(-0.2)


def test_expected_duration_is_cruise_time_plus_the_ramp():
    assert expected_duration(Primitive("forward", 2.0), Limits()) == \
        pytest.approx(10.667, abs=1e-3)


class TestForward:
    def test_arrives_straight_inside_the_speed_cap_without_overshoot(self):
        run = simulate(Primitive("forward", 2.0), Limits())
        assert run.runner.done
        assert run.pose[0] == pytest.approx(2.0, abs=DISTANCE_TOLERANCE)
        assert abs(run.pose[1]) < 0.01
        assert run.peak_linear <= 0.2 + 1e-9
        assert run.elapsed < 20.0
        assert run.max_progress <= 2.02

    def test_a_negative_distance_drives_backward(self):
        run = simulate(Primitive("forward", -0.5), Limits())
        assert run.runner.done
        assert run.pose[0] == pytest.approx(-0.5, abs=DISTANCE_TOLERANCE)

    def test_distance_is_along_the_heading_the_robot_started_with(self):
        run = simulate(Primitive("forward", 2.0), Limits(), start=(3.0, -2.0, 2.0))
        assert run.runner.done
        assert math.hypot(run.pose[0] - 3.0, run.pose[1] + 2.0) == \
            pytest.approx(2.0, abs=DISTANCE_TOLERANCE)

    def test_holding_a_heading_at_the_wrap_does_not_flip_the_command(self):
        run = simulate(Primitive("forward", 1.0), Limits(), start=(0.0, 0.0, math.pi))
        assert run.runner.done
        assert run.pose[0] == pytest.approx(-1.0, abs=DISTANCE_TOLERANCE)

    def test_a_blocked_robot_never_reports_done(self):
        run = simulate(Primitive("forward", 1.0), Limits(), max_time=20.0, blocked=True)
        assert not run.runner.done

    def test_once_done_the_command_is_zero(self):
        run = simulate(Primitive("forward", 0.3), Limits())
        assert run.runner.done
        assert run.runner.update(run.pose, 0.05) == (0.0, 0.0)


class TestTurn:
    def test_turns_a_quarter_left_inside_the_rate_cap(self):
        run = simulate(Primitive("turn", math.pi / 2), Limits())
        assert run.runner.done
        assert run.pose[2] == pytest.approx(math.pi / 2, abs=ANGLE_TOLERANCE)
        assert run.peak_angular <= 0.5 + 1e-9

    def test_a_turn_beyond_half_a_circle_goes_the_commanded_way_round(self):
        # The simulated yaw is not wrapped: the short way would read -90 degrees.
        run = simulate(Primitive("turn", math.radians(270)), Limits())
        assert run.runner.done
        assert run.pose[2] == pytest.approx(math.radians(270), abs=ANGLE_TOLERANCE)

    def test_a_turn_to_the_right(self):
        run = simulate(Primitive("turn", -math.pi / 2), Limits())
        assert run.runner.done
        assert run.pose[2] == pytest.approx(-math.pi / 2, abs=ANGLE_TOLERANCE)

    def test_turning_across_the_yaw_wrap(self):
        run = simulate(Primitive("turn", math.radians(20)), Limits(), start=(0.0, 0.0, 3.1))
        assert run.runner.done
        assert run.pose[2] - 3.1 == pytest.approx(math.radians(20), abs=ANGLE_TOLERANCE)

    def test_the_robot_stays_in_place(self):
        run = simulate(Primitive("turn", math.pi), Limits())
        assert run.max_progress < 1e-9
