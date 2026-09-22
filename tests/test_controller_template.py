"""Unit tests for the control template's pure logic.

Runs inside the container via tests/run_unit_tests.sh.
"""

import math
import sys

sys.path.insert(0, "/ros2_ws/src")

import pytest

from controller_template import (
    compute_command,
    normalize_angle,
    pose_error,
    reference_at,
)


class TestNormalizeAngle:
    def test_leaves_an_angle_already_in_range(self):
        assert normalize_angle(1.0) == pytest.approx(1.0)

    def test_wraps_just_above_pi_to_just_above_minus_pi(self):
        assert normalize_angle(math.pi + 0.1) == pytest.approx(-math.pi + 0.1)

    def test_wraps_a_full_turn_to_zero(self):
        assert normalize_angle(2 * math.pi) == pytest.approx(0.0, abs=1e-9)

    def test_wraps_large_negative_angles(self):
        # -3pi lands exactly on the boundary, where +pi and -pi are both correct
        # and denote the same heading. Assert the real requirement instead of
        # picking one: in range, and unchanged modulo a full turn.
        wrapped = normalize_angle(-3 * math.pi)
        assert -math.pi <= wrapped <= math.pi
        assert math.cos(wrapped) == pytest.approx(math.cos(-3 * math.pi), abs=1e-9)
        assert math.sin(wrapped) == pytest.approx(math.sin(-3 * math.pi), abs=1e-9)


class TestReferenceAt:
    def test_point_reference_is_constant_over_time(self):
        params = {"x": 1.5, "y": -2.0, "yaw": 0.5}
        assert reference_at(0.0, "point", params) == (1.5, -2.0, 0.5)
        assert reference_at(99.0, "point", params) == (1.5, -2.0, 0.5)

    def test_circle_starts_at_radius_on_the_x_axis(self):
        x, y, _ = reference_at(0.0, "circle", {"radius": 2.0, "speed": 1.0})
        assert x == pytest.approx(2.0)
        assert y == pytest.approx(0.0, abs=1e-9)

    def test_circle_holds_its_radius_as_time_advances(self):
        params = {"radius": 2.0, "speed": 1.0}
        for t in (0.0, 0.7, 1.9, 4.2):
            x, y, _ = reference_at(t, "circle", params)
            assert math.hypot(x, y) == pytest.approx(2.0)

    def test_figure8_returns_to_the_origin_at_the_half_period(self):
        # The lemniscate crosses its own centre twice per lap.
        x, y, _ = reference_at(math.pi, "figure8", {"size": 2.0, "speed": 1.0})
        assert x == pytest.approx(0.0, abs=1e-9)
        assert y == pytest.approx(0.0, abs=1e-9)

    def test_waypoints_holds_the_first_point_before_its_dwell_elapses(self):
        params = {"points": [(0.0, 0.0), (1.0, 1.0)], "dwell": 5.0}
        x, y, _ = reference_at(1.0, "waypoints", params)
        assert (x, y) == (0.0, 0.0)

    def test_waypoints_advances_once_the_dwell_elapses(self):
        params = {"points": [(0.0, 0.0), (1.0, 1.0)], "dwell": 5.0}
        x, y, _ = reference_at(6.0, "waypoints", params)
        assert (x, y) == (1.0, 1.0)

    def test_waypoints_holds_the_last_point_forever(self):
        params = {"points": [(0.0, 0.0), (1.0, 1.0)], "dwell": 5.0}
        x, y, _ = reference_at(500.0, "waypoints", params)
        assert (x, y) == (1.0, 1.0)

    def test_rejects_an_unknown_mode(self):
        with pytest.raises(ValueError):
            reference_at(0.0, "spiral", {})


class TestPoseError:
    def test_reports_distance_to_the_target(self):
        distance, _ = pose_error((0.0, 0.0, 0.0), (3.0, 4.0, 0.0))
        assert distance == pytest.approx(5.0)

    def test_reports_zero_heading_error_when_facing_the_target(self):
        _, heading = pose_error((0.0, 0.0, 0.0), (1.0, 0.0, 0.0))
        assert heading == pytest.approx(0.0)

    def test_reports_a_left_turn_as_positive_heading_error(self):
        _, heading = pose_error((0.0, 0.0, 0.0), (0.0, 1.0, 0.0))
        assert heading == pytest.approx(math.pi / 2)

    def test_takes_the_short_way_round_rather_than_the_long_way(self):
        # Target is behind and slightly right; the turn must not exceed pi.
        _, heading = pose_error((0.0, 0.0, 3.0), (-1.0, -0.01, 0.0))
        assert abs(heading) <= math.pi


class TestComputeCommand:
    def test_drives_forward_when_the_target_is_ahead(self):
        linear, angular = compute_command((0.0, 0.0, 0.0), (0.0, 0.0), (2.0, 0.0, 0.0))
        assert linear > 0.0
        assert angular == pytest.approx(0.0, abs=1e-6)

    def test_turns_toward_a_target_off_to_one_side(self):
        _, angular = compute_command((0.0, 0.0, 0.0), (0.0, 0.0), (0.0, 2.0, 0.0))
        assert angular > 0.0

    def test_stops_once_inside_the_goal_tolerance(self):
        linear, angular = compute_command((0.0, 0.0, 0.0), (0.0, 0.0), (0.001, 0.0, 0.0))
        assert linear == pytest.approx(0.0)
        assert angular == pytest.approx(0.0)

    def test_never_exceeds_the_velocity_limits(self):
        linear, angular = compute_command((0.0, 0.0, 0.0), (0.0, 0.0), (500.0, 500.0, 0.0))
        assert 0.0 <= linear <= 0.26
        assert abs(angular) <= 1.82
