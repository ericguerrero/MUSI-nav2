"""Unit tests for the path arithmetic of tests/ackermann_probe.py.

Runs inside the nav2_student:humble container. See tests/run_unit_tests.sh.
"""

import math

import pytest

import ackermann_probe as probe


def circle(radius, angle, steps=200, t0=0.0):
    """Samples of a robot driving an arc of that radius, heading along it."""
    samples = []
    for i in range(steps + 1):
        a = angle * i / steps
        yaw = math.atan2(math.sin(a), math.cos(a))
        samples.append((t0 + i * 0.01, radius * math.sin(a), radius * (1 - math.cos(a)), yaw))
    return samples


def test_yaw_of_a_quarter_turn_quaternion():
    half = math.pi / 4
    assert probe.yaw_from_quaternion(0.0, 0.0, math.sin(half), math.cos(half)) == pytest.approx(
        math.pi / 2
    )


def test_angle_difference_does_not_jump_at_pi():
    assert probe.angle_difference(-3.1, 3.1) == pytest.approx(2 * math.pi - 6.2)


def test_full_circle_measures_its_circumference_and_two_pi():
    distance, rotation = probe.path_metrics(circle(0.5, 2 * math.pi))
    assert distance == pytest.approx(2 * math.pi * 0.5, rel=1e-3)
    assert rotation == pytest.approx(2 * math.pi)


def test_right_turn_has_negative_rotation_and_positive_radius():
    distance, rotation = probe.path_metrics(circle(0.5, -1.0))
    assert rotation == pytest.approx(-1.0)
    assert probe.turn_radius(distance, rotation) == pytest.approx(0.5, rel=1e-3)


def test_straight_path_has_infinite_radius():
    assert probe.turn_radius(2.0, 0.001) == math.inf


def test_radius_ignores_the_samples_before_the_settle_time():
    # Straight while the wheels steer, then a 0.5 m arc.
    straight = [(i * 0.01, i * 0.01, 0.0, 0.0) for i in range(100)]
    arc = [(t, x + 1.0, y, yaw) for t, x, y, yaw in circle(0.5, 2.0, t0=1.0)]
    distance, rotation, radius = probe.summarise(straight + arc, settle_s=1.0)
    assert distance == pytest.approx(0.99 + 0.01 + 1.0, rel=1e-2)
    assert rotation == pytest.approx(2.0)
    assert radius == pytest.approx(0.5, rel=1e-3)


def test_too_few_samples_is_an_error_not_a_zero():
    with pytest.raises(probe.NoDataError):
        probe.summarise([(0.0, 0.0, 0.0, 0.0)], settle_s=1.0)
