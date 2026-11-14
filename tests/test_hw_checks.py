"""Unit tests for the pass/fail decisions of the ROSbot 2R hardware check."""

import math
import sys

sys.path.insert(0, "/robot/hw_check")

import pytest

from checks import (
    WHEEL_JOINTS,
    check_depth_image,
    check_imu_stationary,
    check_joint_names,
    check_range_reading,
    check_rate,
    check_scan,
    check_stop,
    check_throttled,
    check_voltage,
    check_wheel_response,
    expected_wheel_velocities,
    nearest_bearing_in_base,
)

NAN = float("nan")
INF = float("inf")


class TestCheckRate:
    def test_passes_at_the_expected_rate(self):
        times_20hz = [i * 0.05 for i in range(200)]
        assert check_rate("j", times_20hz, 20.0, 0.10).passed

    def test_fails_when_the_rate_is_outside_the_tolerance(self):
        assert not check_rate("j", [i / 15 for i in range(150)], 20.0, 0.10).passed

    def test_reports_no_messages_for_an_absent_topic(self):
        result = check_rate("j", [], 20.0, 0.10)
        assert not result.passed
        assert result.detail == "no messages"

    def test_a_single_message_is_not_a_rate(self):
        assert check_rate("j", [1.0], 20.0, 0.10).detail == "no messages"


class TestCheckVoltage:
    def test_passes_on_the_bench_supply_reading(self):
        assert check_voltage([12.15, 12.20]).passed

    def test_fails_when_any_sample_sags_below_the_minimum(self):
        assert not check_voltage([12.1, 10.4]).passed

    def test_fails_without_samples(self):
        assert not check_voltage([]).passed

    def test_fails_on_nan_instead_of_raising(self):
        assert not check_voltage([12.1, NAN]).passed


class TestCheckImuStationary:
    def test_passes_on_the_sample_measured_on_the_robot(self):
        assert check_imu_stationary(
            (-0.0078125, -0.06524658, 0.0, 0.99786377),
            (1.28, -0.12, 9.91),
            (0.0, 0.0011, 0.0022),
        ).passed

    def test_fails_when_gravity_is_missing(self):
        assert not check_imu_stationary((0, 0, 0, 1), (0.0, 0.0, 0.0), (0, 0, 0)).passed

    def test_fails_on_nan_instead_of_raising(self):
        assert not check_imu_stationary((0, 0, 0, 1), (0.0, 0.0, NAN), (0, 0, 0)).passed

    def test_fails_when_the_robot_is_rotating(self):
        assert not check_imu_stationary((0, 0, 0, 1), (0.0, 0.0, 9.81), (0, 0, 0.3)).passed


class TestCheckJointNames:
    def test_accepts_the_four_wheels_in_the_order_the_robot_reports(self):
        names = ["fr_wheel_joint", "rl_wheel_joint", "rr_wheel_joint", "fl_wheel_joint"]
        assert check_joint_names(names).passed

    def test_fails_when_a_wheel_is_missing(self):
        assert not check_joint_names(["fr_wheel_joint", "rl_wheel_joint"]).passed


class TestCheckRangeReading:
    def test_passes_on_a_reading_inside_the_sensor_range(self):
        assert check_range_reading("fl", 0.277, 0.01, 0.90).passed

    def test_infinity_means_nothing_in_front_and_still_passes(self):
        result = check_range_reading("fl", INF, 0.01, 0.90)
        assert result.passed
        assert result.detail == "no target in range"

    def test_fails_on_nan(self):
        assert not check_range_reading("fl", NAN, 0.01, 0.90).passed

    def test_fails_on_a_negative_distance(self):
        assert not check_range_reading("fl", -1.0, 0.01, 0.90).passed


class TestCheckScan:
    def test_passes_with_enough_valid_returns_over_a_full_turn(self):
        ranges = [1.0] * 500 + [INF] * 220
        assert check_scan(ranges, 0.15, 12.0, -3.14159, 3.14159).passed

    def test_fails_when_no_return_is_valid(self):
        assert not check_scan([INF] * 720, 0.15, 12.0, -3.14159, 3.14159).passed

    def test_fails_when_the_scan_covers_half_a_turn(self):
        assert not check_scan([1.0] * 720, 0.15, 12.0, 0.0, 3.14159).passed

    def test_fails_on_an_empty_scan(self):
        assert not check_scan([], 0.15, 12.0, -3.14159, 3.14159).passed


class TestCheckThrottled:
    def test_past_under_voltage_alone_passes(self):
        assert check_throttled(0x50000).passed

    def test_a_fault_present_now_fails(self):
        assert not check_throttled(0x50005).passed


class TestExpectedWheelVelocities:
    def test_straight_motion_turns_all_wheels_alike(self):
        forward = expected_wheel_velocities(0.10, 0.0)
        for joint in WHEEL_JOINTS:
            assert forward[joint] == pytest.approx(2.3529, abs=1e-3)

    def test_turning_left_drives_the_sides_in_opposite_directions(self):
        turn = expected_wheel_velocities(0.0, 0.5)
        assert turn["fr_wheel_joint"] == pytest.approx(1.5865, abs=1e-3)
        assert turn["rr_wheel_joint"] == pytest.approx(1.5865, abs=1e-3)
        assert turn["fl_wheel_joint"] == pytest.approx(-1.5865, abs=1e-3)
        assert turn["rl_wheel_joint"] == pytest.approx(-1.5865, abs=1e-3)


class TestCheckWheelResponse:
    forward = expected_wheel_velocities(0.10, 0.0)

    def test_passes_when_every_wheel_is_within_tolerance(self):
        measured = {j: 2.2 for j in WHEEL_JOINTS}
        assert check_wheel_response("forward", measured, self.forward).passed

    def test_names_the_wheel_that_does_not_turn(self):
        stalled = {**{j: 2.3 for j in WHEEL_JOINTS}, "rr_wheel_joint": 0.0}
        result = check_wheel_response("forward", stalled, self.forward)
        assert not result.passed
        assert "rr_wheel_joint" in result.detail
        assert "fl_wheel_joint" not in result.detail

    def test_fails_when_a_wheel_turns_the_wrong_way(self):
        reversed_one = {**{j: 2.3 for j in WHEEL_JOINTS}, "fl_wheel_joint": -2.3}
        assert not check_wheel_response("forward", reversed_one, self.forward).passed

    def test_fails_on_nan_instead_of_raising(self):
        with_nan = {**{j: 2.3 for j in WHEEL_JOINTS}, "rl_wheel_joint": NAN}
        assert not check_wheel_response("forward", with_nan, self.forward).passed

    def test_fails_when_a_wheel_was_never_measured(self):
        missing = {j: 2.3 for j in WHEEL_JOINTS if j != "fr_wheel_joint"}
        result = check_wheel_response("forward", missing, self.forward)
        assert not result.passed
        assert "fr_wheel_joint" in result.detail


class TestCheckStop:
    def test_passes_when_every_wheel_is_at_rest(self):
        assert check_stop({j: 0.02 for j in WHEEL_JOINTS}).passed

    def test_fails_when_a_wheel_keeps_turning(self):
        moving = {**{j: 0.0 for j in WHEEL_JOINTS}, "fl_wheel_joint": 0.6}
        assert not check_stop(moving).passed

    def test_fails_without_measurements(self):
        assert not check_stop({}).passed


class TestNearestBearingInBase:
    def test_a_laser_mounted_backwards_reports_a_front_object_at_zero(self):
        ranges = [5.0] * 360
        ranges[180] = 0.5
        bearing = nearest_bearing_in_base(ranges, 0.0, math.radians(1), math.pi)
        assert abs(bearing) == pytest.approx(0.0, abs=0.02)

    def test_returns_none_when_nothing_is_close(self):
        assert nearest_bearing_in_base([5.0] * 360, 0.0, math.radians(1), 0.0) is None

    def test_ignores_invalid_returns(self):
        ranges = [NAN, INF, 0.0, 0.6] + [5.0] * 356
        bearing = nearest_bearing_in_base(ranges, 0.0, math.radians(1), 0.0)
        assert bearing == pytest.approx(math.radians(3))


class TestCheckDepthImage:
    def test_passes_on_a_full_resolution_frame_with_valid_depth(self):
        assert check_depth_image(0.6, 640, 480).passed

    def test_fails_when_almost_no_pixel_has_depth(self):
        assert not check_depth_image(0.05, 640, 480).passed

    def test_fails_on_an_unexpected_resolution(self):
        assert not check_depth_image(0.6, 320, 240).passed
