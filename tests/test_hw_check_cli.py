"""Unit tests for the command line and reporting of the hardware check runner."""

import math
import struct
import sys

sys.path.insert(0, "/robot/hw_check")

import pytest

from checks import MAX_ANGULAR, MAX_LINEAR, CheckResult
from hw_check import (
    MOTOR_SEQUENCE,
    PAUSE_SECONDS,
    STAGE_RUNNERS,
    STAGES,
    STOP_WAIT_SECONDS,
    button_result,
    depth_valid_fraction,
    front_result,
    hand_result,
    led_result,
    mean_wheel_velocities,
    parse_args,
    require_wheels_off_ground,
    summarise,
)

NAN = float("nan")


class TestParseArgs:
    def test_stage_is_required(self):
        with pytest.raises(SystemExit):
            parse_args([])

    def test_an_unknown_stage_is_rejected(self):
        with pytest.raises(SystemExit):
            parse_args(["--stage", "everything"])

    def test_throttled_accepts_hex(self):
        args = parse_args(["--stage", "sensors", "--throttled", "0x50000"])
        assert args.throttled == 0x50000

    def test_report_defaults_to_a_file_named_after_the_stage(self):
        assert parse_args(["--stage", "sensors"]).report == "/out/sensors.json"

    def test_the_sampling_window_defaults_to_ten_seconds(self):
        assert parse_args(["--stage", "sensors"]).window == 10.0

    def test_distance_is_a_stage_of_its_own(self):
        # The lidar-front check needs the robot placed in a clear area, so it
        # is run in a second pass without repeating hands, buttons and LEDs.
        args = parse_args(["--stage", "distance"])
        assert args.stage == "distance"
        assert args.report == "/out/distance.json"

    def test_every_stage_has_a_runner(self):
        assert set(STAGE_RUNNERS) == set(STAGES)


class TestSummarise:
    ok = CheckResult("a", True, "20.00 Hz")
    bad = CheckResult("b", False, "no messages")

    def test_exit_code_is_zero_when_every_check_passes(self):
        assert summarise([self.ok, self.ok])[1] == 0

    def test_one_failure_makes_the_exit_code_one(self):
        table, code = summarise([self.ok, self.bad])
        assert code == 1
        assert "FAIL" in table
        assert "no messages" in table

    def test_an_empty_result_list_is_a_failure(self):
        # A stage that checked nothing has proved nothing.
        assert summarise([])[1] == 1


class TestMotorStageSafety:
    def test_refuses_without_the_wheels_flag(self):
        args = parse_args(["--stage", "motors"])
        with pytest.raises(SystemExit) as refusal:
            require_wheels_off_ground(args)
        assert "--wheels-off-ground" in str(refusal.value)

    def test_accepts_the_wheels_flag(self):
        args = parse_args(["--stage", "motors", "--wheels-off-ground"])
        assert require_wheels_off_ground(args) is None

    def test_sequence_is_bounded_and_inside_the_caps(self):
        # If the SSH link drops, the stage must end by itself within seconds.
        total = sum(seconds + PAUSE_SECONDS for *_, seconds in MOTOR_SEQUENCE)
        assert total + STOP_WAIT_SECONDS <= 30.0
        for _, linear, angular, _ in MOTOR_SEQUENCE:
            assert abs(linear) <= MAX_LINEAR
            assert abs(angular) <= MAX_ANGULAR

    def test_sequence_exercises_both_directions_of_both_motions(self):
        commands = {(linear, angular) for _, linear, angular, _ in MOTOR_SEQUENCE}
        assert commands == {(0.10, 0.0), (-0.10, 0.0), (0.0, 0.5), (0.0, -0.5)}


class TestMeanWheelVelocities:
    def test_averages_only_the_samples_after_the_start_time(self):
        samples = [
            (1.0, {"fl_wheel_joint": 0.0}),
            (2.0, {"fl_wheel_joint": 2.0}),
            (3.0, {"fl_wheel_joint": 3.0}),
        ]
        assert mean_wheel_velocities(samples, since=1.5) == {"fl_wheel_joint": 2.5}

    def test_returns_nothing_when_no_sample_is_recent_enough(self):
        assert mean_wheel_velocities([(1.0, {"fl_wheel_joint": 2.0})], since=5.0) == {}


class TestDepthValidFraction:
    def test_counts_non_zero_16uc1_pixels(self):
        data = struct.pack("<4H", 0, 800, 0, 1200)
        assert depth_valid_fraction(data, "16UC1") == pytest.approx(0.5)

    def test_ignores_nan_in_32fc1(self):
        data = struct.pack("<4f", NAN, 0.0, 1.5, 2.0)
        assert depth_valid_fraction(data, "32FC1") == pytest.approx(0.5)

    def test_an_empty_frame_has_no_valid_pixel(self):
        assert depth_valid_fraction(b"", "16UC1") == 0.0

    def test_rejects_an_encoding_it_cannot_read(self):
        with pytest.raises(ValueError):
            depth_valid_fraction(b"\x00\x00", "rgb8")


class TestLedResult:
    def test_fails_when_the_operator_answers_no(self):
        row = led_result("left", answer="n")
        assert not row.passed
        assert row.name == "led/left"

    def test_passes_when_the_operator_answers_yes(self):
        assert led_result("right", answer=" Y ").passed


class TestHandResult:
    def test_passes_when_the_reading_drops_below_the_hand_distance(self):
        row = hand_result("fl", baseline=0.45, readings=[0.44, 0.30, 0.09])
        assert row.passed
        assert row.name == "hand range/fl"

    def test_open_space_then_a_hand_passes(self):
        # nan is how the firmware reports nothing in range.
        assert hand_result("rl", baseline=NAN, readings=[NAN, NAN, 0.12]).passed

    def test_fails_when_the_reading_never_drops(self):
        assert not hand_result("rl", baseline=NAN, readings=[NAN, NAN, NAN]).passed

    def test_fails_when_something_was_already_that_close(self):
        # Without a far baseline a near reading proves nothing about the hand.
        row = hand_result("fr", baseline=0.10, readings=[0.10, 0.09])
        assert not row.passed
        assert "clear" in row.detail


class TestButtonResult:
    def test_passes_when_a_press_arrives(self):
        assert button_result("left", [False, True]).passed

    def test_fails_when_nothing_arrives(self):
        row = button_result("left", [])
        assert not row.passed
        assert row.name == "button/left"

    def test_fails_but_says_so_when_only_false_arrives(self):
        row = button_result("right", [False, False])
        assert not row.passed
        assert "never true" in row.detail


class TestFrontResult:
    def test_passes_when_the_object_is_seen_ahead(self):
        assert front_result(math.radians(8)).passed

    def test_fails_when_the_object_is_seen_behind(self):
        row = front_result(math.radians(178))
        assert not row.passed
        assert "178" in row.detail

    def test_fails_when_nothing_is_within_a_metre(self):
        assert not front_result(None).passed
