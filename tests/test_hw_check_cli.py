"""Unit tests for the command line and reporting of the hardware check runner."""

import sys

sys.path.insert(0, "/robot/hw_check")

import pytest

from checks import MAX_ANGULAR, MAX_LINEAR, CheckResult
from hw_check import (
    MOTOR_SEQUENCE,
    PAUSE_SECONDS,
    STOP_WAIT_SECONDS,
    mean_wheel_velocities,
    parse_args,
    require_wheels_off_ground,
    summarise,
)


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
