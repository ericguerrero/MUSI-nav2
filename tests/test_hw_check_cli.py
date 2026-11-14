"""Unit tests for the command line and reporting of the hardware check runner."""

import sys

sys.path.insert(0, "/robot/hw_check")

import pytest

from checks import CheckResult
from hw_check import parse_args, summarise


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
