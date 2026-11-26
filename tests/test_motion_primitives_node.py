"""Unit tests for the motion primitives node: the parts that decide when to
give up, and the command line. The loop itself is proved in simulation by
tests/run_motion_primitives_sim_test.sh."""

import sys

sys.path.insert(0, "/ros2_ws/src")

import pytest

from motion_primitives import (
    EXIT_USAGE,
    feedback_is_stale,
    main,
    split_arguments,
    timeout_for,
)
from motion_primitives_core import Limits, Primitive


class TestTimeout:
    def test_scales_with_the_primitive(self):
        # 2 m at 0.2 m/s is expected to take 10.67 s; three times that plus 5.
        assert timeout_for(Primitive("forward", 2.0), Limits()) == \
            pytest.approx(37.0, abs=0.01)

    def test_a_short_move_still_gets_time_to_start(self):
        assert timeout_for(Primitive("forward", 0.05), Limits()) > 5.0


class TestFeedbackIsStale:
    def test_no_feedback_yet_counts_as_stale(self):
        assert feedback_is_stale(now=10.0, last_feedback=None)

    def test_recent_feedback_is_not_stale(self):
        assert not feedback_is_stale(now=10.0, last_feedback=9.7)

    def test_feedback_older_than_half_a_second_is_stale(self):
        assert feedback_is_stale(now=10.0, last_feedback=9.4)


class TestCommandLine:
    def test_ros_arguments_are_not_primitives(self):
        argv = ["motion_primitives.py", "forward", "2.0",
                "--ros-args", "-p", "max_linear:=0.1"]
        assert split_arguments(argv) == ["forward", "2.0"]

    def test_a_negative_amount_is_not_taken_for_an_option(self):
        argv = ["motion_primitives.py", "turn", "-45"]
        assert split_arguments(argv) == ["turn", "-45"]

    @pytest.mark.parametrize("words", [["jump", "1"], ["forward"], [], ["forward", "nan"]])
    def test_bad_input_is_a_usage_error_before_ros_starts(self, words, capsys):
        # rclpy.init has not run in this process: reaching ROS would raise.
        assert main(["motion_primitives.py", *words]) == EXIT_USAGE
        assert "usage:" in capsys.readouterr().err
