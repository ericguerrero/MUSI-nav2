"""Unit tests for the groundtruth pose node's pure logic.

Runs inside the nav2_student:humble container, where gazebo_msgs exists.
See tests/run_unit_tests.sh.
"""

import sys

sys.path.insert(0, "/ros2_ws/src")

from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import Pose, Twist

from groundtruth_pose import extract_model_state, find_model_index, should_publish


def _model_states(names):
    """Build a ModelStates carrying one distinguishable pose per name."""
    msg = ModelStates()
    msg.name = list(names)
    for i, _ in enumerate(names):
        pose = Pose()
        pose.position.x = float(i)
        twist = Twist()
        twist.linear.x = float(i) * 10.0
        msg.pose.append(pose)
        msg.twist.append(twist)
    return msg


class TestFindModelIndex:
    def test_returns_index_of_named_model(self):
        assert find_model_index(["ground_plane", "waffle"], "waffle") == 1

    def test_returns_none_when_model_absent(self):
        assert find_model_index(["ground_plane"], "waffle") is None

    def test_returns_none_for_empty_name_list(self):
        assert find_model_index([], "waffle") is None


class TestExtractModelState:
    def test_returns_pose_and_twist_of_named_model(self):
        msg = _model_states(["ground_plane", "waffle"])

        pose, twist = extract_model_state(msg, "waffle")

        assert pose.position.x == 1.0
        assert twist.linear.x == 10.0

    def test_returns_none_when_model_absent(self):
        msg = _model_states(["ground_plane"])

        assert extract_model_state(msg, "waffle") is None


class TestShouldPublish:
    def test_allows_the_first_sample(self):
        assert should_publish(None, 100.0, 50.0) is True

    def test_blocks_before_the_interval_elapses(self):
        # 50 Hz means a 0.02 s interval; 0.01 s is too soon.
        assert should_publish(100.0, 100.01, 50.0) is False

    def test_allows_once_the_interval_elapses(self):
        assert should_publish(100.0, 100.02, 50.0) is True

    def test_treats_a_non_positive_rate_as_unthrottled(self):
        assert should_publish(100.0, 100.0, 0.0) is True
