"""Unit tests for the tracking-error maths in the evaluation harness."""

import math
import sys

sys.path.insert(0, "/ros2_ws/src")

import pytest

from trajectory_analyzer import (
    cross_track_error,
    interpolate_at,
    max_abs_error,
    point_to_segment_distance,
    rmse,
    settling_time,
)


class TestPointToSegmentDistance:
    def test_measures_perpendicular_distance_to_the_segment(self):
        assert point_to_segment_distance((0.0, 1.0), (0.0, 0.0), (1.0, 0.0)) == \
            pytest.approx(1.0)

    def test_clamps_to_the_near_endpoint_when_the_point_is_past_it(self):
        # Perpendicular foot lies beyond the segment, so the endpoint is nearest.
        assert point_to_segment_distance((5.0, 0.0), (0.0, 0.0), (1.0, 0.0)) == \
            pytest.approx(4.0)

    def test_clamps_at_the_start_endpoint_too(self):
        assert point_to_segment_distance((-3.0, 0.0), (0.0, 0.0), (1.0, 0.0)) == \
            pytest.approx(3.0)

    def test_handles_a_degenerate_zero_length_segment(self):
        # Both endpoints equal: the segment is a point, not a divide-by-zero.
        assert point_to_segment_distance((3.0, 4.0), (0.0, 0.0), (0.0, 0.0)) == \
            pytest.approx(5.0)


class TestCrossTrackError:
    def test_is_zero_for_a_point_on_the_path(self):
        path = [(0.0, 0.0), (1.0, 0.0), (2.0, 0.0)]
        assert cross_track_error((1.0, 0.0), path) == pytest.approx(0.0)

    def test_finds_the_nearest_segment_not_the_first(self):
        path = [(0.0, 0.0), (1.0, 0.0), (2.0, 0.0)]
        assert cross_track_error((2.0, 0.5), path) == pytest.approx(0.5)

    def test_rejects_an_empty_path(self):
        with pytest.raises(ValueError):
            cross_track_error((0.0, 0.0), [])


class TestRmse:
    def test_computes_root_mean_square(self):
        assert rmse([3.0, 4.0]) == pytest.approx(math.sqrt(12.5))

    def test_is_zero_for_perfect_tracking(self):
        assert rmse([0.0, 0.0, 0.0]) == pytest.approx(0.0)

    def test_refuses_an_empty_series_rather_than_returning_zero(self):
        # A silent 0.0 here would read as flawless tracking.
        with pytest.raises(ValueError):
            rmse([])


class TestMaxAbsError:
    def test_reports_the_largest_magnitude_ignoring_sign(self):
        assert max_abs_error([0.1, -0.9, 0.3]) == pytest.approx(0.9)

    def test_refuses_an_empty_series(self):
        with pytest.raises(ValueError):
            max_abs_error([])


class TestSettlingTime:
    def test_reports_when_the_error_last_dropped_below_the_threshold(self):
        times = [0.0, 1.0, 2.0, 3.0]
        errors = [1.0, 0.5, 0.01, 0.01]
        assert settling_time(times, errors, 0.05) == pytest.approx(2.0)

    def test_ignores_an_early_dip_that_does_not_hold(self):
        times = [0.0, 1.0, 2.0, 3.0]
        errors = [0.01, 5.0, 0.01, 0.01]
        assert settling_time(times, errors, 0.05) == pytest.approx(2.0)

    def test_returns_none_when_it_never_settles(self):
        assert settling_time([0.0, 1.0], [9.0, 9.0], 0.05) is None


class TestInterpolateAt:
    def test_returns_an_exact_sample_unchanged(self):
        samples = [(0.0, (0.0, 0.0)), (2.0, (2.0, 4.0))]
        assert interpolate_at(samples, 0.0) == pytest.approx((0.0, 0.0))

    def test_interpolates_linearly_between_samples(self):
        samples = [(0.0, (0.0, 0.0)), (2.0, (2.0, 4.0))]
        assert interpolate_at(samples, 1.0) == pytest.approx((1.0, 2.0))

    def test_clamps_before_the_first_sample(self):
        samples = [(1.0, (5.0, 5.0)), (2.0, (6.0, 6.0))]
        assert interpolate_at(samples, 0.0) == pytest.approx((5.0, 5.0))

    def test_clamps_after_the_last_sample(self):
        samples = [(1.0, (5.0, 5.0)), (2.0, (6.0, 6.0))]
        assert interpolate_at(samples, 99.0) == pytest.approx((6.0, 6.0))

    def test_rejects_an_empty_series(self):
        with pytest.raises(ValueError):
            interpolate_at([], 0.0)
