#!/usr/bin/env python3
"""Score how well a controller tracked its reference.

Records /reference/pose against /groundtruth/odom during a run, then reports
cross-track error, heading error, RMSE, maximum error and settling time, and
writes the paired samples to CSV for plotting.

    ros2 run ... trajectory_analyzer.py --ros-args -p output:=/shared/student_work/run1

Derived from the dev branch's trajectory analyzer, which compared odometry
against AMCL to demonstrate localisation drift. Here the question is different:
how far is the robot from where it was told to be.
"""

import csv
import json
import math

# A run this short cannot support a meaningful statistic.
MIN_SAMPLES = 2


def point_to_segment_distance(point, start, end):
    """Shortest distance from a point to a line segment.

    Clamped to the segment, so a point past either end measures to that
    endpoint rather than to the infinite line.
    """
    px, py = point
    ax, ay = start
    bx, by = end
    dx, dy = bx - ax, by - ay
    length_sq = dx * dx + dy * dy

    if length_sq == 0.0:
        # Degenerate segment: both endpoints coincide.
        return math.hypot(px - ax, py - ay)

    t = ((px - ax) * dx + (py - ay) * dy) / length_sq
    t = max(0.0, min(1.0, t))
    return math.hypot(px - (ax + t * dx), py - (ay + t * dy))


def cross_track_error(point, path):
    """Distance from a point to the nearest part of a reference path."""
    if not path:
        raise ValueError("cross_track_error needs a non-empty path")
    if len(path) == 1:
        return math.hypot(point[0] - path[0][0], point[1] - path[0][1])
    return min(point_to_segment_distance(point, path[i], path[i + 1])
               for i in range(len(path) - 1))


def rmse(values):
    """Root mean square of an error series."""
    if not values:
        raise ValueError("rmse needs at least one sample; a run with no data "
                         "is not perfect tracking")
    return math.sqrt(sum(v * v for v in values) / len(values))


def max_abs_error(values):
    """Largest error magnitude, ignoring sign."""
    if not values:
        raise ValueError("max_abs_error needs at least one sample")
    return max(abs(v) for v in values)


def settling_time(times, errors, threshold):
    """Time after which the error stays below threshold for the rest of the run.

    Returns None when it never settles. An early dip that does not hold does
    not count, which is the point: overshoot that recovers late is not settled.
    """
    settled_from = None
    for t, e in zip(times, errors):
        if abs(e) < threshold:
            if settled_from is None:
                settled_from = t
        else:
            settled_from = None
    return settled_from


def interpolate_at(samples, t):
    """Linearly interpolate (x, y) at time t from [(time, (x, y)), ...].

    Reference and groundtruth arrive on different clocks and at different
    rates, so they must be put on a common time base before being compared.
    Clamps outside the sampled range rather than extrapolating.
    """
    if not samples:
        raise ValueError("interpolate_at needs at least one sample")
    if t <= samples[0][0]:
        return samples[0][1]
    if t >= samples[-1][0]:
        return samples[-1][1]

    for i in range(len(samples) - 1):
        t0, p0 = samples[i]
        t1, p1 = samples[i + 1]
        if t0 <= t <= t1:
            span = t1 - t0
            if span == 0.0:
                return p0
            frac = (t - t0) / span
            return (p0[0] + frac * (p1[0] - p0[0]),
                    p0[1] + frac * (p1[1] - p0[1]))
    return samples[-1][1]


def analyse(reference_samples, actual_samples, settle_threshold=0.1):
    """Compare a recorded run against its reference and return metrics.

    Both series are [(time, (x, y)), ...]. Groundtruth drives the time base,
    with the reference interpolated onto it.
    """
    if len(actual_samples) < MIN_SAMPLES or len(reference_samples) < MIN_SAMPLES:
        raise ValueError(
            f"insufficient data: {len(actual_samples)} groundtruth and "
            f"{len(reference_samples)} reference samples, need >= {MIN_SAMPLES} of each")

    reference_path = [p for _, p in reference_samples]
    times, errors, paired = [], [], []

    for t, actual in actual_samples:
        expected = interpolate_at(reference_samples, t)
        error = math.hypot(actual[0] - expected[0], actual[1] - expected[1])
        times.append(t)
        errors.append(error)
        paired.append((t, expected[0], expected[1], actual[0], actual[1], error))

    cross_track = [cross_track_error(a, reference_path) for _, a in actual_samples]

    return {
        "samples": len(actual_samples),
        "duration_s": times[-1] - times[0],
        "position_rmse_m": rmse(errors),
        "position_max_error_m": max_abs_error(errors),
        "cross_track_rmse_m": rmse(cross_track),
        "cross_track_max_m": max_abs_error(cross_track),
        "settling_time_s": settling_time(times, errors, settle_threshold),
        "settle_threshold_m": settle_threshold,
        "note": "reference interpolated onto the groundtruth time base",
    }, paired


def write_csv(path, paired):
    with open(path, "w", newline="") as handle:
        writer = csv.writer(handle)
        writer.writerow(["time_s", "ref_x", "ref_y", "actual_x", "actual_y", "error_m"])
        writer.writerows(paired)


def write_metrics(path, metrics):
    with open(path, "w") as handle:
        json.dump(metrics, handle, indent=2)


import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node

DEFAULT_OUTPUT_PREFIX = "/shared/student_work/run"


class TrajectoryAnalyzer(Node):
    """Record a run, then score it on shutdown."""

    def __init__(self):
        super().__init__("trajectory_analyzer")

        self.declare_parameter("output", DEFAULT_OUTPUT_PREFIX)
        self.declare_parameter("settle_threshold", 0.1)
        self._output = self.get_parameter("output").value
        self._threshold = self.get_parameter("settle_threshold").value

        self._reference = []
        self._actual = []
        self._t0 = None

        self.create_subscription(PoseStamped, "/reference/pose", self._on_reference, 50)
        self.create_subscription(Odometry, "/groundtruth/odom", self._on_actual, 50)

        self.get_logger().info(
            f"Recording; writing {self._output}.csv and {self._output}.json on exit")

    def _elapsed(self, stamp):
        t = stamp.sec + stamp.nanosec * 1e-9
        if self._t0 is None:
            self._t0 = t
        return t - self._t0

    def _on_reference(self, msg):
        self._reference.append(
            (self._elapsed(msg.header.stamp), (msg.pose.position.x, msg.pose.position.y)))

    def _on_actual(self, msg):
        self._actual.append(
            (self._elapsed(msg.header.stamp), (msg.pose.pose.position.x,
                                               msg.pose.pose.position.y)))

    def report(self):
        """Score the run. Says why it cannot rather than inventing a number."""
        try:
            metrics, paired = analyse(self._reference, self._actual, self._threshold)
        except ValueError as exc:
            self.get_logger().error(f"Cannot score this run: {exc}")
            return

        write_csv(f"{self._output}.csv", paired)
        write_metrics(f"{self._output}.json", metrics)

        self.get_logger().info(
            "Tracking results:\n"
            f"  samples            {metrics['samples']}\n"
            f"  duration           {metrics['duration_s']:.1f} s\n"
            f"  position RMSE      {metrics['position_rmse_m']:.4f} m\n"
            f"  position max       {metrics['position_max_error_m']:.4f} m\n"
            f"  cross-track RMSE   {metrics['cross_track_rmse_m']:.4f} m\n"
            f"  cross-track max    {metrics['cross_track_max_m']:.4f} m\n"
            f"  settling time      {metrics['settling_time_s']}\n"
            f"  wrote              {self._output}.csv / .json")


def main(args=None):
    rclpy.init(args=args)
    node = TrajectoryAnalyzer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.report()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
