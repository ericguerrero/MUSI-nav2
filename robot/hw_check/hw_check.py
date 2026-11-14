#!/usr/bin/env python3
"""Bench check of the ROSbot 2R's sensors and actuators.

Runs on the robot, in a throwaway container of the driver image, so that a
failure is a hardware or driver failure and never a network one. Samples the
topics for a window, hands the numbers to the decisions in checks.py, prints
one PASS/FAIL row per item and writes the same rows as JSON.

Rates use this process's monotonic clock. The robot has no real-time clock and
its wall time is wrong without internet, so message stamps cannot be trusted.
"""

import argparse
import json
import sys
import time
from collections import defaultdict
from dataclasses import asdict
from functools import partial

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import BatteryState, Imu, JointState, LaserScan, Range

from checks import (
    MAX_ANGULAR,
    MAX_LINEAR,
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
)

STAGES = ("sensors", "motors", "interactive", "camera")
RANGE_SENSORS = ("fl", "fr", "rl", "rr")

# (topic, message type, expected rate in Hz, relative tolerance), as measured
# on the robot with driver 0.14.0 and firmware 0.11.0.
RATE_CHECKS = (
    ("/joint_states", JointState, 20.0, 0.10),
    ("/imu_broadcaster/imu", Imu, 20.0, 0.10),
    ("/odometry/filtered", Odometry, 20.0, 0.10),
    ("/rosbot_base_controller/odom", Odometry, 20.0, 0.10),
    *((f"/range/{s}", Range, 5.0, 0.20) for s in RANGE_SENSORS),
    ("/scan", LaserScan, 10.0, 0.50),
    ("/scan_filtered", LaserScan, 10.0, 0.50),
)
BATTERY_TOPIC = "/battery"
JOINT_TOPIC = "/joint_states"

# (segment, linear m/s, angular rad/s, seconds). Short and bounded on purpose:
# if the SSH link drops, the stage still ends by itself.
MOTOR_SEQUENCE = (
    ("forward", 0.10, 0.0, 3.0),
    ("backward", -0.10, 0.0, 3.0),
    ("turn_left", 0.0, 0.5, 3.0),
    ("turn_right", 0.0, -0.5, 3.0),
)
PAUSE_SECONDS = 1.0
STOP_WAIT_SECONDS = 1.0
COMMAND_PERIOD = 0.05
# Wheel speed is averaged over the end of a segment, after the ramp-up.
AVERAGING_SECONDS = 2.0


def parse_args(argv: list) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--stage", required=True, choices=STAGES)
    parser.add_argument("--wheels-off-ground", action="store_true",
                        help="required by the motors stage")
    parser.add_argument("--throttled", type=lambda text: int(text, 0), default=0,
                        help="value of vcgencmd get_throttled on the robot")
    parser.add_argument("--report", default=None,
                        help="JSON output path, default /out/<stage>.json")
    parser.add_argument("--window", type=float, default=10.0,
                        help="seconds to sample the topics")
    args = parser.parse_args(argv)
    if args.report is None:
        args.report = f"/out/{args.stage}.json"
    return args


def summarise(results: list) -> tuple:
    """A printable table and the exit code: 0 only when every check passed."""
    width = max((len(r.name) for r in results), default=4)
    rows = [f"{r.name:<{width}}  {'PASS' if r.passed else 'FAIL'}  {r.detail}"
            for r in results]
    failed = sum(1 for r in results if not r.passed)
    rows.append(f"{len(results) - failed} passed, {failed} failed")
    # A stage that checked nothing has proved nothing.
    code = 0 if results and failed == 0 else 1
    return "\n".join(rows), code


class Sampler(Node):
    """Subscribes to topics and remembers when each message arrived."""

    def __init__(self):
        super().__init__("hw_check")
        self._times = defaultdict(list)
        self._last = {}
        self._history = defaultdict(list)
        self._kept = set()

    def watch(self, topic: str, msg_type, keep: bool = False) -> None:
        """Subscribe to a topic; keep=True also stores every message."""
        if keep:
            self._kept.add(topic)
        # Sensor-data QoS is best effort, which matches both the lidar driver's
        # best-effort publisher and the reliable ones.
        self.create_subscription(msg_type, topic, partial(self._on_message, topic),
                                 qos_profile_sensor_data)

    def _on_message(self, topic: str, msg) -> None:
        now = time.monotonic()
        self._times[topic].append(now)
        self._last[topic] = msg
        if topic in self._kept:
            self._history[topic].append((now, msg))

    def collect(self, seconds: float) -> None:
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.02)

    def times(self, topic: str) -> list:
        return self._times[topic]

    def last(self, topic: str):
        return self._last.get(topic)

    def history(self, topic: str) -> list:
        return self._history[topic]


def _vector(v) -> tuple:
    return (v.x, v.y, v.z)


def _content_checks(sampler: Sampler) -> list:
    """Checks on what the messages say. A silent topic is skipped here: its
    rate row already reports it."""
    results = [check_voltage([m.voltage for _, m in sampler.history(BATTERY_TOPIC)])]
    imu = sampler.last("/imu_broadcaster/imu")
    if imu is not None:
        q = imu.orientation
        results.append(check_imu_stationary(
            (q.x, q.y, q.z, q.w), _vector(imu.linear_acceleration),
            _vector(imu.angular_velocity)))
    joints = sampler.last("/joint_states")
    if joints is not None:
        results.append(check_joint_names(list(joints.name)))
    for sensor in RANGE_SENSORS:
        reading = sampler.last(f"/range/{sensor}")
        if reading is not None:
            results.append(check_range_reading(
                sensor, reading.range, reading.min_range, reading.max_range))
    scan = sampler.last("/scan")
    if scan is not None:
        results.append(check_scan(list(scan.ranges), scan.range_min, scan.range_max,
                                  scan.angle_min, scan.angle_max))
    return results


def run_sensors(sampler: Sampler, args) -> list:
    for topic, msg_type, _, _ in RATE_CHECKS:
        sampler.watch(topic, msg_type)
    sampler.watch(BATTERY_TOPIC, BatteryState, keep=True)
    print(f"Sampling for {args.window:g} s. Keep the robot still.", flush=True)
    sampler.collect(args.window)
    results = [check_throttled(args.throttled)]
    results += [check_rate(topic, sampler.times(topic), hz, tolerance)
                for topic, _, hz, tolerance in RATE_CHECKS]
    return results + _content_checks(sampler)


def require_wheels_off_ground(args) -> None:
    if not args.wheels_off_ground:
        raise SystemExit(
            "The motors stage turns the wheels. Lift the robot so that all four "
            "wheels are off the ground, then pass --wheels-off-ground.")


def mean_wheel_velocities(samples: list, since: float) -> dict:
    """Mean velocity per joint over the (time, {joint: velocity}) samples
    received at or after `since`."""
    recent = [velocities for stamp, velocities in samples if stamp >= since]
    joints = {joint for velocities in recent for joint in velocities}
    return {joint: sum(v[joint] for v in recent if joint in v)
            / sum(1 for v in recent if joint in v) for joint in joints}


def _joint_samples(sampler: Sampler) -> list:
    return [(stamp, dict(zip(msg.name, msg.velocity)))
            for stamp, msg in sampler.history(JOINT_TOPIC)]


def _drive(sampler: Sampler, publisher, linear: float, angular: float,
           seconds: float) -> None:
    command = Twist()
    command.linear.x = max(-MAX_LINEAR, min(MAX_LINEAR, linear))
    command.angular.z = max(-MAX_ANGULAR, min(MAX_ANGULAR, angular))
    end = time.monotonic() + seconds
    while time.monotonic() < end:
        publisher.publish(command)
        sampler.collect(COMMAND_PERIOD)


def _halt(publisher) -> None:
    for _ in range(5):
        publisher.publish(Twist())
        time.sleep(COMMAND_PERIOD)


def _run_motor_sequence(sampler: Sampler, publisher) -> list:
    results = []
    last = len(MOTOR_SEQUENCE) - 1
    for index, (segment, linear, angular, seconds) in enumerate(MOTOR_SEQUENCE):
        print(f"{segment}: {linear:+.2f} m/s, {angular:+.2f} rad/s", flush=True)
        _drive(sampler, publisher, linear, angular, seconds)
        measured = mean_wheel_velocities(
            _joint_samples(sampler), time.monotonic() - AVERAGING_SECONDS)
        results.append(check_wheel_response(
            segment, measured, expected_wheel_velocities(linear, angular)))
        if index < last:
            sampler.collect(PAUSE_SECONDS)
    return results


def run_motors(sampler: Sampler, args, ask=input) -> list:
    require_wheels_off_ground(args)
    if ask("All four wheels are off the ground? Type yes to turn them: ").strip() != "yes":
        raise SystemExit("Not confirmed. Nothing was commanded.")
    sampler.watch(JOINT_TOPIC, JointState, keep=True)
    sampler.watch(BATTERY_TOPIC, BatteryState, keep=True)
    publisher = sampler.create_publisher(Twist, "/cmd_vel", 10)
    sampler.collect(1.0)  # let discovery connect the publisher to the controller
    finished = False
    try:
        results = _run_motor_sequence(sampler, publisher)
        finished = True
    finally:
        if not finished:
            _halt(publisher)
    # No zero command is sent on purpose: the wheels must stop because the
    # controller's cmd_vel_timeout (0.5 s) notices that commands ended.
    sampler.collect(STOP_WAIT_SECONDS)
    samples = _joint_samples(sampler)
    results.append(check_stop(samples[-1][1] if samples else {}))
    results.append(check_voltage([m.voltage for _, m in sampler.history(BATTERY_TOPIC)]))
    return results


STAGE_RUNNERS = {"sensors": run_sensors, "motors": run_motors}


def main(argv: list) -> int:
    args = parse_args(argv)
    runner = STAGE_RUNNERS.get(args.stage)
    if runner is None:
        print(f"stage '{args.stage}' is not implemented yet", file=sys.stderr)
        return 2
    rclpy.init()
    sampler = Sampler()
    try:
        results = runner(sampler, args)
    finally:
        sampler.destroy_node()
        rclpy.shutdown()
    table, code = summarise(results)
    print(table)
    with open(args.report, "w", encoding="utf-8") as report:
        json.dump([asdict(r) for r in results], report, indent=2)
    return code


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
