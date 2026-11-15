#!/usr/bin/env python3
"""Tell the robot to go forward a distance or turn an angle, and stop there.

    python3 motion_primitives.py forward 2.0
    python3 motion_primitives.py turn 90
    python3 motion_primitives.py forward 1 turn 90 forward 1 turn 90
    python3 motion_primitives.py forward 2.0 --ros-args -p max_linear:=0.1

Reads the robot's pose from odometry and publishes velocity commands until the
pose says the distance or the angle is covered. Positive turns are to the left;
a negative distance drives backward.

Works the same against the ROSbot 2R simulation and the real robot: both give
/odometry/filtered and take /cmd_vel. Timing uses this computer's own clock and
never the message stamps, so a robot whose clock is hours off is no problem.

Exit codes: 0 done, 2 bad input, 3 no odometry at start, 4 took too long
(robot blocked?), 5 odometry stopped arriving, 130 interrupted.
"""

import math
import sys
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.utilities import remove_ros_args

from motion_primitives_core import (
    Limits,
    Primitive,
    PrimitiveError,
    PrimitiveRunner,
    expected_duration,
    parse_primitives,
)

EXIT_OK = 0
EXIT_USAGE = 2
EXIT_NO_FEEDBACK = 3
EXIT_TIMEOUT = 4
EXIT_STALE = 5
EXIT_INTERRUPTED = 130

CONTROL_RATE_HZ = 20.0
FEEDBACK_WAIT_S = 5.0
# The robot publishes odometry at 20 Hz; half a second of silence is ten
# missed messages, which is a lost link and not jitter.
STALE_AFTER_S = 0.5

USAGE = ("usage: motion_primitives.py forward <metres> | turn <degrees> "
         "[more pairs ...] [--ros-args -p max_linear:=0.2 -p max_angular:=0.5]")


def timeout_for(primitive: Primitive, limits: Limits) -> float:
    """How long to keep trying before deciding the robot is blocked."""
    return 3 * expected_duration(primitive, limits) + 5


def feedback_is_stale(now: float, last_feedback) -> bool:
    return last_feedback is None or now - last_feedback > STALE_AFTER_S


def split_arguments(argv: list) -> list:
    """The primitive words: argv without the program name and the ROS arguments."""
    return remove_ros_args(argv)[1:]


def yaw_from_quaternion(q) -> float:
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def describe(primitive: Primitive, covered: float) -> str:
    if primitive.kind == "turn":
        return (f"turn {math.degrees(primitive.amount):+.1f} deg: odometry reports "
                f"{math.degrees(covered):+.1f} deg")
    return f"forward {primitive.amount:+.3f} m: odometry reports {covered:+.3f} m"


class MotionPrimitives(Node):
    """Odometry in, velocity commands out, one primitive at a time."""

    def __init__(self):
        super().__init__("motion_primitives")
        self.declare_parameter("feedback_topic", "/odometry/filtered")
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.declare_parameter("max_linear", Limits.max_linear)
        self.declare_parameter("max_angular", Limits.max_angular)
        self.limits = Limits(max_linear=self.get_parameter("max_linear").value,
                             max_angular=self.get_parameter("max_angular").value)
        self.feedback_topic = self.get_parameter("feedback_topic").value
        self._commands = self.create_publisher(
            Twist, self.get_parameter("cmd_vel_topic").value, 10)
        # Best effort: over Wi-Fi a late pose is worse than a missing one.
        self.create_subscription(Odometry, self.feedback_topic, self._on_odometry,
                                 qos_profile_sensor_data)
        self._pose = None
        self._last_feedback = None

    def _on_odometry(self, msg) -> None:
        p = msg.pose.pose
        self._pose = (p.position.x, p.position.y, yaw_from_quaternion(p.orientation))
        self._last_feedback = time.monotonic()

    def _spin_until(self, deadline: float) -> None:
        while True:
            left = deadline - time.monotonic()
            if left <= 0:
                return
            rclpy.spin_once(self, timeout_sec=left)

    def wait_for_feedback(self) -> bool:
        deadline = time.monotonic() + FEEDBACK_WAIT_S
        while self._pose is None and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
        return self._pose is not None

    def run(self, primitive: Primitive) -> int:
        runner = PrimitiveRunner(primitive, self.limits)
        period = 1.0 / CONTROL_RATE_HZ
        started = previous = time.monotonic()
        while not runner.done:
            self._spin_until(previous + period)
            now = time.monotonic()
            if feedback_is_stale(now, self._last_feedback):
                print(f"motion_primitives: no odometry for {STALE_AFTER_S} s, stopping",
                      file=sys.stderr)
                return EXIT_STALE
            if now - started > timeout_for(primitive, self.limits):
                print("motion_primitives: took too long, is the robot blocked? Stopping",
                      file=sys.stderr)
                return EXIT_TIMEOUT
            linear, angular = runner.update(self._pose, now - previous)
            previous = now
            command = Twist()
            command.linear.x = float(linear)
            command.angular.z = float(angular)
            self._commands.publish(command)
        covered = primitive.amount - runner.remaining
        print(f"{describe(primitive, covered)}, {time.monotonic() - started:.1f} s",
              flush=True)
        return EXIT_OK

    def stop(self) -> None:
        """Zero commands on the way out. The robot also stops by itself half a
        second after the last command, so this only makes the stop prompt."""
        for _ in range(5):
            self._commands.publish(Twist())
            time.sleep(1.0 / CONTROL_RATE_HZ)


def _usage_error(error: Exception) -> int:
    print(f"motion_primitives: {error}", file=sys.stderr)
    print(USAGE, file=sys.stderr)
    return EXIT_USAGE


def _run_all(node: MotionPrimitives, primitives: list) -> int:
    if not node.wait_for_feedback():
        print(f"motion_primitives: no odometry on {node.feedback_topic} in "
              f"{FEEDBACK_WAIT_S:g} s; is the robot or the simulation running?",
              file=sys.stderr)
        return EXIT_NO_FEEDBACK
    try:
        for primitive in primitives:
            code = node.run(primitive)
            if code != EXIT_OK:
                return code
        return EXIT_OK
    except KeyboardInterrupt:
        return EXIT_INTERRUPTED
    finally:
        node.stop()


def main(argv: list) -> int:
    # Parsed before ROS starts: a typing slip must not create a publisher.
    try:
        primitives = parse_primitives(split_arguments(argv))
    except PrimitiveError as error:
        return _usage_error(error)
    rclpy.init(args=argv)
    try:
        try:
            node = MotionPrimitives()
        except PrimitiveError as error:
            return _usage_error(error)
        try:
            return _run_all(node, primitives)
        finally:
            node.destroy_node()
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main(sys.argv))
