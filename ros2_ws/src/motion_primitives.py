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
(robot blocked?), 5 odometry stopped arriving, 6 odometry jumped,
130 interrupted.
"""

import math
import signal
import sys
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions
from rclpy.utilities import remove_ros_args

from motion_primitives_core import (
    Limits,
    Primitive,
    PrimitiveError,
    PrimitiveRunner,
    expected_duration,
    parse_primitives,
    pose_is_finite,
    pose_jumped,
)

EXIT_OK = 0
EXIT_USAGE = 2
EXIT_NO_FEEDBACK = 3
EXIT_TIMEOUT = 4
EXIT_STALE = 5
EXIT_JUMP = 6
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


def complain(message: str) -> None:
    print(f"motion_primitives: {message}", file=sys.stderr)


class MotionPrimitives(Node):
    """Odometry in, velocity commands out, one primitive at a time."""

    def __init__(self):
        super().__init__("motion_primitives")
        self.declare_parameter("feedback_topic", "/odometry/filtered")
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.limits = Limits(max_linear=self._speed("max_linear", Limits.max_linear),
                             max_angular=self._speed("max_angular", Limits.max_angular))
        self.feedback_topic = self.get_parameter("feedback_topic").value
        self._commands = self.create_publisher(
            Twist, self.get_parameter("cmd_vel_topic").value, 10)
        # Best effort: over Wi-Fi a late pose is worse than a missing one.
        self.create_subscription(Odometry, self.feedback_topic, self._on_odometry,
                                 qos_profile_sensor_data)
        self._pose = None
        self._last_feedback = None
        self._poses_seen = 0

    def _speed(self, name: str, default: float) -> float:
        # Any type accepted, then converted: "max_angular:=1" arrives as an
        # integer, and a parameter declared as a float would refuse it.
        self.declare_parameter(name, default, ParameterDescriptor(dynamic_typing=True))
        value = self.get_parameter(name).value
        try:
            return float(value)
        except (TypeError, ValueError):
            raise PrimitiveError(f"{name} must be a number, got '{value}'") from None

    def _on_odometry(self, msg) -> None:
        p = msg.pose.pose
        pose = (p.position.x, p.position.y, yaw_from_quaternion(p.orientation))
        if not pose_is_finite(pose):
            # Ignored as if it had not arrived, so that a stream of them ends
            # the move through the "odometry stopped" exit.
            return
        self._pose = pose
        self._last_feedback = time.monotonic()
        self._poses_seen += 1

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

    def _publish(self, linear: float, angular: float) -> None:
        command = Twist()
        command.linear.x = float(linear)
        command.angular.z = float(angular)
        self._commands.publish(command)

    def _give_up(self, primitive: Primitive, started: float, now: float):
        """The exit code for a move that must end now, or None to carry on."""
        if feedback_is_stale(now, self._last_feedback):
            complain(f"no odometry for {STALE_AFTER_S} s, stopping")
            return EXIT_STALE
        if now - started > timeout_for(primitive, self.limits):
            complain("took too long, is the robot blocked? Stopping")
            return EXIT_TIMEOUT
        return None

    def run(self, primitive: Primitive) -> int:
        runner = PrimitiveRunner(primitive, self.limits)
        period = 1.0 / CONTROL_RATE_HZ
        started = tick = time.monotonic()
        used_pose, used_at, used_count = None, None, None
        command = (0.0, 0.0)
        while not runner.done:
            self._spin_until(tick + period)
            tick = time.monotonic()
            code = self._give_up(primitive, started, tick)
            if code is not None:
                return code
            # The control law runs only on a pose it has not seen. Feeding it
            # the same pose again reads as "the robot is standing still", and
            # with odometry arriving in gaps that ended moves mid-motion.
            if self._poses_seen != used_count:
                dt = period if used_at is None else self._last_feedback - used_at
                if used_pose is not None and pose_jumped(used_pose, self._pose, dt):
                    complain("odometry jumped (was the pose estimate reset?), stopping")
                    return EXIT_JUMP
                command = runner.update(self._pose, dt)
                used_pose, used_at, used_count = (self._pose, self._last_feedback,
                                                  self._poses_seen)
            self._publish(*command)
        covered = primitive.amount - runner.remaining
        print(f"{describe(primitive, covered)}, {time.monotonic() - started:.1f} s",
              flush=True)
        return EXIT_OK

    def stop(self) -> None:
        """Zero commands on the way out. The robot also stops by itself half a
        second after the last command, so this only makes the stop prompt."""
        for _ in range(5):
            self._publish(0.0, 0.0)
            time.sleep(1.0 / CONTROL_RATE_HZ)


def _usage_error(error: Exception) -> int:
    complain(str(error))
    print(USAGE, file=sys.stderr)
    return EXIT_USAGE


def _run_all(node: MotionPrimitives, primitives: list) -> int:
    try:
        if not node.wait_for_feedback():
            complain(f"no odometry on {node.feedback_topic} in {FEEDBACK_WAIT_S:g} s; "
                     "is the robot or the simulation running?")
            return EXIT_NO_FEEDBACK
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
    # rclpy's own signal handlers shut the context down before Python sees the
    # interrupt, and the zero commands then cannot be published: Ctrl-C left
    # the robot to its own timeout. So the signals stay with Python, and
    # SIGTERM is treated like Ctrl-C. SIGINT is set explicitly as well: a
    # process started in the background by a script inherits it as "ignored",
    # and a program that moves a robot must always be stoppable.
    rclpy.init(args=argv, signal_handler_options=SignalHandlerOptions.NO)
    signal.signal(signal.SIGINT, signal.default_int_handler)
    signal.signal(signal.SIGTERM, signal.default_int_handler)
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
