#!/usr/bin/env python3
"""Send the same velocity commands to any robot and measure what it does.

    python3 /ros2_ws/src/drive_test.py --ros-args -p robot:=turtlebot3
    python3 /ros2_ws/src/drive_test.py --ros-args -p robot:=rosbot
    python3 /ros2_ws/src/drive_test.py --ros-args -p robot:=rosbot_mecanum
    python3 /ros2_ws/src/drive_test.py --ros-args -p robot:=ackermann

Runs four manoeuvres in order (straight, spin on the spot, arc, sideways), each
for a fixed simulation time with a stop between them, and prints one table row
per manoeuvre: the motion that was commanded, the motion Gazebo says happened,
and the motion the wheel odometry believes happened. All three are given in the
frame the robot had when the manoeuvre started: metres forward, metres to the
left, degrees turned counter-clockwise.

Simulation only: the true pose comes from Gazebo. Other parameters:
    odom_topic:=/odometry/filtered   compare another nav_msgs/Odometry topic
    truth_model:=<name>              Gazebo model to follow (ackermann: the
                                     topic that carries the true pose)

Exit codes: 0 done, 2 bad parameter, 3 simulation not ready or stopped,
130 interrupted.
"""

import signal
import subprocess
import sys
import threading
import time

import rclpy
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import JointState

from drive_test_core import (
    MANOEUVRES,
    DiffDriveOdometry,
    DriveTestError,
    Manoeuvre,
    SimClock,
    parse_ign_model_pose,
    preset_for,
    relative_motion,
    table_header,
    table_row,
    yaw_from_quaternion,
)

EXIT_OK = 0
EXIT_USAGE = 2
EXIT_NOT_READY = 3
EXIT_INTERRUPTED = 130

COMMAND_PERIOD_S = 0.05
READY_WAIT_S = 15.0
# Long enough for the slowest deceleration (TurtleBot3 from 0.2 m/s) and for
# the last odometry message of the move to arrive.
SETTLE_S = 2.0
# Wall-clock seconds allowed per simulated second before the simulation is
# taken to be paused or gone.
MAX_SLOWDOWN = 20.0
IGN_TIMEOUT_S = 10.0

# Set by SIGINT and SIGTERM, read between steps. Letting Python raise
# KeyboardInterrupt wherever the signal lands does not work here: inside
# rclpy's message take it surfaces as a RuntimeError, and the run ended with a
# traceback instead of a clean stop.
INTERRUPTED = threading.Event()


def pose_from_msg(pose) -> tuple:
    q = pose.orientation
    return pose.position.x, pose.position.y, yaw_from_quaternion(q.x, q.y, q.z, q.w)


def ign_model_pose(model: str):
    """Ask Gazebo Fortress for the model's pose; None when it has no answer."""
    try:
        result = subprocess.run(["ign", "model", "-m", model, "-p"], capture_output=True,
                                text=True, timeout=IGN_TIMEOUT_S, check=False)
    except (OSError, subprocess.TimeoutExpired):
        return None
    return parse_ign_model_pose(result.stdout)


class DriveTest(Node):
    """Commands out; simulator pose, wheel odometry and simulation time in."""

    def __init__(self):
        super().__init__("drive_test")
        self.declare_parameter("robot", "turtlebot3")
        self.declare_parameter("odom_topic", "")
        self.declare_parameter("truth_model", "")
        self.robot = self.get_parameter("robot").value
        preset = preset_for(self.robot)
        self.model = self.get_parameter("truth_model").value or preset["model"]
        self.truth_source = preset["truth"]
        self.odom_topic = self.get_parameter("odom_topic").value or preset["odom"]

        self._clock = SimClock()
        self._truth = None
        self._odom = None
        self._wheels = None
        self._commands = self.create_publisher(Twist, preset["cmd_vel"], 10)
        # Depth 1: after a pause in spinning, the one message waiting must be
        # the latest, or a manoeuvre is timed from a time already past.
        self.create_subscription(
            Clock, "/clock", self._on_clock,
            QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT))
        if self.truth_source == "model_states":
            self.create_subscription(ModelStates, "/model_states", self._on_model_states,
                                     qos_profile_sensor_data)
        if self.truth_source == "odometry":
            self.create_subscription(Odometry, self.model, self._on_true_odometry,
                                     qos_profile_sensor_data)
        if self.odom_topic:
            self.create_subscription(Odometry, self.odom_topic, self._on_odometry,
                                     qos_profile_sensor_data)
        else:
            self._left, self._right, radius, separation = preset["wheels"]
            self._wheels = DiffDriveOdometry(radius, separation)
            self.odom_topic = "/joint_states (wheel angles, integrated here)"
            # Reliable and deep: a lost encoder message is a piece of path
            # missing from the dead reckoning.
            self.create_subscription(JointState, "/joint_states", self._on_joint_states, 50)

    def _on_clock(self, msg) -> None:
        self._clock.update(msg.clock.sec + msg.clock.nanosec * 1e-9, time.monotonic())

    def _on_model_states(self, msg) -> None:
        if self.model in msg.name:
            self._truth = pose_from_msg(msg.pose[msg.name.index(self.model)])

    def _on_true_odometry(self, msg) -> None:
        self._truth = pose_from_msg(msg.pose.pose)

    def _on_odometry(self, msg) -> None:
        self._odom = pose_from_msg(msg.pose.pose)

    def _on_joint_states(self, msg) -> None:
        if self._left in msg.name and self._right in msg.name:
            self._odom = self._wheels.update(msg.position[msg.name.index(self._left)],
                                             msg.position[msg.name.index(self._right)])

    def true_pose(self):
        if self.truth_source != "ign_model":
            return self._truth
        pose = ign_model_pose(self.model)
        # Ctrl-C in a terminal also ends the `ign` child, which then looks
        # like a missing model.
        if INTERRUPTED.is_set():
            raise KeyboardInterrupt
        return pose

    def _missing(self) -> list:
        """What the simulation has not delivered yet, in words for the student."""
        missing = []
        if self._clock.now(time.monotonic()) is None:
            missing.append("simulation time on /clock")
        if self._odom is None:
            missing.append(f"wheel odometry on {self.odom_topic}")
        if self.true_pose() is None:
            missing.append(f"true pose from Gazebo ({self.model})")
        return missing

    def _spin(self, timeout_s: float) -> None:
        rclpy.spin_once(self, timeout_sec=timeout_s)
        if INTERRUPTED.is_set():
            raise KeyboardInterrupt

    def wait_until_ready(self) -> None:
        deadline = time.monotonic() + READY_WAIT_S
        while time.monotonic() < deadline:
            self._spin(0.1)
            waiting_for_topic = (self._clock.now(time.monotonic()) is None or self._odom is None
                                 or (self.truth_source != "ign_model" and self._truth is None))
            if not waiting_for_topic:
                break
        missing = self._missing()
        if missing:
            raise DriveTestError(
                f"no {' and no '.join(missing)}. Is the simulation for "
                f"robot:={self.robot} running in this container?")

    def _publish(self, vx: float, vy: float, wz: float) -> None:
        command = Twist()
        command.linear.x, command.linear.y, command.angular.z = vx, vy, wz
        self._commands.publish(command)

    def _fresh_sim_time(self) -> float:
        """Simulation time from a /clock message that arrives from now on.
        Reading the true pose blocks for about a second, and the time held
        from before it would shorten the manoeuvre by that much."""
        asked = time.monotonic()
        while self._clock.updated_at < asked:
            if time.monotonic() - asked > READY_WAIT_S:
                raise DriveTestError("the simulation clock stopped; is Gazebo paused or closed?")
            self._spin(0.1)
        return self._clock.now(time.monotonic())

    def _hold(self, vx: float, vy: float, wz: float, sim_seconds: float) -> None:
        """Repeat one command until that much simulation time has passed."""
        end = self._fresh_sim_time() + sim_seconds
        give_up = time.monotonic() + MAX_SLOWDOWN * sim_seconds
        last_sent = 0.0
        while self._clock.now(time.monotonic()) < end:
            if time.monotonic() > give_up:
                raise DriveTestError("the simulation clock stopped; is Gazebo paused or closed?")
            if time.monotonic() - last_sent >= COMMAND_PERIOD_S:
                self._publish(vx, vy, wz)
                last_sent = time.monotonic()
            self._spin(COMMAND_PERIOD_S / 5)

    def _poses(self) -> tuple:
        truth = self.true_pose()
        if truth is None:
            raise DriveTestError(f"Gazebo no longer reports the true pose ({self.model})")
        return truth, self._odom

    def run(self, manoeuvre: Manoeuvre) -> str:
        """Drive one manoeuvre from rest to rest and return its table row."""
        true_start, odom_start = self._poses()
        self._hold(manoeuvre.vx, manoeuvre.vy, manoeuvre.wz, manoeuvre.duration)
        self._hold(0.0, 0.0, 0.0, SETTLE_S)
        true_end, odom_end = self._poses()
        return table_row(manoeuvre, relative_motion(true_start, true_end),
                         relative_motion(odom_start, odom_end))

    def stop(self) -> None:
        """Zero commands on the way out. The TurtleBot3 keeps its last command
        for ever, so without these Ctrl-C would leave it driving."""
        for _ in range(5):
            self._publish(0.0, 0.0, 0.0)
            time.sleep(COMMAND_PERIOD_S)


def complain(message: str) -> None:
    print(f"drive_test: {message}", file=sys.stderr)


def _run_all(node: DriveTest) -> int:
    try:
        node.wait_until_ready()
        print(f"robot: {node.robot}   true pose: Gazebo, {node.model}   "
              f"wheel odometry: {node.odom_topic}")
        print(table_header(), flush=True)
        for manoeuvre in MANOEUVRES:
            print(node.run(manoeuvre), flush=True)
        return EXIT_OK
    except DriveTestError as error:
        complain(str(error))
        return EXIT_NOT_READY
    except KeyboardInterrupt:
        return EXIT_INTERRUPTED
    finally:
        node.stop()


def main(argv: list) -> int:
    # rclpy's own handlers shut the context down before Python sees the
    # interrupt, and the zero commands then cannot be published. So the signals
    # stay with Python. SIGINT is set explicitly: a process started in the
    # background by a script inherits it as "ignored".
    rclpy.init(args=argv, signal_handler_options=SignalHandlerOptions.NO)
    signal.signal(signal.SIGINT, lambda *_: INTERRUPTED.set())
    signal.signal(signal.SIGTERM, lambda *_: INTERRUPTED.set())
    try:
        try:
            node = DriveTest()
        except DriveTestError as error:
            complain(str(error))
            return EXIT_USAGE
        try:
            return _run_all(node)
        finally:
            node.destroy_node()
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main(sys.argv))
