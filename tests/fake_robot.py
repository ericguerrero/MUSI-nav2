#!/usr/bin/env python3
"""A stand-in robot for testing how motion_primitives.py behaves when things
go wrong: odometry that stops, freezes, has gaps, turns to nan or jumps.

It takes /cmd_vel, moves a unicycle that obeys with a lag, stops by itself
half a second after the last command as the ROSbot's controller does, and
publishes /odometry/filtered. What really happened is written to a state file,
so a test can compare it with what the node believed.

    fake_robot.py --mode normal|frozen|silent|nan|reset [--after S] [--period S]

The fault of a mode starts --after seconds after the first non-zero command.
"""

import argparse
import json
import math
import os
import sys
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node

MODES = ("normal", "frozen", "silent", "nan", "reset")
STEP = 0.02               # s, simulation step
LAG = 0.2                 # s, how slowly the base follows a command
COMMAND_TIMEOUT = 0.5     # s, as rosbot_base_controller's cmd_vel_timeout
STATE_FILE = "/tmp/fake_robot_state.json"


class FakeRobot(Node):
    def __init__(self, mode: str, after: float, period: float):
        super().__init__("fake_robot")
        self._mode, self._after = mode, after
        self.x = self.y = self.yaw = 0.0          # true pose; yaw is not wrapped
        self._linear = self._angular = 0.0
        self._command = (0.0, 0.0)
        self._last_command = None
        self._first_motion = None
        self._offset = (0.0, 0.0, 0.0)            # subtracted from the reported pose
        self._reset_done = False
        self.commands = 0
        self.zero_after_motion = False
        self._odometry = self.create_publisher(Odometry, "/odometry/filtered", 10)
        self.create_subscription(Twist, "/cmd_vel", self._on_command, 10)
        self.create_timer(STEP, self._step)
        self.create_timer(period, self._publish)

    def _on_command(self, msg) -> None:
        now = time.monotonic()
        self._command = (msg.linear.x, msg.angular.z)
        self._last_command = now
        self.commands += 1
        if self._command != (0.0, 0.0):
            self.zero_after_motion = False
            if self._first_motion is None:
                self._first_motion = now
        elif self._first_motion is not None:
            self.zero_after_motion = True

    def _fault_active(self) -> bool:
        return (self._first_motion is not None
                and time.monotonic() - self._first_motion >= self._after)

    def _step(self) -> None:
        stale = (self._last_command is None
                 or time.monotonic() - self._last_command > COMMAND_TIMEOUT)
        target = (0.0, 0.0) if stale else self._command
        blend = STEP / (LAG + STEP)
        self._linear += blend * (target[0] - self._linear)
        self._angular += blend * (target[1] - self._angular)
        if self._mode != "frozen":
            self.x += self._linear * math.cos(self.yaw) * STEP
            self.y += self._linear * math.sin(self.yaw) * STEP
            self.yaw += self._angular * STEP
        self._write_state()

    def _write_state(self) -> None:
        state = {"x": self.x, "y": self.y, "yaw_deg": math.degrees(self.yaw),
                 "speed": math.hypot(self._linear, self._angular),
                 "commands": self.commands, "zero_after_motion": self.zero_after_motion}
        with open(STATE_FILE + ".tmp", "w", encoding="utf-8") as handle:
            json.dump(state, handle)
        os.replace(STATE_FILE + ".tmp", STATE_FILE)

    def _publish(self) -> None:
        fault = self._fault_active()
        if self._mode == "silent" and fault:
            return
        if self._mode == "reset" and fault and not self._reset_done:
            # What an EKF reset does: the reported pose restarts at the origin
            # while the robot is wherever it is.
            self._offset = (self.x, self.y, self.yaw)
            self._reset_done = True
        msg = Odometry()
        msg.header.frame_id = "odom"
        x, y, yaw = (self.x - self._offset[0], self.y - self._offset[1],
                     self.yaw - self._offset[2])
        if self._mode == "nan" and fault:
            x = y = yaw = math.nan
        msg.pose.pose.position.x = x
        msg.pose.pose.position.y = y
        msg.pose.pose.orientation.z = math.sin(yaw / 2.0)
        msg.pose.pose.orientation.w = math.cos(yaw / 2.0)
        msg.twist.twist.linear.x = self._linear
        msg.twist.twist.angular.z = self._angular
        self._odometry.publish(msg)


def main(argv: list) -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--mode", choices=MODES, default="normal")
    parser.add_argument("--after", type=float, default=2.0)
    parser.add_argument("--period", type=float, default=0.05)
    args = parser.parse_args(argv[1:])
    rclpy.init()
    node = FakeRobot(args.mode, args.after, args.period)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv))
