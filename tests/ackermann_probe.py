#!/usr/bin/env python3
"""Send one constant /cmd_vel to the Ackermann robot and report how it moved.

Prints one line of key=value pairs, for the simulator's true pose and for wheel
odometry: distance travelled along the path, rotation, and the turn radius once
the steering has settled. tests/test_ackermann_sim.sh parses that line.

Usage:
    python3 ackermann_probe.py --linear 0.3 --angular 2.0 --duration 8
"""

import argparse
import math
import sys
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node

CMD_TOPIC = "/cmd_vel"
TRUTH_TOPIC = "/groundtruth/odom"
ODOM_TOPIC = "/ackermann_steering_controller/odometry"
PUBLISH_PERIOD_S = 0.05
# Below this rotation a path counts as straight and has no radius.
MIN_ROTATION_RAD = 0.05


class NoDataError(RuntimeError):
    """A pose topic delivered too few samples to measure anything."""


def yaw_from_quaternion(x, y, z, w):
    """Rotation about the vertical axis, in radians."""
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def angle_difference(a, b):
    """a - b wrapped into (-pi, pi], so a turn through +-pi is not a jump."""
    return math.atan2(math.sin(a - b), math.cos(a - b))


def path_metrics(samples):
    """Return (distance, rotation) summed along (t, x, y, yaw) samples.

    Both are sums over consecutive samples: a full circle gives its
    circumference and 2*pi, not zero.
    """
    distance = 0.0
    rotation = 0.0
    for (_, x0, y0, yaw0), (_, x1, y1, yaw1) in zip(samples, samples[1:]):
        distance += math.hypot(x1 - x0, y1 - y0)
        rotation += angle_difference(yaw1, yaw0)
    return distance, rotation


def turn_radius(distance, rotation):
    """Radius of the arc with that length and rotation; infinite when straight."""
    if abs(rotation) < MIN_ROTATION_RAD:
        return math.inf
    return distance / abs(rotation)


def after(samples, start_s):
    """Samples taken at or after start_s."""
    return [s for s in samples if s[0] >= start_s]


def summarise(samples, settle_s):
    """Distance and rotation over the whole run, radius after the settle time."""
    if len(samples) < 2:
        raise NoDataError("fewer than two pose samples")
    distance, rotation = path_metrics(samples)
    settled = after(samples, samples[0][0] + settle_s)
    return distance, rotation, turn_radius(*path_metrics(settled))


class Probe(Node):
    """Publishes the command and records both pose topics."""

    def __init__(self, linear, angular):
        super().__init__("ackermann_probe")
        self._cmd = Twist()
        self._cmd.linear.x = linear
        self._cmd.angular.z = angular
        self.truth = []
        self.odom = []
        self._pub = self.create_publisher(Twist, CMD_TOPIC, 10)
        self.create_subscription(Odometry, TRUTH_TOPIC, lambda m: self._record(self.truth, m), 50)
        self.create_subscription(Odometry, ODOM_TOPIC, lambda m: self._record(self.odom, m), 50)
        self._timer = self.create_timer(PUBLISH_PERIOD_S, lambda: self._pub.publish(self._cmd))

    def _record(self, samples, msg):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        samples.append((time.monotonic(), p.x, p.y, yaw_from_quaternion(q.x, q.y, q.z, q.w)))

    def stop(self):
        self._timer.cancel()
        self._pub.publish(Twist())


def parse_args(argv):
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--linear", type=float, required=True, help="linear.x in m/s")
    parser.add_argument("--angular", type=float, required=True, help="angular.z in rad/s")
    parser.add_argument("--duration", type=float, default=6.0, help="seconds to drive")
    parser.add_argument("--settle", type=float, default=2.0,
                        help="seconds left out of the radius while the wheels steer")
    return parser.parse_args(argv)


def main(argv=None):
    args = parse_args(argv)
    rclpy.init()
    node = Probe(args.linear, args.angular)
    end = time.monotonic() + args.duration
    while time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=0.05)
    node.stop()
    try:
        fields = []
        for name, samples in (("truth", node.truth), ("odom", node.odom)):
            distance, rotation, radius = summarise(samples, args.settle)
            fields += [f"{name}_distance={distance:.4f}", f"{name}_rotation={rotation:.4f}",
                       f"{name}_radius={radius:.4f}"]
    except NoDataError as err:
        print(f"ackermann_probe: {err}", file=sys.stderr)
        return 1
    finally:
        node.destroy_node()
        rclpy.shutdown()
    print(" ".join(fields))
    return 0


if __name__ == "__main__":
    sys.exit(main())
