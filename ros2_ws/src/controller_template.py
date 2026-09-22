#!/usr/bin/env python3
"""Starting point for a student-written motion controller.

Reads the robot's true pose from /groundtruth/odom, decides what velocity to
command, and publishes it on /cmd_vel. The reference trajectory lives here too,
so the whole control loop is one readable file.

Only one function is meant to be replaced: compute_command(). Everything else is
plumbing you can leave alone.

    ros2 run ... controller_template.py --ros-args -p reference:=circle

/cmd_vel carries geometry_msgs/Twist, which is the command interface for the
diff-drive, Ackermann, mecanum and tricycle controllers alike, so a controller
written here keeps working if you change the robot's kinematics.
"""

import math

# TurtleBot3 Waffle limits. Commands are clamped to these so a runaway gain
# produces a slow robot rather than an unphysical one.
MAX_LINEAR_VEL = 0.26
MAX_ANGULAR_VEL = 1.82

# Stop when this close to the target, so the robot settles instead of hunting.
GOAL_TOLERANCE_M = 0.05

DEFAULT_LINEAR_GAIN = 0.5
DEFAULT_ANGULAR_GAIN = 1.5


def normalize_angle(angle):
    """Wrap an angle into [-pi, pi].

    Without this a controller told to turn 359 degrees left turns the long way
    round instead of 1 degree right.
    """
    return math.atan2(math.sin(angle), math.cos(angle))


def reference_at(t, mode, params):
    """Return the target pose (x, y, yaw) at time t seconds.

    Replace or extend this to give your controller something else to follow.
    """
    if mode == "point":
        return (params["x"], params["y"], params.get("yaw", 0.0))

    if mode == "circle":
        radius = params["radius"]
        omega = params["speed"] / radius
        angle = omega * t
        # Heading is tangent to the circle, a quarter turn ahead of the radius.
        return (radius * math.cos(angle),
                radius * math.sin(angle),
                normalize_angle(angle + math.pi / 2))

    if mode == "figure8":
        size = params["size"]
        angle = params["speed"] * t
        # Lemniscate of Gerono: crosses its own centre twice per lap.
        x = size * math.sin(angle)
        y = size * math.sin(angle) * math.cos(angle)
        dx = size * math.cos(angle)
        dy = size * math.cos(2 * angle)
        return (x, y, math.atan2(dy, dx))

    if mode == "waypoints":
        points = params["points"]
        dwell = params.get("dwell", 5.0)
        index = min(int(t / dwell), len(points) - 1)
        x, y = points[index]
        return (x, y, 0.0)

    raise ValueError(f"unknown reference mode: {mode!r}")


def pose_error(current, target):
    """Return (distance, heading_error) from current pose to target pose.

    heading_error is the turn needed to point at the target, not the difference
    between the two yaw values.
    """
    dx = target[0] - current[0]
    dy = target[1] - current[1]
    distance = math.hypot(dx, dy)
    heading_error = normalize_angle(math.atan2(dy, dx) - current[2])
    return distance, heading_error


def compute_command(pose, twist, reference):
    """Decide what velocity to command. THIS IS THE FUNCTION TO REPLACE.

    Args:
        pose: (x, y, yaw) where the robot actually is, from groundtruth.
        twist: (linear_x, angular_z) how fast it is currently moving.
        reference: (x, y, yaw) where it should be now.

    Returns:
        (linear_x, angular_z) to publish on /cmd_vel.

    The shipped implementation is a proportional controller: drive at a speed
    proportional to how far away the target is, turn at a rate proportional to
    how far off-heading it is. It works, but it overshoots and it ignores the
    reference heading entirely. Improving on it is the exercise.
    """
    distance, heading_error = pose_error(pose, reference)

    if distance < GOAL_TOLERANCE_M:
        return 0.0, 0.0

    linear = DEFAULT_LINEAR_GAIN * distance
    angular = DEFAULT_ANGULAR_GAIN * heading_error

    linear = max(0.0, min(linear, MAX_LINEAR_VEL))
    angular = max(-MAX_ANGULAR_VEL, min(angular, MAX_ANGULAR_VEL))
    return linear, angular


def yaw_from_quaternion(q):
    """Extract yaw from a quaternion, ignoring roll and pitch."""
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node

CONTROL_RATE_HZ = 20.0


class ControllerTemplate(Node):
    """Closes the loop: groundtruth pose in, /cmd_vel out, reference published."""

    def __init__(self):
        super().__init__("controller_template")

        self.declare_parameter("reference", "circle")
        self.declare_parameter("radius", 1.0)
        self.declare_parameter("speed", 0.15)
        self.declare_parameter("size", 1.5)
        self.declare_parameter("target_x", 1.0)
        self.declare_parameter("target_y", 1.0)
        self.declare_parameter("dwell", 8.0)

        self._mode = self.get_parameter("reference").value
        self._params = self._reference_params()

        self._cmd_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        # Published so RViz can draw the target and the analyzer can log it
        # alongside groundtruth. Without it, tracking error is unmeasurable.
        self._ref_pub = self.create_publisher(PoseStamped, "/reference/pose", 10)
        self.create_subscription(Odometry, "/groundtruth/odom", self._on_odom, 10)

        self._pose = None
        self._twist = (0.0, 0.0)
        self._start_s = None
        self.create_timer(1.0 / CONTROL_RATE_HZ, self._on_control_tick)

        self.get_logger().info(f"Following '{self._mode}' reference")

    def _reference_params(self):
        value = self.get_parameter
        return {
            "point": lambda: {"x": value("target_x").value,
                              "y": value("target_y").value},
            "circle": lambda: {"radius": value("radius").value,
                               "speed": value("speed").value},
            "figure8": lambda: {"size": value("size").value,
                                "speed": value("speed").value},
            "waypoints": lambda: {"points": [(1.0, 0.0), (1.0, 1.0),
                                             (0.0, 1.0), (0.0, 0.0)],
                                  "dwell": value("dwell").value},
        }.get(self._mode, dict)()

    def _on_odom(self, msg):
        p = msg.pose.pose
        self._pose = (p.position.x, p.position.y, yaw_from_quaternion(p.orientation))
        self._twist = (msg.twist.twist.linear.x, msg.twist.twist.angular.z)

    def _on_control_tick(self):
        if self._pose is None:
            return

        now_s = self.get_clock().now().nanoseconds * 1e-9
        if self._start_s is None:
            self._start_s = now_s
        elapsed = now_s - self._start_s

        reference = reference_at(elapsed, self._mode, self._params)
        linear, angular = compute_command(self._pose, self._twist, reference)

        cmd = Twist()
        cmd.linear.x = float(linear)
        cmd.angular.z = float(angular)
        self._cmd_pub.publish(cmd)
        self._publish_reference(reference)

    def _publish_reference(self, reference):
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "world"
        msg.pose.position.x = float(reference[0])
        msg.pose.position.y = float(reference[1])
        msg.pose.orientation.z = math.sin(reference[2] / 2.0)
        msg.pose.orientation.w = math.cos(reference[2] / 2.0)
        self._ref_pub.publish(msg)

    def stop_robot(self):
        """Leave the robot stationary rather than coasting on the last command."""
        self._cmd_pub.publish(Twist())


def main(args=None):
    rclpy.init(args=args)
    node = ControllerTemplate()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop_robot()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
