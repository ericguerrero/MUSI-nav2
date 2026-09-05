#!/usr/bin/env python3
"""Publish the simulator's true robot pose as a clean, reusable signal.

Gazebo knows exactly where the robot is. Nav2's AMCL only estimates it, and
odometry drifts. Control exercises need the true pose as feedback, so this node
republishes it under stable names decoupled from the navigation stack.

Requires gzserver to load libgazebo_ros_state.so, which simulation.launch.py
does. Without that plugin /model_states never appears and this node says so
rather than substituting a different signal.
"""

# Absorbs floating-point error in timestamp subtraction.
_TIMING_EPSILON_S = 1e-9


def find_model_index(names, model_name):
    """Return the index of model_name in names, or None when it is absent."""
    try:
        return list(names).index(model_name)
    except ValueError:
        return None


def extract_model_state(msg, model_name):
    """Return (pose, twist) for model_name from a ModelStates message.

    Returns None when the model is not in the message, so callers can warn
    instead of publishing a pose belonging to some other model.
    """
    index = find_model_index(msg.name, model_name)
    if index is None:
        return None
    return msg.pose[index], msg.twist[index]


def should_publish(last_publish_s, now_s, rate_hz):
    """Decide whether enough time has passed to publish again.

    Gazebo emits /model_states far faster than any controller needs, so
    republishing every message wastes bandwidth. A non-positive rate disables
    throttling.
    """
    if last_publish_s is None or rate_hz <= 0:
        return True
    # Timestamps rarely land exactly on the interval: 100.02 - 100.0 evaluates to
    # 0.019999999999996447, which would drop a sample that is genuinely due.
    return (now_s - last_publish_s) + _TIMING_EPSILON_S >= (1.0 / rate_hz)


import rclpy
from geometry_msgs.msg import PoseStamped, TransformStamped
from gazebo_msgs.msg import ModelStates
from nav_msgs.msg import Odometry
from rclpy.node import Node
from tf2_ros import TransformBroadcaster

# The spawned entity name, set by simulation.launch.py's -entity argument.
DEFAULT_MODEL_NAME = "turtlebot3_waffle"
# 0.0 means publish every message. The world file's <update_rate> already governs
# the rate; throttling again here aliases 50 Hz down to roughly 33 Hz. Set a
# positive value only to downsample below the simulator's rate.
DEFAULT_PUBLISH_RATE_HZ = 0.0
WORLD_FRAME = "world"
ROBOT_FRAME = "base_footprint_gt"

# Gazebo emits /model_states continuously, so an unthrottled warning would flood
# the log at the message rate.
_WARN_INTERVAL_S = 5.0


class GroundtruthPublisher(Node):
    """Republish Gazebo's true robot pose under stable, Nav2-independent names."""

    def __init__(self):
        super().__init__("groundtruth_pose")

        self.declare_parameter("model_name", DEFAULT_MODEL_NAME)
        self.declare_parameter("publish_rate", DEFAULT_PUBLISH_RATE_HZ)
        self._model_name = self.get_parameter("model_name").value
        self._publish_rate = self.get_parameter("publish_rate").value

        self._pose_pub = self.create_publisher(PoseStamped, "/groundtruth/pose", 10)
        self._odom_pub = self.create_publisher(Odometry, "/groundtruth/odom", 10)
        self._tf = TransformBroadcaster(self)
        self.create_subscription(ModelStates, "/model_states", self._on_model_states, 10)

        self._last_publish_s = None
        self._last_warn_s = None

        rate_note = ("every message" if self._publish_rate <= 0
                     else f"{self._publish_rate} Hz")
        self.get_logger().info(
            f"Tracking model '{self._model_name}', publishing {rate_note}")

    def _now_s(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def _on_model_states(self, msg):
        now_s = self._now_s()
        if not should_publish(self._last_publish_s, now_s, self._publish_rate):
            return

        state = extract_model_state(msg, self._model_name)
        if state is None:
            self._warn_model_missing(msg.name, now_s)
            return

        pose, twist = state
        self._last_publish_s = now_s
        stamp = self.get_clock().now().to_msg()
        self._publish_pose(pose, stamp)
        self._publish_odom(pose, twist, stamp)
        self._broadcast_tf(pose, stamp)

    def _warn_model_missing(self, available, now_s):
        """Say the model is missing rather than silently substituting another signal."""
        if self._last_warn_s is not None and now_s - self._last_warn_s < _WARN_INTERVAL_S:
            return
        self._last_warn_s = now_s
        self.get_logger().warn(
            f"Model '{self._model_name}' not in /model_states; publishing nothing. "
            f"Available: {sorted(available)}")

    def _publish_pose(self, pose, stamp):
        msg = PoseStamped()
        msg.header.stamp = stamp
        msg.header.frame_id = WORLD_FRAME
        msg.pose = pose
        self._pose_pub.publish(msg)

    def _publish_odom(self, pose, twist, stamp):
        msg = Odometry()
        msg.header.stamp = stamp
        msg.header.frame_id = WORLD_FRAME
        msg.child_frame_id = ROBOT_FRAME
        msg.pose.pose = pose
        msg.twist.twist = twist
        self._odom_pub.publish(msg)

    def _broadcast_tf(self, pose, stamp):
        tf = TransformStamped()
        tf.header.stamp = stamp
        tf.header.frame_id = WORLD_FRAME
        tf.child_frame_id = ROBOT_FRAME
        tf.transform.translation.x = pose.position.x
        tf.transform.translation.y = pose.position.y
        tf.transform.translation.z = pose.position.z
        tf.transform.rotation = pose.orientation
        self._tf.sendTransform(tf)


def main(args=None):
    rclpy.init(args=args)
    node = GroundtruthPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
