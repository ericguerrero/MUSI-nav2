#!/usr/bin/env python3
"""
Path Tracker Node - Publishes robot paths in both odom and map frames

Subscribes to /odom and /tf to track robot position in:
- odom frame (accumulates drift)
- map frame (corrected by localization)

Publishes Path messages for RViz visualization.
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped
import tf2_ros
from tf2_ros import TransformException


class PathTracker(Node):
    def __init__(self):
        super().__init__('path_tracker')

        # Publishers for path visualization
        self.odom_path_pub = self.create_publisher(Path, '/path_odom', 10)
        self.map_path_pub = self.create_publisher(Path, '/path_map', 10)

        # Path storage
        self.odom_path = Path()
        self.odom_path.header.frame_id = 'odom'

        self.map_path = Path()
        self.map_path.header.frame_id = 'map'

        # Subscribe to odometry
        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )

        # TF2 buffer and listener for map frame
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Timer for publishing paths (10 Hz)
        self.timer = self.create_timer(0.1, self.publish_paths)

        # Sample counter (only add point every N samples to avoid clutter)
        self.sample_counter = 0
        self.sample_interval = 5  # Add point every 0.5 seconds

        self.get_logger().info('Path Tracker Node Started')
        self.get_logger().info('Publishing:')
        self.get_logger().info('  - /path_odom (red in RViz)')
        self.get_logger().info('  - /path_map (green in RViz)')

    def odom_callback(self, msg: Odometry):
        """Store odometry path"""
        self.sample_counter += 1

        if self.sample_counter >= self.sample_interval:
            self.sample_counter = 0

            # Create pose from odometry
            pose = PoseStamped()
            pose.header = msg.header
            pose.pose = msg.pose.pose

            # Add to odom path
            self.odom_path.poses.append(pose)

            # Also try to get map frame position
            try:
                transform = self.tf_buffer.lookup_transform(
                    'map',
                    'base_link',
                    rclpy.time.Time(),
                    timeout=rclpy.duration.Duration(seconds=0.1)
                )

                # Create map frame pose
                map_pose = PoseStamped()
                map_pose.header.stamp = msg.header.stamp
                map_pose.header.frame_id = 'map'
                map_pose.pose.position.x = transform.transform.translation.x
                map_pose.pose.position.y = transform.transform.translation.y
                map_pose.pose.position.z = transform.transform.translation.z
                map_pose.pose.orientation = transform.transform.rotation

                # Add to map path
                self.map_path.poses.append(map_pose)

            except TransformException as ex:
                # TF not available yet, skip this sample
                pass

    def publish_paths(self):
        """Publish both paths for visualization"""
        if len(self.odom_path.poses) > 0:
            self.odom_path.header.stamp = self.get_clock().now().to_msg()
            self.odom_path_pub.publish(self.odom_path)

        if len(self.map_path.poses) > 0:
            self.map_path.header.stamp = self.get_clock().now().to_msg()
            self.map_path_pub.publish(self.map_path)


def main(args=None):
    rclpy.init(args=args)
    node = PathTracker()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
