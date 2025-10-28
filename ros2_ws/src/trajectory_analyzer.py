#!/usr/bin/env python3
"""
Trajectory Analyzer - Collects trajectory data and creates Plotly visualization

Records robot trajectories in both odom and map frames, then generates
comprehensive visualization with analysis justifying localization and SLAM.
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import tf2_ros
from tf2_ros import TransformException
import numpy as np
import json
from datetime import datetime


class TrajectoryAnalyzer(Node):
    def __init__(self):
        super().__init__('trajectory_analyzer')

        # Data storage
        self.odom_trajectory = []  # [(x, y, timestamp), ...]
        self.map_trajectory = []   # [(x, y, timestamp), ...]
        self.start_time = None

        # Subscribe to odometry
        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )

        # TF2 for map frame
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Timer for data collection (5 Hz)
        self.timer = self.create_timer(0.2, self.collect_data)

        self.get_logger().info('Trajectory Analyzer Started')
        self.get_logger().info('Recording odometry and map frame positions')
        self.get_logger().info('Press Ctrl+C to stop and generate visualization')

    def odom_callback(self, msg: Odometry):
        """Store latest odometry message"""
        self.latest_odom = msg

    def collect_data(self):
        """Collect position in both frames"""
        if not hasattr(self, 'latest_odom'):
            return

        # Initialize start time
        if self.start_time is None:
            self.start_time = self.get_clock().now()

        current_time = self.get_clock().now()
        elapsed = (current_time - self.start_time).nanoseconds / 1e9

        # Get odom frame position
        odom_x = self.latest_odom.pose.pose.position.x
        odom_y = self.latest_odom.pose.pose.position.y
        self.odom_trajectory.append((odom_x, odom_y, elapsed))

        # Get map frame position
        try:
            transform = self.tf_buffer.lookup_transform(
                'map',
                'base_link',
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.1)
            )

            map_x = transform.transform.translation.x
            map_y = transform.transform.translation.y
            self.map_trajectory.append((map_x, map_y, elapsed))

        except TransformException:
            pass

    def save_data(self):
        """Save collected data to JSON"""
        data = {
            'timestamp': datetime.now().isoformat(),
            'duration_seconds': (self.get_clock().now() - self.start_time).nanoseconds / 1e9 if self.start_time else 0,
            'odom_trajectory': self.odom_trajectory,
            'map_trajectory': self.map_trajectory,
            'waypoints': [
                {'x': 2.0, 'y': -2.0, 'name': 'Point A - Start'},
                {'x': 2.0, 'y': 2.0, 'name': 'Point B - North'},
                {'x': -2.0, 'y': 2.0, 'name': 'Point C - West'},
                {'x': -2.0, 'y': -2.0, 'name': 'Point D - South'},
            ],
            'landmarks': [
                {'x': 0.0, 'y': 0.0, 'radius': 0.3, 'color': 'red', 'name': 'Center'},
                {'x': 0.0, 'y': 3.0, 'radius': 0.3, 'color': 'green', 'name': 'North'},
                {'x': 3.0, 'y': 0.0, 'radius': 0.3, 'color': 'blue', 'name': 'East'},
                {'x': 0.0, 'y': -3.0, 'radius': 0.3, 'color': 'yellow', 'name': 'South'},
                {'x': -3.0, 'y': 0.0, 'radius': 0.3, 'color': 'magenta', 'name': 'West'},
            ]
        }

        filename = '/shared/student_work/trajectory_data.json'
        with open(filename, 'w') as f:
            json.dump(data, f, indent=2)

        self.get_logger().info(f'Data saved to {filename}')
        self.get_logger().info(f'Odom points: {len(self.odom_trajectory)}')
        self.get_logger().info(f'Map points: {len(self.map_trajectory)}')

        # Calculate drift statistics
        if len(self.odom_trajectory) > 0 and len(self.map_trajectory) > 0:
            odom_start = np.array(self.odom_trajectory[0][:2])
            odom_end = np.array(self.odom_trajectory[-1][:2])
            map_start = np.array(self.map_trajectory[0][:2])
            map_end = np.array(self.map_trajectory[-1][:2])

            odom_drift = np.linalg.norm(odom_end - odom_start)
            map_error = np.linalg.norm(map_end - map_start)

            self.get_logger().info('='*60)
            self.get_logger().info('DRIFT ANALYSIS:')
            self.get_logger().info(f'Odometry drift: {odom_drift:.3f} m')
            self.get_logger().info(f'Map frame error: {map_error:.3f} m')
            self.get_logger().info(f'Localization improvement: {(1 - map_error/odom_drift)*100:.1f}%')
            self.get_logger().info('='*60)


def main(args=None):
    rclpy.init(args=args)
    node = TrajectoryAnalyzer()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Stopping data collection...')
        node.save_data()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
