#!/usr/bin/env python3
"""
Simple Odometry-Based Waypoint Follower
Exercise 3: Demonstrates odometry drift problem

This script navigates a square path using ONLY wheel odometry.
Students will observe that the robot does NOT return to start position!

Author: MUSI-nav2 Educational Project
License: MIT
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import math


class SimpleOdomWaypoint(Node):
    """
    Simple waypoint follower using only odometry (no map, no localization).

    This demonstrates the fundamental problem of odometry drift:
    - Robot tries to navigate a square path
    - Expected to return to (0, 0)
    - Actually ends up at (~0.3, ~-0.2) due to accumulated errors
    """

    def __init__(self):
        super().__init__('simple_odom_waypoint')

        # ROS2 Publishers and Subscribers
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10)

        # Waypoint sequence: Fixed map coordinates (2m x 2m square centered on origin)
        # These are absolute coordinates in the odom frame
        # Square from (-1.0, -1.0) to (1.0, 1.0)
        self.waypoints = [
            {'x': 1.0, 'y': -1.0, 'name': 'Point A - bottom right'},
            {'x': 1.0, 'y': 1.0, 'name': 'Point B - top right'},
            {'x': -1.0, 'y': 1.0, 'name': 'Point C - top left'},
            {'x': -1.0, 'y': -1.0, 'name': 'Point D - bottom left (return to start)'},
        ]

        # State variables
        self.current_waypoint_idx = 0
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0
        self.start_x = None  # Will record actual start position
        self.start_y = None

        # Control parameters
        self.goal_tolerance = 0.1  # 10cm - reached waypoint threshold
        self.linear_speed = 0.15   # m/s - slow and steady
        self.angular_speed = 0.4   # rad/s - turning speed
        self.angle_tolerance = 0.1  # rad (~5.7 degrees)

        # Gains for proportional control
        self.k_linear = 0.5   # Linear velocity gain
        self.k_angular = 1.0  # Angular velocity gain

        self.get_logger().info('='*60)
        self.get_logger().info('Simple Odometry Waypoint Follower Started')
        self.get_logger().info('='*60)
        self.get_logger().info('This script demonstrates ODOMETRY DRIFT')
        self.get_logger().info('Path: 2m × 2m square centered on origin')
        self.get_logger().info('Route: (1.0,-1.0) → (1.0,1.0) → (-1.0,1.0) → (-1.0,-1.0)')
        self.get_logger().info('Expected: Return to start position')
        self.get_logger().info('Reality: Will drift due to odometry errors')
        self.get_logger().info('='*60)
        self.get_logger().info(f'Total waypoints: {len(self.waypoints)}')

    def odom_callback(self, msg):
        """
        Callback for odometry messages.
        Updates current position and navigates to current waypoint.
        """
        # Record start position on first callback
        if self.start_x is None:
            self.start_x = msg.pose.pose.position.x
            self.start_y = msg.pose.pose.position.y
            self.get_logger().info(
                f'Start position recorded: ({self.start_x:.3f}, {self.start_y:.3f})')
            self.get_logger().info('Target waypoints (fixed map coordinates):')
            for i, wp in enumerate(self.waypoints, 1):
                self.get_logger().info(
                    f'  Waypoint {i}: {wp["name"]} → ({wp["x"]:.3f}, {wp["y"]:.3f})')

        # Extract position
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y

        # Convert quaternion to yaw (heading angle)
        orientation_q = msg.pose.pose.orientation
        siny_cosp = 2 * (orientation_q.w * orientation_q.z +
                        orientation_q.x * orientation_q.y)
        cosy_cosp = 1 - 2 * (orientation_q.y * orientation_q.y +
                            orientation_q.z * orientation_q.z)
        self.current_yaw = math.atan2(siny_cosp, cosy_cosp)

        # Navigate to current waypoint
        self.navigate_to_waypoint()

    def navigate_to_waypoint(self):
        """
        Main navigation logic using proportional control.

        Strategy:
        1. Calculate angle and distance to goal
        2. If angle error > threshold: rotate in place
        3. If angle aligned: move forward
        4. If within tolerance: proceed to next waypoint
        """
        # Check if all waypoints completed
        if self.current_waypoint_idx >= len(self.waypoints):
            self.complete_navigation()
            return

        # Get current goal waypoint
        goal = self.waypoints[self.current_waypoint_idx]
        goal_x = goal['x']
        goal_y = goal['y']

        # Calculate distance and angle to goal
        dx = goal_x - self.current_x
        dy = goal_y - self.current_y
        distance = math.sqrt(dx**2 + dy**2)
        angle_to_goal = math.atan2(dy, dx)
        angle_diff = self.normalize_angle(angle_to_goal - self.current_yaw)

        # Check if waypoint reached
        if distance < self.goal_tolerance:
            self.get_logger().info(
                f'✓ Waypoint {self.current_waypoint_idx + 1}/{len(self.waypoints)} ' +
                f'reached: {goal["name"]}')
            self.get_logger().info(
                f'  Position: ({self.current_x:.3f}, {self.current_y:.3f}), ' +
                f'Yaw: {math.degrees(self.current_yaw):.1f}°')
            self.current_waypoint_idx += 1
            return

        # Create velocity command
        cmd = Twist()

        # Proportional controller
        if abs(angle_diff) > self.angle_tolerance:
            # Need to rotate: stop forward motion, turn towards goal
            cmd.linear.x = 0.0
            cmd.angular.z = self.k_angular * self.clip(
                angle_diff, -self.angular_speed, self.angular_speed)
        else:
            # Aligned with goal: move forward with proportional speed
            cmd.linear.x = self.k_linear * min(distance, self.linear_speed)
            # Small angular correction while moving
            cmd.angular.z = 0.3 * angle_diff

        # Publish command
        self.cmd_pub.publish(cmd)

        # Debug logging (every 30 callbacks = ~1 second)
        if hasattr(self, '_debug_counter'):
            self._debug_counter += 1
            if self._debug_counter % 30 == 0:
                self.get_logger().info(
                    f'→ Waypoint {self.current_waypoint_idx + 1}: ' +
                    f'dist={distance:.2f}m, angle_err={math.degrees(angle_diff):.1f}°',
                    throttle_duration_sec=1.0)
        else:
            self._debug_counter = 0

    def complete_navigation(self):
        """
        Called when all waypoints are reached.
        Stops robot and displays drift analysis.
        """
        # Stop the robot
        self.stop_robot()

        # Calculate drift from start position
        final_x = self.current_x
        final_y = self.current_y
        drift_x = final_x - self.start_x
        drift_y = final_y - self.start_y
        total_drift = math.sqrt(drift_x**2 + drift_y**2)

        # Display results
        self.get_logger().info('='*60)
        self.get_logger().info('ALL WAYPOINTS COMPLETED!')
        self.get_logger().info('='*60)
        self.get_logger().info('DRIFT ANALYSIS:')
        self.get_logger().info(f'  Start position:    ({self.start_x:.3f}, {self.start_y:.3f})')
        self.get_logger().info(f'  Expected final:    ({self.start_x:.3f}, {self.start_y:.3f})')
        self.get_logger().info(f'  Actual final:      ({final_x:.3f}, {final_y:.3f})')
        self.get_logger().info(f'  Drift (X, Y):      ({drift_x:.3f}, {drift_y:.3f})')
        self.get_logger().info(f'  Total drift:       {total_drift:.3f} meters')
        self.get_logger().info('='*60)
        self.get_logger().info('WHY DID THIS HAPPEN?')
        self.get_logger().info('  - Wheel slippage on smooth floor')
        self.get_logger().info('  - Odometry measurement noise')
        self.get_logger().info('  - Accumulated rounding errors')
        self.get_logger().info('='*60)
        self.get_logger().info('SOLUTION: We need SLAM and Localization!')
        self.get_logger().info('  → Exercise 4: SLAM Toolbox mapping')
        self.get_logger().info('  → Exercise 6: Sensor fusion (EKF)')
        self.get_logger().info('  → Exercise 7: SLAM Toolbox localization')
        self.get_logger().info('='*60)

        # Shutdown node
        rclpy.shutdown()

    def stop_robot(self):
        """Send zero velocity command to stop the robot."""
        cmd = Twist()
        cmd.linear.x = 0.0
        cmd.angular.z = 0.0
        self.cmd_pub.publish(cmd)
        self.get_logger().info('Robot stopped.')

    @staticmethod
    def normalize_angle(angle):
        """
        Normalize angle to [-pi, pi] range.

        Args:
            angle (float): Angle in radians

        Returns:
            float: Normalized angle in [-pi, pi]
        """
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle

    @staticmethod
    def clip(value, min_val, max_val):
        """
        Clip value to [min_val, max_val] range.

        Args:
            value (float): Value to clip
            min_val (float): Minimum allowed value
            max_val (float): Maximum allowed value

        Returns:
            float: Clipped value
        """
        return max(min_val, min(value, max_val))


def main(args=None):
    """Main entry point for the waypoint follower node."""
    rclpy.init(args=args)
    node = SimpleOdomWaypoint()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Interrupted by user (Ctrl+C)')
        node.stop_robot()
    except Exception as e:
        node.get_logger().error(f'Unexpected error: {e}')
        node.stop_robot()
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
