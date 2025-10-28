#!/usr/bin/env python3
"""
Nav2 Waypoint Follower with Drift Tracking
Exercise 3b: Demonstrates Nav2 collision avoidance while measuring drift

This script uses Nav2's BasicNavigator API to safely navigate a 2m × 2m square
while tracking positions in both odom frame (drifting) and map frame (corrected).

Author: MUSI-nav2 Educational Project
License: MIT
"""

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
import tf2_ros
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException
import math
from typing import List, Tuple, Optional


class Nav2WaypointFollower(Node):
    """
    Navigate waypoints using Nav2 while tracking drift.

    Compares odometry frame (drifting) with map frame (corrected by Nav2)
    to demonstrate that drift still exists but Nav2 compensates for it.
    """

    def __init__(self):
        super().__init__('nav2_waypoint_follower')

        # Initialize BasicNavigator
        self.navigator = BasicNavigator()

        # Odometry tracking (drifting frame)
        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10)
        self.odom_positions: List[Tuple[float, float, float]] = []
        self.start_odom: Optional[Tuple[float, float]] = None

        # TF tracking (corrected map frame)
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.map_positions: List[Tuple[float, float, float]] = []
        self.start_map: Optional[Tuple[float, float]] = None

        # Waypoints (4m × 4m square - centered at origin in empty world)
        self.waypoints = [
            {'x': 2.0, 'y': -2.0, 'name': 'Point A - bottom right'},
            {'x': 2.0, 'y': 2.0, 'name': 'Point B - top right'},
            {'x': -2.0, 'y': 2.0, 'name': 'Point C - top left'},
            {'x': -2.0, 'y': -2.0, 'name': 'Point D - bottom left (return)'},
        ]

        self.get_logger().info('='*60)
        self.get_logger().info('Nav2 Waypoint Follower with Drift Tracking')
        self.get_logger().info('='*60)
        self.get_logger().info('Path: 1m × 1m square centered on origin')
        self.get_logger().info('Waypoints: 4 corners (safe within TurtleBot3 world)')
        self.get_logger().info('Navigation: Nav2 stack (collision avoidance enabled)')
        self.get_logger().info('Tracking: Both odom and map frames for drift comparison')
        self.get_logger().info('='*60)

    def create_pose(self, x: float, y: float) -> PoseStamped:
        """
        Convert waypoint coordinates to PoseStamped message.

        Args:
            x: X coordinate in map frame (meters)
            y: Y coordinate in map frame (meters)

        Returns:
            PoseStamped message for Nav2
        """
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.navigator.get_clock().now().to_msg()
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = 0.0
        # Orientation: facing forward (no rotation)
        pose.pose.orientation.x = 0.0
        pose.pose.orientation.y = 0.0
        pose.pose.orientation.z = 0.0
        pose.pose.orientation.w = 1.0
        return pose

    def odom_callback(self, msg: Odometry):
        """
        Track odometry frame positions (drifting reference).

        Args:
            msg: Odometry message from /odom topic
        """
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        timestamp = self.get_clock().now().nanoseconds / 1e9

        self.odom_positions.append((x, y, timestamp))

        if self.start_odom is None:
            self.start_odom = (x, y)
            self.get_logger().info(f'Start odom position: ({x:.3f}, {y:.3f})')

    def get_map_position(self) -> Optional[Tuple[float, float]]:
        """
        Get current position in map frame via TF lookup.

        Returns:
            Tuple of (x, y) in map frame, or None if lookup fails
        """
        try:
            # Lookup transform from map to base_link
            trans = self.tf_buffer.lookup_transform(
                'map',
                'base_link',
                rclpy.time.Time(),
                timeout=Duration(seconds=0.1))

            x = trans.transform.translation.x
            y = trans.transform.translation.y
            return (x, y)

        except (LookupException, ConnectivityException, ExtrapolationException) as e:
            self.get_logger().debug(f'TF lookup failed: {e}')
            return None

    def navigate_waypoints(self):
        """
        Main navigation logic using Nav2 BasicNavigator.

        Process:
        1. Wait for Nav2 to be ready
        2. Set initial pose
        3. Send waypoint goals
        4. Monitor navigation progress
        5. Calculate and display drift
        """
        self.get_logger().info('Waiting for Nav2 to become active...')
        self.navigator.waitUntilNav2Active()
        self.get_logger().info('Nav2 is active!')

        # Set initial pose (robot spawns at 2, -2)
        initial_pose = self.create_pose(2.0, -2.0)
        self.navigator.setInitialPose(initial_pose)
        self.get_logger().info('Initial pose set: (2.0, -2.0) - matching spawn location')

        # Wait a moment for localization to settle
        self.get_logger().info('Waiting 2 seconds for localization...')
        rclpy.spin_once(self, timeout_sec=2.0)

        # Convert waypoints to PoseStamped messages
        goal_poses = []
        for wp in self.waypoints:
            pose = self.create_pose(wp['x'], wp['y'])
            goal_poses.append(pose)
            self.get_logger().info(f'Waypoint: {wp["name"]} → ({wp["x"]:.1f}, {wp["y"]:.1f})')

        # Start waypoint following
        self.get_logger().info('='*60)
        self.get_logger().info('Starting waypoint navigation...')
        self.get_logger().info('='*60)
        self.navigator.followWaypoints(goal_poses)

        # Monitor progress and track positions
        waypoint_idx = 0
        while not self.navigator.isTaskComplete():
            # Spin to process callbacks (odom updates)
            rclpy.spin_once(self, timeout_sec=0.1)

            # Record map position via TF
            map_pos = self.get_map_position()
            if map_pos:
                timestamp = self.get_clock().now().nanoseconds / 1e9
                self.map_positions.append((map_pos[0], map_pos[1], timestamp))

                if self.start_map is None:
                    self.start_map = map_pos
                    self.get_logger().info(f'Start map position: ({map_pos[0]:.3f}, {map_pos[1]:.3f})')

            # Get feedback (current waypoint being followed)
            feedback = self.navigator.getFeedback()
            if feedback and hasattr(feedback, 'current_waypoint'):
                if feedback.current_waypoint != waypoint_idx:
                    waypoint_idx = feedback.current_waypoint
                    if waypoint_idx < len(self.waypoints):
                        self.get_logger().info(
                            f'→ Navigating to waypoint {waypoint_idx + 1}/4: '
                            f'{self.waypoints[waypoint_idx]["name"]}')

        # Check result
        result = self.navigator.getResult()
        self.get_logger().info('='*60)
        if result == TaskResult.SUCCEEDED:
            self.get_logger().info('✓ ALL WAYPOINTS COMPLETED SUCCESSFULLY!')
            self.calculate_drift()
        elif result == TaskResult.CANCELED:
            self.get_logger().warn('Navigation was canceled')
        elif result == TaskResult.FAILED:
            self.get_logger().error('Navigation failed')
        else:
            self.get_logger().error(f'Unknown result: {result}')

    def calculate_drift(self):
        """
        Calculate and display drift comparison between odom and map frames.

        This demonstrates that:
        1. Odometry still drifts (odom frame != start position)
        2. Nav2 corrects it (map frame ≈ start position)
        """
        if not self.odom_positions or not self.map_positions:
            self.get_logger().warn('Insufficient position data for drift calculation')
            return

        if self.start_odom is None or self.start_map is None:
            self.get_logger().warn('Start positions not recorded')
            return

        # Final positions
        final_odom = self.odom_positions[-1][:2]  # (x, y)
        final_map = self.map_positions[-1][:2]

        # Drift calculations
        odom_drift_x = final_odom[0] - self.start_odom[0]
        odom_drift_y = final_odom[1] - self.start_odom[1]
        odom_drift = math.sqrt(odom_drift_x**2 + odom_drift_y**2)

        map_error_x = final_map[0] - self.start_map[0]
        map_error_y = final_map[1] - self.start_map[1]
        map_error = math.sqrt(map_error_x**2 + map_error_y**2)

        # Display comprehensive results
        self.get_logger().info('='*60)
        self.get_logger().info('DRIFT ANALYSIS (Odom vs Map Frame):')
        self.get_logger().info('='*60)

        self.get_logger().info('ODOMETRY FRAME (wheel encoders only - DRIFTS):')
        self.get_logger().info(f'  Start position:  ({self.start_odom[0]:7.3f}, {self.start_odom[1]:7.3f})')
        self.get_logger().info(f'  Final position:  ({final_odom[0]:7.3f}, {final_odom[1]:7.3f})')
        self.get_logger().info(f'  Drift vector:    ({odom_drift_x:7.3f}, {odom_drift_y:7.3f})')
        self.get_logger().info(f'  Total drift:     {odom_drift:.3f} meters')

        self.get_logger().info('')
        self.get_logger().info('MAP FRAME (Nav2 localization - CORRECTED):')
        self.get_logger().info(f'  Start position:  ({self.start_map[0]:7.3f}, {self.start_map[1]:7.3f})')
        self.get_logger().info(f'  Final position:  ({final_map[0]:7.3f}, {final_map[1]:7.3f})')
        self.get_logger().info(f'  Error vector:    ({map_error_x:7.3f}, {map_error_y:7.3f})')
        self.get_logger().info(f'  Total error:     {map_error:.3f} meters')

        self.get_logger().info('='*60)
        self.get_logger().info('KEY INSIGHTS:')
        self.get_logger().info(f'  • Odometry drift: {odom_drift:.3f}m (accumulated error)')
        self.get_logger().info(f'  • Nav2 accuracy:  {map_error:.3f}m (corrected by localization)')

        if odom_drift > 0.1:
            improvement = ((odom_drift - map_error) / odom_drift) * 100
            self.get_logger().info(f'  • Improvement:    {improvement:.1f}% reduction in error')

        self.get_logger().info('')
        self.get_logger().info('WHY THIS MATTERS:')
        self.get_logger().info('  - Odometry ALONE accumulates error (no correction)')
        self.get_logger().info('  - Nav2 uses localization to CORRECT drift continuously')
        self.get_logger().info('  - This is why SLAM and localization are essential!')
        self.get_logger().info('='*60)

        # Log position sample counts
        self.get_logger().info(f'Recorded {len(self.odom_positions)} odom positions, '
                              f'{len(self.map_positions)} map positions')


def main(args=None):
    """Main entry point for Nav2 waypoint follower."""
    rclpy.init(args=args)
    node = Nav2WaypointFollower()

    try:
        node.navigate_waypoints()
    except KeyboardInterrupt:
        node.get_logger().info('Interrupted by user (Ctrl+C)')
    except Exception as e:
        node.get_logger().error(f'Unexpected error: {e}')
        import traceback
        traceback.print_exc()
    finally:
        node.get_logger().info('Shutting down...')
        node.navigator.lifecycleShutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
