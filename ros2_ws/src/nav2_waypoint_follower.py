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
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from gazebo_msgs.msg import ModelStates
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

        # AMCL tracking (corrected map frame via /amcl_pose)
        self.amcl_sub = self.create_subscription(
            PoseWithCovarianceStamped, '/amcl_pose', self.amcl_callback, 10)
        self.map_positions: List[Tuple[float, float, float]] = []
        self.start_map: Optional[Tuple[float, float]] = None

        # Gazebo groundtruth tracking (actual position)
        self.groundtruth_sub = self.create_subscription(
            ModelStates, '/model_states', self.groundtruth_callback, 10)
        self.groundtruth_positions: List[Tuple[float, float, float]] = []
        self.start_groundtruth: Optional[Tuple[float, float]] = None
        self.robot_model_name = 'turtlebot3_waffle'

        # Waypoints (4m × 4m square)
        # Robot spawns at (2.0, -2.0) - bottom right corner
        # Navigate clockwise: north → northwest → southwest → back to start
        self.waypoints = [
            {'x': 2.0, 'y': 2.0, 'name': 'Point A - northeast corner'},
            {'x': -2.0, 'y': 2.0, 'name': 'Point B - northwest corner'},
            {'x': -2.0, 'y': -2.0, 'name': 'Point C - southwest corner'},
            {'x': 2.0, 'y': -2.0, 'name': 'Point D - return to start (southeast)'},
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

    def amcl_callback(self, msg: PoseWithCovarianceStamped):
        """
        Track AMCL localized position (corrected by Nav2).

        Args:
            msg: PoseWithCovarianceStamped message from /amcl_pose topic
        """
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        timestamp = self.get_clock().now().nanoseconds / 1e9

        self.map_positions.append((x, y, timestamp))

        if self.start_map is None:
            self.start_map = (x, y)
            self.get_logger().info(f'Start map position: ({x:.3f}, {y:.3f})')

    def groundtruth_callback(self, msg: ModelStates):
        """
        Track Gazebo groundtruth position (actual simulator position).

        Args:
            msg: ModelStates message from Gazebo /model_states topic
        """
        try:
            # Find robot model in model states
            robot_idx = msg.name.index(self.robot_model_name)
            x = msg.pose[robot_idx].position.x
            y = msg.pose[robot_idx].position.y
            timestamp = self.get_clock().now().nanoseconds / 1e9

            self.groundtruth_positions.append((x, y, timestamp))

            if self.start_groundtruth is None:
                self.start_groundtruth = (x, y)
                self.get_logger().info(f'Start groundtruth position: ({x:.3f}, {y:.3f})')

        except (ValueError, IndexError) as e:
            self.get_logger().debug(f'Robot model not found in model_states: {e}')

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

        # Set initial pose with realistic small error
        # Robot spawns at (2.0, -2.0) but we tell AMCL it's at (2.1, -1.9)
        # This creates ~0.14m initial error (realistic GPS/manual placement error)
        # AMCL will correct this, demonstrating localization working
        initial_pose = self.create_pose(2.1, -1.9)
        self.navigator.setInitialPose(initial_pose)
        self.get_logger().info('Initial pose set: (2.1, -1.9) - small 0.14m error from spawn (2.0, -2.0)')

        # Wait a moment for localization to settle and start collecting data
        self.get_logger().info('Waiting 2 seconds for localization...')
        for _ in range(20):
            rclpy.spin_once(self, timeout_sec=0.1)

        # Convert waypoints to PoseStamped messages
        goal_poses = []
        for wp in self.waypoints:
            pose = self.create_pose(wp['x'], wp['y'])
            goal_poses.append(pose)
            self.get_logger().info(f'Waypoint: {wp["name"]} → ({wp["x"]:.1f}, {wp["y"]:.1f})')

        # Start waypoint following
        self.get_logger().info('='*60)
        self.get_logger().info('Starting waypoint navigation...')
        self.get_logger().info(f'Data collection started: {len(self.odom_positions)} odom, '
                              f'{len(self.map_positions)} map, {len(self.groundtruth_positions)} groundtruth positions')
        self.get_logger().info('='*60)
        self.navigator.followWaypoints(goal_poses)

        # Monitor progress and track positions
        waypoint_idx = 0
        loop_count = 0
        last_report = 0

        while not self.navigator.isTaskComplete():
            # Spin to process callbacks (odom, amcl, groundtruth updates)
            rclpy.spin_once(self, timeout_sec=0.1)
            loop_count += 1

            # Periodic data collection status report (every 50 loops = ~5 seconds)
            if loop_count - last_report >= 50:
                last_report = loop_count
                self.get_logger().info(f'[Data collection] Odom: {len(self.odom_positions)}, '
                                      f'Map: {len(self.map_positions)}, '
                                      f'GT: {len(self.groundtruth_positions)} samples')

            # Get feedback (current waypoint being followed)
            feedback = self.navigator.getFeedback()
            if feedback and hasattr(feedback, 'current_waypoint'):
                if feedback.current_waypoint != waypoint_idx:
                    waypoint_idx = feedback.current_waypoint
                    if waypoint_idx < len(self.waypoints):
                        self.get_logger().info(
                            f'→ Navigating to waypoint {waypoint_idx + 1}/4: '
                            f'{self.waypoints[waypoint_idx]["name"]}')
                        # Report data collection at each waypoint
                        self.get_logger().info(
                            f'  [Samples collected] Odom: {len(self.odom_positions)}, '
                            f'Map: {len(self.map_positions)}, GT: {len(self.groundtruth_positions)}')

        # Check result
        result = self.navigator.getResult()
        self.get_logger().info('='*60)
        if result == TaskResult.SUCCEEDED:
            self.get_logger().info('✓ ALL WAYPOINTS COMPLETED SUCCESSFULLY!')
            self.calculate_drift()
        elif result == TaskResult.CANCELED:
            self.get_logger().warn('Navigation was canceled')
            self.get_logger().info('Calculating drift from partial data...')
            self.calculate_drift()
        elif result == TaskResult.FAILED:
            self.get_logger().error('Navigation failed')
            self.get_logger().info('Calculating drift from partial data...')
            self.calculate_drift()
        else:
            self.get_logger().error(f'Unknown result: {result}')
            self.calculate_drift()

    def calculate_drift(self):
        """
        Calculate and display drift comparison between odom, map, and groundtruth.

        This demonstrates that:
        1. Odometry drifts from groundtruth (wheel encoders accumulate error)
        2. Nav2 map frame corrects drift (stays close to groundtruth)
        3. Groundtruth is the actual position (from Gazebo simulator)
        """
        # Debug: Log data collection status
        self.get_logger().info(f'Data collection status:')
        self.get_logger().info(f'  Odom positions: {len(self.odom_positions)}')
        self.get_logger().info(f'  Map positions: {len(self.map_positions)}')
        self.get_logger().info(f'  Groundtruth positions: {len(self.groundtruth_positions)}')
        self.get_logger().info(f'  Start odom: {self.start_odom}')
        self.get_logger().info(f'  Start map: {self.start_map}')
        self.get_logger().info(f'  Start groundtruth: {self.start_groundtruth}')

        if not self.odom_positions or not self.map_positions:
            self.get_logger().warn('Insufficient position data for drift calculation')
            self.get_logger().warn('This usually means navigation did not run long enough to collect data')
            return

        if self.start_odom is None or self.start_map is None:
            self.get_logger().warn('Start positions not recorded')
            return

        # Final positions
        final_odom = self.odom_positions[-1][:2]  # (x, y)
        final_map = self.map_positions[-1][:2]

        # Display comprehensive results
        self.get_logger().info('='*60)
        self.get_logger().info('DRIFT ANALYSIS (Odom vs Map vs Groundtruth):')
        self.get_logger().info('='*60)

        # Groundtruth (if available)
        if self.groundtruth_positions and self.start_groundtruth:
            final_groundtruth = self.groundtruth_positions[-1][:2]

            self.get_logger().info('GROUNDTRUTH (Gazebo simulator - ACTUAL POSITION):')
            self.get_logger().info(f'  Start position:  ({self.start_groundtruth[0]:7.3f}, {self.start_groundtruth[1]:7.3f})')
            self.get_logger().info(f'  Final position:  ({final_groundtruth[0]:7.3f}, {final_groundtruth[1]:7.3f})')
            gt_drift_x = final_groundtruth[0] - self.start_groundtruth[0]
            gt_drift_y = final_groundtruth[1] - self.start_groundtruth[1]
            gt_drift = math.sqrt(gt_drift_x**2 + gt_drift_y**2)
            self.get_logger().info(f'  Drift from start: {gt_drift:.3f} meters (should be ~0)')
            self.get_logger().info('')

            # Calculate errors relative to groundtruth
            odom_error_x = final_odom[0] - final_groundtruth[0]
            odom_error_y = final_odom[1] - final_groundtruth[1]
            odom_error = math.sqrt(odom_error_x**2 + odom_error_y**2)

            map_error_x = final_map[0] - final_groundtruth[0]
            map_error_y = final_map[1] - final_groundtruth[1]
            map_error = math.sqrt(map_error_x**2 + map_error_y**2)

            self.get_logger().info('ODOMETRY FRAME (wheel encoders only - DRIFTS):')
            self.get_logger().info(f'  Final position:     ({final_odom[0]:7.3f}, {final_odom[1]:7.3f})')
            self.get_logger().info(f'  Error vs truth:     ({odom_error_x:7.3f}, {odom_error_y:7.3f})')
            self.get_logger().info(f'  Total error:        {odom_error:.3f} meters')

            self.get_logger().info('')
            self.get_logger().info('MAP FRAME (Nav2 localization - CORRECTED):')
            self.get_logger().info(f'  Final position:     ({final_map[0]:7.3f}, {final_map[1]:7.3f})')
            self.get_logger().info(f'  Error vs truth:     ({map_error_x:7.3f}, {map_error_y:7.3f})')
            self.get_logger().info(f'  Total error:        {map_error:.3f} meters')

            self.get_logger().info('='*60)
            self.get_logger().info('KEY INSIGHTS:')
            self.get_logger().info(f'  • Odometry error:  {odom_error:.3f}m (vs groundtruth)')
            self.get_logger().info(f'  • Nav2 accuracy:   {map_error:.3f}m (vs groundtruth)')

            if odom_error > 0.01:
                improvement = ((odom_error - map_error) / odom_error) * 100
                self.get_logger().info(f'  • Improvement:     {improvement:.1f}% reduction in error')

        else:
            # Fallback: odom vs map comparison (no groundtruth)
            odom_drift_x = final_odom[0] - self.start_odom[0]
            odom_drift_y = final_odom[1] - self.start_odom[1]
            odom_drift = math.sqrt(odom_drift_x**2 + odom_drift_y**2)

            map_error_x = final_map[0] - self.start_map[0]
            map_error_y = final_map[1] - self.start_map[1]
            map_error = math.sqrt(map_error_x**2 + map_error_y**2)

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
        self.get_logger().info(f'Recorded {len(self.odom_positions)} odom, '
                              f'{len(self.map_positions)} map, '
                              f'{len(self.groundtruth_positions)} groundtruth positions')


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
