#!/usr/bin/env python3
"""Put the simulated ROSbot back at its start pose with an empty map.

An Empty message on /sim/reset triggers it. Gazebo itself keeps running: a
world reset in Fortress also rewinds the clock, which the ROS nodes on
use_sim_time do not survive. Instead the robot is stopped, moved to its start
pose, the odometry is zeroed and SLAM is restarted through /slam/forget_map.

Usage: sim_reset.py [x y yaw]    start pose in the Gazebo world, default 0 -2 0
"""
import math
import subprocess
import sys
import threading
import time

import rclpy
from action_msgs.srv import CancelGoal
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from nav2_msgs.srv import ClearEntireCostmap
from rclpy.node import Node
from std_msgs.msg import Bool, Empty

WORLD = 'husarion_world'
MODEL = 'rosbot'
# slam_toolbox needs this long to come back before the costmaps are cleared;
# cleared earlier, they are filled again from the map it is about to drop.
SECONDS_FOR_SLAM_RESTART = 10.0


class SimReset(Node):
    def __init__(self, x, y, yaw):
        super().__init__('sim_reset')
        self.start_pose = (x, y, yaw)
        self.resetting = False
        self.cmd_vel = self.create_publisher(Twist, '/cmd_vel', 10)
        self.explore = self.create_publisher(Bool, '/explore/resume', 10)
        self.odometry = self.create_publisher(PoseWithCovarianceStamped, '/set_pose', 10)
        self.forget_map = self.create_publisher(Empty, '/slam/forget_map', 10)
        self.cancel_goal = self.create_client(CancelGoal, '/navigate_to_pose/_action/cancel_goal')
        self.clear_costmaps = [
            self.create_client(ClearEntireCostmap, '/global_costmap/clear_entirely_global_costmap'),
            self.create_client(ClearEntireCostmap, '/local_costmap/clear_entirely_local_costmap'),
        ]
        self.create_subscription(Empty, '/sim/reset', self.on_reset, 10)

    def on_reset(self, _msg):
        # A second press while a reset runs would move the robot mid-sequence.
        if self.resetting:
            return
        self.resetting = True
        threading.Thread(target=self.reset, daemon=True).start()

    def reset(self):
        try:
            self.get_logger().info('Resetting the simulation.')
            self.stop_robot()
            if not self.move_to_start():
                self.get_logger().error('Gazebo refused the pose; reset stopped.')
                return
            time.sleep(1.0)
            self.zero_odometry()
            self.forget_map.publish(Empty())
            time.sleep(SECONDS_FOR_SLAM_RESTART)
            for client in self.clear_costmaps:
                client.call_async(ClearEntireCostmap.Request())
            self.get_logger().info('Reset done: robot at start pose, map empty.')
        finally:
            self.resetting = False

    def stop_robot(self):
        self.explore.publish(Bool(data=False))
        # An empty request cancels every goal of the action server.
        self.cancel_goal.call_async(CancelGoal.Request())
        for _ in range(15):
            self.cmd_vel.publish(Twist())
            time.sleep(0.1)

    def move_to_start(self):
        x, y, yaw = self.start_pose
        request = (
            f'name: "{MODEL}", position: {{x: {x}, y: {y}, z: 0.0}}, '
            f'orientation: {{z: {math.sin(yaw / 2)}, w: {math.cos(yaw / 2)}}}'
        )
        result = subprocess.run(
            ['ign', 'service', '-s', f'/world/{WORLD}/set_pose',
             '--reqtype', 'ignition.msgs.Pose', '--reptype', 'ignition.msgs.Boolean',
             '--timeout', '5000', '--req', request],
            capture_output=True, text=True, check=False)
        return 'data: true' in result.stdout

    def zero_odometry(self):
        # The EKF fuses wheel velocities, not wheel positions, so it keeps the
        # pose it is given here and odom starts again at the start pose.
        pose = PoseWithCovarianceStamped()
        pose.header.frame_id = 'odom'
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.pose.orientation.w = 1.0
        self.odometry.publish(pose)


def main():
    args = rclpy.utilities.remove_ros_args(sys.argv)[1:]
    if len(args) not in (0, 3):
        sys.exit(__doc__)
    x, y, yaw = (float(a) for a in args) if args else (0.0, -2.0, 0.0)
    rclpy.init()
    node = SimReset(x, y, yaw)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()
