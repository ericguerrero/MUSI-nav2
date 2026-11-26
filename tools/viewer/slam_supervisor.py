#!/usr/bin/env python3
"""Run slam_toolbox and restart it on request, which forgets the map.

Humble's slam_toolbox has no reset service, so the only way to drop the pose
graph is a fresh process. An Empty message on /slam/forget_map triggers it.

Usage: slam_supervisor.py <command that starts slam_toolbox ...>
"""
import os
import signal
import subprocess
import sys
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Empty

# A double click on the button must not restart SLAM twice.
MIN_SECONDS_BETWEEN_RESTARTS = 5.0


class SlamSupervisor(Node):
    def __init__(self, command):
        super().__init__('slam_supervisor')
        self.command = command
        self.last_restart = 0.0
        self.explore_pub = self.create_publisher(Bool, '/explore/resume', 10)
        self.create_subscription(Empty, '/slam/forget_map', self.forget, 10)
        self.process = self.start_slam()

    def start_slam(self):
        # Own process group: `ros2 run` does not pass a signal on to the node
        # it started, so a signal sent to it alone leaves slam_toolbox running
        # and publishing the map that was meant to be forgotten.
        return subprocess.Popen(self.command, start_new_session=True)

    def signal_slam(self, signum):
        try:
            os.killpg(self.process.pid, signum)
        except ProcessLookupError:
            pass

    def stop_slam(self):
        self.signal_slam(signal.SIGINT)
        try:
            self.process.wait(timeout=10)
        except subprocess.TimeoutExpired:
            self.signal_slam(signal.SIGKILL)
            self.process.wait()

    def forget(self, _msg):
        if time.monotonic() - self.last_restart < MIN_SECONDS_BETWEEN_RESTARTS:
            return
        self.get_logger().info('Forgetting the map: restarting slam_toolbox.')
        # Goals are expressed in the map frame, which a fresh slam_toolbox
        # realigns with the odometry frame. Stopping exploration cancels them.
        self.explore_pub.publish(Bool(data=False))
        self.stop_slam()
        self.process = self.start_slam()
        self.last_restart = time.monotonic()


def main():
    if len(sys.argv) < 2:
        sys.exit(__doc__)
    rclpy.init()
    node = SlamSupervisor(sys.argv[1:])
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop_slam()


if __name__ == '__main__':
    main()
