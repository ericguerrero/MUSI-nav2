#!/usr/bin/env python3
"""Republish a laser scan with no-return rays turned into long readings.

slam_toolbox ignores a ray with no return (inf), so open space in front of the
robot stays unknown and frontier exploration never clears it. A reading just
under range_max is above slam_toolbox's max_laser_range, which makes it mark
the ray free up to max_laser_range without placing an obstacle at the end.

Only valid where no return means nothing is there. On the real robot glass,
dark surfaces and sunlight also give no return.
"""
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan

# Distance below range_max given to a no-return ray, in metres.
MARGIN = 0.05


class ScanFreeSpace(Node):
    def __init__(self):
        super().__init__('scan_free_space')
        self.publisher = self.create_publisher(LaserScan, '/scan_free_space', qos_profile_sensor_data)
        self.create_subscription(LaserScan, '/scan_filtered', self.relay, qos_profile_sensor_data)

    def relay(self, scan):
        far = scan.range_max - MARGIN
        scan.ranges = [far if math.isinf(r) and r > 0 else r for r in scan.ranges]
        self.publisher.publish(scan)


def main():
    rclpy.init()
    try:
        rclpy.spin(ScanFreeSpace())
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()
