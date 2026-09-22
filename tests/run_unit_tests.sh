#!/bin/bash
# Run the Python unit tests inside the ROS2 container, where rclpy and
# gazebo_msgs are available. The host has pytest but no ROS.
set -e
REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
exec docker run --rm \
    -v "${REPO}/ros2_ws:/ros2_ws:ro" \
    -v "${REPO}/tests:/tests:ro" \
    nav2_student:humble \
    bash -lc "python3 -m pytest /tests/ -v -p no:cacheprovider ${*}"
