#!/bin/bash
# Run the motion primitives simulation test inside the image.
#
# Deliberately not --network host: the lab ROSbot 2R runs on ROS_DOMAIN_ID 0
# with no namespace, and a simulation on the same network would exchange
# /cmd_vel and /odometry/filtered with it. The default bridge network plus
# ROS_LOCALHOST_ONLY keeps every DDS packet inside this container, so this
# test can run while the real robot is switched on next to it.
set -e
REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
exec docker run --rm \
    -e ROS_LOCALHOST_ONLY=1 \
    -e ROS_DOMAIN_ID=77 \
    -e EXPECT_FORWARD_M \
    -v "${REPO}/ros2_ws:/ros2_ws:ro" \
    -v "${REPO}/tests:/tests:ro" \
    nav2_student:humble \
    bash /tests/test_motion_primitives_sim.sh
