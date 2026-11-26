#!/bin/bash
# Run the ROSbot simulation contract test inside the image.
#
# Deliberately not --network host: the lab ROSbot 2R runs on ROS_DOMAIN_ID 0
# with no namespace, and a simulation on the same network would exchange
# /cmd_vel and /odometry/filtered with it. The default bridge network plus
# ROS_LOCALHOST_ONLY keeps every DDS packet inside this container.
set -e
REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
exec docker run --rm \
    -e ROS_LOCALHOST_ONLY=1 \
    -e ROS_DOMAIN_ID=77 \
    -e MIN_ODOM_HZ \
    -e MIN_RTF \
    -e WORLD \
    -v "${REPO}/ros2_ws:/ros2_ws:ro" \
    -v "${REPO}/tests:/tests:ro" \
    nav2_student:humble \
    bash /tests/test_rosbot_sim_contract.sh
