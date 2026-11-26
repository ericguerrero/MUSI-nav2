#!/bin/bash
# Run the Ackermann robot simulation test inside the image.
#
# Deliberately not --network host: the lab ROSbot 2R runs on ROS_DOMAIN_ID 0
# and also listens on /cmd_vel. The default bridge network plus
# ROS_LOCALHOST_ONLY keeps every DDS packet inside this container.
set -e
REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
exec docker run --rm \
    --name "ackb_sim_test_$$" \
    -e ROS_LOCALHOST_ONLY=1 \
    -e ROS_DOMAIN_ID=78 \
    -e RADIUS_TOLERANCE \
    -v "${REPO}/ros2_ws:/ros2_ws:ro" \
    -v "${REPO}/tests:/tests:ro" \
    nav2_student:humble \
    bash /tests/test_ackermann_sim.sh
