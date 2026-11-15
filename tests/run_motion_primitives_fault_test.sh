#!/bin/bash
# Run the motion primitives fault test inside the image, against a fake robot.
#
# No network at all: the node and the fake robot talk over loopback, and no
# /cmd_vel can leave this container for the lab robot on the same switch.
set -e
REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
exec docker run --rm \
    --network none \
    -e ROS_LOCALHOST_ONLY=1 \
    -e ROS_DOMAIN_ID=77 \
    -v "${REPO}/ros2_ws:/ros2_ws:ro" \
    -v "${REPO}/tests:/tests:ro" \
    nav2_student:humble \
    bash /tests/test_motion_primitives_faults.sh
