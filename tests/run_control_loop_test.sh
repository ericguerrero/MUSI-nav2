#!/bin/bash
# Run the end-to-end control loop test inside the container.
set -e
REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
exec docker run --rm --network host \
    -v "${REPO}/ros2_ws:/ros2_ws" \
    -v "${REPO}/shared:/shared" \
    -v "${REPO}/tests:/tests:ro" \
    -e TURTLEBOT3_MODEL=waffle \
    nav2_student:humble \
    bash /tests/test_control_loop.sh
