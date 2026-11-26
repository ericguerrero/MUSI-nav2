#!/bin/bash
# Run the drive comparison simulation test inside the image, once per robot.
#
#   tests/run_drive_test_sim_test.sh                  all four robots
#   tests/run_drive_test_sim_test.sh rosbot_mecanum   one robot
#
# Deliberately not --network host: the lab ROSbot 2R runs on ROS_DOMAIN_ID 0
# with no namespace, and a simulation on the same network would exchange
# /cmd_vel with it. The default bridge network plus ROS_LOCALHOST_ONLY keeps
# every DDS packet inside the container. Set CONTAINER_PREFIX to tell these
# containers apart from someone else's on a shared computer.
REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ROBOTS=("$@")
if [ "${#ROBOTS[@]}" -eq 0 ]; then
    ROBOTS=(turtlebot3 rosbot rosbot_mecanum ackermann)
fi

failed=0
for robot in "${ROBOTS[@]}"; do
    echo "=== ${robot} ==="
    docker run --rm \
        --name "${CONTAINER_PREFIX:-drive_test_}${robot}_$$" \
        -e ROS_LOCALHOST_ONLY=1 \
        -e ROS_DOMAIN_ID=78 \
        -e ROBOT="${robot}" \
        -e STARTUP_TIMEOUT_S \
        -v "${REPO}/ros2_ws:/ros2_ws:ro" \
        -v "${REPO}/shared:/shared:ro" \
        -v "${REPO}/tests:/tests:ro" \
        nav2_student:humble \
        bash /tests/test_drive_test_sim.sh || failed=$((failed + 1))
done
echo "=== ${failed} robot(s) failed ==="
[ "${failed}" -eq 0 ]
