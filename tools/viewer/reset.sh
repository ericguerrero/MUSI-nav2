#!/bin/bash
# Reset for the isolated ROSbot simulation: robot back at its start pose,
# odometry zeroed, map forgotten. Needs musi_sim and musi_sim_slam running.
#
#   ./reset.sh up     start the node that listens for the reset request
#   ./reset.sh now    reset the simulation
#   ./reset.sh down   remove the node
#
# In the browser, now is the "Reset simulation" button in the layout: it
# publishes std_msgs/Empty on /sim/reset.
set -euo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
IMAGE="nav2_student:humble"
NAME="musi_sim_reset"
SETUP="source /opt/ros/humble/setup.bash && source /opt/rosbot_ws/install/local_setup.bash"

case "${1:-}" in
    up)
        docker rm -f "${NAME}" > /dev/null 2>&1 || true
        docker run -d --name "${NAME}" \
            --network container:musi_sim --ipc container:musi_sim \
            -e ROS_DOMAIN_ID=77 -e ROS_LOCALHOST_ONLY=1 \
            -v "${HERE}/sim_reset.py:/sim_reset.py:ro" \
            "${IMAGE}" bash -c "${SETUP} && \
                exec python3 /sim_reset.py --ros-args -p use_sim_time:=true" > /dev/null
        ;;
    now)
        docker exec "${NAME}" bash -c "${SETUP} && \
            ros2 topic pub --once /sim/reset std_msgs/msg/Empty '{}'" > /dev/null
        ;;
    down)
        docker rm -f "${NAME}" > /dev/null 2>&1 || true
        ;;
    *)
        echo "usage: $0 up | now | down" >&2
        exit 2
        ;;
esac
