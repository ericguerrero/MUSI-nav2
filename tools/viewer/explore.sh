#!/bin/bash
# Frontier exploration (explore_lite from m-explore-ros2) for the isolated
# ROSbot simulation. Needs musi_sim, SLAM and Nav2 already running.
#
#   ./explore.sh build   compile explore_lite into ./explore_ws
#   ./explore.sh up      start the node, paused
#   ./explore.sh on      start or resume exploring
#   ./explore.sh off     stop exploring and cancel the current goal
#   ./explore.sh down    remove the node
#
# In the browser the same on/off is the pair of buttons in the layout: they
# publish std_msgs/Bool on /explore/resume.
set -euo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
IMAGE="nav2_student:humble"
NAME="musi_sim_explore"
SETUP="source /opt/ros/humble/setup.bash && source /explore_ws/install/local_setup.bash"

publish() {
    docker exec "${NAME}" bash -c "${SETUP} && \
        ros2 topic pub --once /explore/resume std_msgs/msg/Bool '{data: $1}'" > /dev/null
}

case "${1:-}" in
    build)
        touch "${HERE}/explore_ws/src/m-explore-ros2/map_merge/COLCON_IGNORE"
        docker run --rm -v "${HERE}/explore_ws:/explore_ws" "${IMAGE}" bash -c \
            "source /opt/ros/humble/setup.bash && cd /explore_ws && \
            colcon build --packages-select explore_lite_msgs explore_lite \
            --cmake-args -DCMAKE_BUILD_TYPE=Release"
        ;;
    up)
        docker rm -f "${NAME}" > /dev/null 2>&1 || true
        docker run -d --name "${NAME}" \
            --network container:musi_sim --ipc container:musi_sim \
            -e ROS_DOMAIN_ID=77 -e ROS_LOCALHOST_ONLY=1 \
            -v "${HERE}/explore_ws:/explore_ws:ro" \
            -v "${HERE}/explore_params.yaml:/explore_params.yaml:ro" \
            "${IMAGE}" bash -c "${SETUP} && exec ros2 run explore_lite explore \
                --ros-args --params-file /explore_params.yaml" > /dev/null
        # explore_lite starts driving as soon as it sees the map and has no
        # parameter to start idle, so it is paused here until someone asks.
        for _ in $(seq 30); do
            docker exec "${NAME}" bash -c "${SETUP} && ros2 topic info /explore/resume" 2> /dev/null \
                | grep -q "Subscription count: 1" && break
            sleep 1
        done
        publish false
        echo "explore_lite is up and paused. Start it from the browser or with: $0 on"
        ;;
    on) publish true ;;
    off) publish false ;;
    down)
        docker rm -f "${NAME}" > /dev/null 2>&1 || true
        ;;
    *)
        echo "usage: $0 build | up | on | off | down" >&2
        exit 2
        ;;
esac
