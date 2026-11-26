#!/bin/bash
# The whole isolated ROSbot 2R simulation stack behind the browser viewer:
# Gazebo, Foxglove bridge, SLAM, Nav2, exploration and the reset node.
#
#   ./sim.sh up       start everything (about two minutes)
#   ./sim.sh down     remove everything; the map is lost
#   ./sim.sh restart  down, then up
#   ./sim.sh status   what is running
#
# Network: default bridge, ROS_LOCALHOST_ONLY=1 and domain 77. The lab robot is
# on atlas's network on domain 0, and a simulation with host networking would
# exchange /cmd_vel with it. The other containers join musi_sim's network and
# IPC namespaces, which is the only way they can see its DDS traffic.
#
# The bridge listens on 127.0.0.1:8765; Lichtblick (viewer.sh) on :8088.
set -euo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO="$(cd "${HERE}/../.." && pwd)"
ROS2_WS="${ROS2_WS:-${REPO}/ros2_ws}"
SETUP="source /opt/ros/humble/setup.bash && source /opt/rosbot_ws/install/local_setup.bash"
ISOLATION=(-e ROS_LOCALHOST_ONLY=1 -e ROS_DOMAIN_ID=77)
JOIN=(--network container:musi_sim --ipc container:musi_sim)
ALL=(musi_sim_reset musi_sim_explore musi_sim_nav musi_sim_slam musi_sim_bridge musi_sim)

wait_for_log() {  # container, text, seconds
    local waited=0
    until docker logs "$1" 2>&1 | grep -q "$2"; do
        if [ "${waited}" -ge "$3" ]; then
            echo "$1: no '$2' after $3 s" >&2
            return 1
        fi
        sleep 2
        waited=$((waited + 2))
    done
}

up() {
    docker run -d --name musi_sim --ipc shareable "${ISOLATION[@]}" \
        -e ROSBOT_CAMERA=true -p 127.0.0.1:8765:8765 \
        -v "${ROS2_WS}:/ros2_ws:ro" nav2_student:humble \
        bash -c "${SETUP} && exec ros2 launch /ros2_ws/src/rosbot_simulation.launch.py headless:=true" > /dev/null
    wait_for_log musi_sim "All systems are up and running" 180

    # clientPublish lets the browser drive this simulation. The bridge that
    # viewer.sh starts for the real robot does not have it.
    docker run -d --name musi_sim_bridge "${JOIN[@]}" "${ISOLATION[@]}" nav2_student:humble-viewer \
        bash -c "${SETUP} && exec ros2 run foxglove_bridge foxglove_bridge --ros-args \
            -p address:=0.0.0.0 -p port:=8765 \
            -p capabilities:='[connectionGraph,assets,clientPublish]' \
            -p send_buffer_limit:=100000000 -p use_sim_time:=true" > /dev/null

    "${HERE}/slam.sh" up

    docker run -d --name musi_sim_nav "${JOIN[@]}" "${ISOLATION[@]}" \
        -v "${HERE}/nav2_params_rosbot_sim.yaml:/nav2_params.yaml:ro" nav2_student:humble \
        bash -c "${SETUP} && exec ros2 launch nav2_bringup navigation_launch.py \
            use_sim_time:=True params_file:=/nav2_params.yaml" > /dev/null
    wait_for_log musi_sim_nav "Managed nodes are active" 180

    "${HERE}/explore.sh" up
    "${HERE}/reset.sh" up
    echo "http://localhost:8088/?ds=foxglove-websocket&ds.url=ws://localhost:8765&layoutUrl=http://localhost:8088/sim-nav-layout.json"
}

down() {
    docker rm -f "${ALL[@]}" > /dev/null 2>&1 || true
}

case "${1:-}" in
    up)      down; up ;;
    down)    down ;;
    restart) down; up ;;
    status)  docker ps -a --format '{{.Names}}\t{{.Status}}' | grep -E '^musi_' || true ;;
    *)
        echo "usage: $0 up | down | restart | status" >&2
        exit 2
        ;;
esac
