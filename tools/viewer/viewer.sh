#!/bin/bash
# Browser viewer for the ROSbot 2R, served from atlas.
#
#   ./viewer.sh up      Foxglove bridge (ws :8765) and Lichtblick (http :8088)
#   ./viewer.sh down    stop both
#
# Both listen on loopback only; from another machine, reach them through an
# SSH tunnel. The bridge is also read-only: no publishing, services or
# parameters from the browser, so the robot cannot be driven through it.
set -euo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BRIDGE_IMAGE="nav2_student:humble-viewer"
WEB_IMAGE="ghcr.io/lichtblick-suite/lichtblick:latest"
WS_PORT="${WS_PORT:-8765}"
WEB_PORT="${WEB_PORT:-8088}"

case "${1:-}" in
    up)
        docker rm -f musi_bridge musi_lichtblick > /dev/null 2>&1 || true
        docker run -d --name musi_bridge --network host --ipc host \
            -e ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}" "${BRIDGE_IMAGE}" \
            bash -c "source /opt/ros/humble/setup.bash && \
                ros2 run foxglove_bridge foxglove_bridge --ros-args \
                -p address:=127.0.0.1 -p port:=${WS_PORT} \
                -p capabilities:='[connectionGraph,assets]' \
                -p send_buffer_limit:=100000000" > /dev/null
        docker run -d --name musi_lichtblick -p "127.0.0.1:${WEB_PORT}:8080" \
            -v "${HERE}/default-layout.json:/lichtblick/default-layout.json:ro" \
            -v "${HERE}/sim-nav-layout.json:/src/sim-nav-layout.json:ro" \
            "${WEB_IMAGE}" > /dev/null
        # Click-to-publish in the 3D panel sends nothing without this; the reason
        # is in the script.
        docker exec -i musi_lichtblick sh < "${HERE}/patch-lichtblick.sh"
        echo "From another machine first: ssh -L ${WEB_PORT}:localhost:${WEB_PORT} -L ${WS_PORT}:localhost:${WS_PORT} atlas"
        echo "http://localhost:${WEB_PORT}/?ds=foxglove-websocket&ds.url=ws://localhost:${WS_PORT}"
        ;;
    down)
        docker rm -f musi_bridge musi_lichtblick > /dev/null 2>&1 || true
        ;;
    *)
        echo "usage: $0 up | down" >&2
        exit 2
        ;;
esac
