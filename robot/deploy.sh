#!/bin/bash
# Copy the robot stack to the ROSbot and start it there.
#
# Arguments are global Compose options, which go before the subcommand:
#   ./robot/deploy.sh                    driver and lidar
#   ./robot/deploy.sh --profile camera   also the depth camera
set -euo pipefail

ROBOT_HOST="${ROBOT_HOST:-husarion@192.168.77.2}"
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

scp "${HERE}/compose.yaml" "${ROBOT_HOST}:~/"
ssh "${ROBOT_HOST}" "docker compose $* pull && docker compose $* up -d"
