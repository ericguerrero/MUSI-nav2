#!/bin/bash
# Copy the robot stack to the ROSbot and start it there.
#
# Arguments are global Compose options, which go before the subcommand:
#   ./robot/deploy.sh                    driver and lidar
#   ./robot/deploy.sh --profile camera   also the depth camera
#
# First time on a robot, or on a freshly flashed card, also run there:
#   sudo ./install.sh
set -euo pipefail

ROBOT_HOST="${ROBOT_HOST:-husarion@192.168.77.2}"
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

scp "${HERE}/compose.yaml" "${HERE}/stack.sh" "${HERE}/install.sh" "${ROBOT_HOST}:~/"
# Through stack.sh, never `docker compose up` directly: it sets the clock
# before the driver starts and keeps it from jumping afterwards. -t because it
# asks for the robot's sudo password when it has to set the clock.
ssh -t "${ROBOT_HOST}" "./stack.sh up $*"
