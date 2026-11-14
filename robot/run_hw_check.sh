#!/bin/bash
# Run one stage of the hardware check on the ROSbot and fetch its report.
#
#   ./robot/run_hw_check.sh --stage sensors
#   ./robot/run_hw_check.sh --stage motors --wheels-off-ground
#
# The check runs on the robot, in a throwaway container of the driver image:
# same ROS packages as the driver, and no dependence on the network between
# this machine and the robot.
set -uo pipefail

ROBOT_HOST="${ROBOT_HOST:-husarion@192.168.77.2}"
IMAGE="husarion/rosbot:humble-0.14.0-20250319"
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# -n: this call must not swallow input meant for the stage's prompts.
ssh -n "${ROBOT_HOST}" "mkdir -p hw_check hw_check_out" || exit 2
scp -q "${HERE}"/hw_check/*.py "${ROBOT_HOST}:hw_check/" || exit 2

# -tt gives the interactive stages a terminal for their prompts, and forces one
# even when this script itself is started without a terminal.
ssh -tt "${ROBOT_HOST}" "docker run --rm -it --network host --ipc host \
    -v \$HOME/hw_check:/hw_check:ro -v \$HOME/hw_check_out:/out \
    ${IMAGE} python3 /hw_check/hw_check.py \
    --throttled \$(vcgencmd get_throttled | cut -d= -f2) $*"
code=$?

mkdir -p "${HERE}/reports"
scp -q "${ROBOT_HOST}:hw_check_out/*.json" "${HERE}/reports/" || exit 2
exit "${code}"
