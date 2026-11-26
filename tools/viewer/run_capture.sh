#!/bin/bash
# One short sensor capture on the ROSbot 2R: lidar and camera only, no driver.
#
#   ./run_capture.sh <output directory on this machine>
#
# The capture runs on the robot and its files are pulled over SSH every few
# seconds, so neither a firewall between the machines nor a power cut on the
# robot loses what was already captured. Sensors are stopped as soon as the
# capture is complete.
set -uo pipefail

ROBOT_HOST="${ROBOT_HOST:-husarion@192.168.77.2}"
IMAGE="husarion/rosbot:humble-0.14.0-20250319"
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
OUT="${1:?usage: $0 <output directory>}"
LIMIT=125   # seconds the sensors may stay on

robot() { ssh -o BatchMode=yes -o ConnectTimeout=5 "${ROBOT_HOST}" "$@"; }
pull() { rsync -a -e "ssh -o BatchMode=yes -o ConnectTimeout=4" "${ROBOT_HOST}:capture_out/" "${OUT}/"; }
now() { date -u +%T; }

mkdir -p "${OUT}"
robot 'mkdir -p ~/capture ~/capture_out && rm -f ~/capture_out/*; uptime; vcgencmd get_throttled' || exit 1
scp -q -o BatchMode=yes "${HERE}/capture.py" "${ROBOT_HOST}:capture/" || exit 1

echo "$(now) start capture container"
robot "docker rm -f capture > /dev/null 2>&1; docker run -d --name capture --network host --ipc host \
    -v \$HOME/capture:/capture:ro -v \$HOME/capture_out:/out ${IMAGE} \
    python3 /capture/capture.py --out /out --scans 100 --frames 5 --timeout 110 > /dev/null" || exit 1

echo "$(now) start rplidar and astra"
robot 'cd ~ && docker compose --profile camera up -d rplidar astra 2>&1 | tail -4'

started=${SECONDS}
while [ $((SECONDS - started)) -lt "${LIMIT}" ]; do
    sleep 5
    if ! pull 2> /dev/null; then
        echo "$(now) pull failed"
        ping -c1 -W1 "${ROBOT_HOST#*@}" > /dev/null 2>&1 || { echo "$(now) robot does not answer"; break; }
    fi
    state="$(robot 'docker inspect capture --format "{{.State.Running}}"; docker logs --tail 1 capture 2>&1' 2> /dev/null | tr '\n' ' ')"
    echo "$(now) +$((SECONDS - started))s files=$(ls "${OUT}" | wc -l) capture: ${state}"
    case "${state}" in false*) break ;; esac
done

echo "$(now) stop sensors"
robot 'cd ~ && ./stack.sh down 2>&1 | tail -3; docker logs capture 2>&1 | tail -4; docker rm -f capture > /dev/null 2>&1
       docker ps --format "{{.Names}} {{.Status}}"; uptime; vcgencmd get_throttled'
pull 2> /dev/null
ls -la "${OUT}"
