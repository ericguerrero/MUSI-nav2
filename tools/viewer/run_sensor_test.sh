#!/bin/bash
# One sensor test on the ROSbot 2R: a single sensor container, no driver.
#
#   ./run_sensor_test.sh rplidar <output directory> [window seconds]
#   ./run_sensor_test.sh astra   <output directory> [window seconds]
#   ./run_sensor_test.sh none    <output directory> [window seconds]   probe only
#
# PROBE_PARTS="serial dmesg" adds the STM32 serial listener and the kernel log.
#
# Phases: probe alone (baseline), sensor running (window), sensor stopped.
# The probe and the capture write on the robot; this script pulls their files
# every 5 s, so a power cut loses at most the last seconds.
# Exit code: 0 the robot stayed up, 3 the robot stopped answering.
set -uo pipefail

ROBOT_HOST="${ROBOT_HOST:-husarion@192.168.77.2}"
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SERVICE="${1:?usage: $0 <rplidar|astra> <output directory> [window seconds]}"
OUT="${2:?usage: $0 <rplidar|astra> <output directory> [window seconds]}"
WINDOW="${3:-300}"
BASELINE="${BASELINE:-60}"
AFTER="${AFTER:-20}"

# Optional probe parts, as words: "serial", "dmesg", both, or empty.
PROBE_PARTS="${PROBE_PARTS:-}"

case "${SERVICE}" in
    # Control run: the probe alone for the whole window, no sensor.
    none) CAPTURE_OPTIONS="" ;;
    # The lidar node starts 30 s after its container; capture joins at 120 s.
    rplidar)
        CAPTURE_DELAY="${CAPTURE_DELAY:-120}"
        CAPTURE_OPTIONS="--scans 100 --frames 0 --timeout ${WINDOW}"
        ;;
    # Capture joins at 60 s and then takes a frame every 5 s to the end of
    # the window, so the camera has a subscriber for the rest of the test.
    astra)
        CAPTURE_DELAY="${CAPTURE_DELAY:-60}"
        CAPTURE_OPTIONS="--scans 0 --frames $((WINDOW / 5)) --frame-period 5 --timeout ${WINDOW}"
        ;;
    *) echo "unknown service: ${SERVICE}" >&2; exit 2 ;;
esac

robot() { ssh -o BatchMode=yes -o ConnectTimeout=5 "${ROBOT_HOST}" "$@"; }
now() { date -u +%T; }

pull() {
    rsync -a -e "ssh -o BatchMode=yes -o ConnectTimeout=4" \
        "${ROBOT_HOST}:capture_out/" "${OUT}/capture/" 2> /dev/null &&
    rsync -a -e "ssh -o BatchMode=yes -o ConnectTimeout=4" \
        "${ROBOT_HOST}:probe_out/" "${OUT}/probe/" 2> /dev/null
}

# Pull every 5 s for the given time. Returns 1 as soon as the robot is gone.
watch() {
    local label="$1" length="$2" started=${SECONDS}
    while [ $((SECONDS - started)) -lt "${length}" ]; do
        sleep 5
        if ! pull && ! ping -c1 -W1 "${ROBOT_HOST#*@}" > /dev/null 2>&1; then
            echo "$(now) ${label} +$((SECONDS - started))s ROBOT DOES NOT ANSWER"
            return 1
        fi
        if [ $(((SECONDS - started) % 30)) -lt 5 ]; then
            echo "$(now) ${label} +$((SECONDS - started))s $(tail -1 "${OUT}/probe/probe.jsonl" 2> /dev/null)"
        fi
    done
}

mkdir -p "${OUT}/capture" "${OUT}/probe"
robot 'mkdir -p ~/capture' || exit 1
scp -q -o BatchMode=yes "${HERE}/capture.py" "${HERE}/probe.py" "${HERE}/robot_side.sh" \
    "${ROBOT_HOST}:capture/" || exit 1
robot 'bash ~/capture/robot_side.sh prepare' || exit 1
robot "bash ~/capture/robot_side.sh probe ${PROBE_PARTS}"

lost=0
echo "$(now) baseline ${BASELINE} s, probe only (parts: ${PROBE_PARTS:-none})"
watch baseline "${BASELINE}" || lost=1
if [ "${SERVICE}" = none ]; then
    [ "${lost}" -eq 0 ] && { watch control "${WINDOW}" || lost=1; }
    [ "${lost}" -eq 0 ] && { robot 'bash ~/capture/robot_side.sh finish'; pull; }
    echo "$(now) done, robot lost: ${lost}; probe samples: $(wc -l < "${OUT}/probe/probe.jsonl" 2> /dev/null)"
    [ "${lost}" -eq 0 ] || exit 3
    exit 0
fi
if [ "${lost}" -eq 0 ]; then
    echo "$(now) start ${SERVICE}, window ${WINDOW} s"
    robot "bash ~/capture/robot_side.sh start ${SERVICE} ${CAPTURE_DELAY} ${CAPTURE_OPTIONS}"
    watch "${SERVICE}" "${WINDOW}" || lost=1
fi
if [ "${lost}" -eq 0 ]; then
    echo "$(now) stop ${SERVICE}, watch ${AFTER} s more"
    robot "bash ~/capture/robot_side.sh stop ${SERVICE}"
    watch after "${AFTER}" || lost=1
fi
if [ "${lost}" -eq 0 ]; then
    robot 'bash ~/capture/robot_side.sh finish'
    pull
fi
echo "$(now) done, robot lost: ${lost}; probe samples: $(wc -l < "${OUT}/probe/probe.jsonl" 2> /dev/null)"
[ "${lost}" -eq 0 ] || exit 3
