#!/usr/bin/env bash
# Check that the STM32 holds the firmware of the driver image. Read-only.
# Run on the robot, in ~/powerlog. The driver is down for about a minute.
# Exit code: 0 MATCH, 1 MISMATCH, 2 READ FAILED, 3 cannot run.
set -euo pipefail
cd "$(dirname "${BASH_SOURCE[0]}")"

DRIVER="${DRIVER:-rosbot}"

# The reference firmware.bin is the one inside the driver's own image.
if ! IMAGE="$(docker inspect -f '{{.Config.Image}}' "$DRIVER" 2>/dev/null)"; then
    echo "fwcheck: no '$DRIVER' container, so no image to take the reference firmware from" >&2
    exit 3
fi

WAS_RUNNING="$(docker ps -q -f "name=^${DRIVER}$")"

restart_driver() {
    if [ -n "$WAS_RUNNING" ]; then
        docker start "$DRIVER" >/dev/null
        echo "fwcheck: driver started again"
    fi
}
trap restart_driver EXIT

# The driver holds the STM32's UART; the bootloader needs it.
if [ -n "$WAS_RUNNING" ]; then
    echo "fwcheck: stopping driver"
    docker stop "$DRIVER" >/dev/null
fi

mkdir -p log
docker run --rm --privileged \
    -v "$PWD":/powerlog:ro -v "$PWD/log":/log \
    "$IMAGE" python3 /powerlog/fwcheck.py
