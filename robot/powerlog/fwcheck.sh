#!/usr/bin/env bash
# Check that the STM32 holds the pinned firmware. Read-only.
# Run on the robot, in ~/powerlog. The driver is down for about a minute.
# Exit code: 0 MATCH, 2 MISMATCH, 3 READ FAILED, 1 no verdict (could not run).
set -euo pipefail
cd "$(dirname "${BASH_SOURCE[0]}")"

DRIVER="${DRIVER:-rosbot}"
CHECK="powerlog-fwcheck"

# The check executes this image's flasher script for its pin sequence, so it
# must be the image that was reviewed: the one pinned in compose.yaml.
IMAGE="$(sed -n 's/^ *image: *//p' compose.yaml)"
if [ -z "$IMAGE" ]; then
    echo "fwcheck: no image pinned in compose.yaml" >&2
    exit 1
fi

# A driver on another image carries another firmware: comparing would mislead.
DRIVER_IMAGE="$(docker inspect -f '{{.Config.Image}}' "$DRIVER" 2>/dev/null || true)"
if [ -n "$DRIVER_IMAGE" ] && [ "$DRIVER_IMAGE" != "$IMAGE" ]; then
    echo "fwcheck: driver image $DRIVER_IMAGE differs from the pinned $IMAGE;" \
        "review its /flash-firmware.py and update compose.yaml first" >&2
    exit 1
fi

WAS_RUNNING="$(docker ps -q -f "name=^${DRIVER}$")"
mkdir -p log

cleanup() {
    # If this script dies first, the check still holds the UART with the chip
    # in its bootloader. Stop it (it then leaves the bootloader) before the
    # driver may open the same port.
    docker stop "$CHECK" >/dev/null 2>&1 || true
    if [ -n "$WAS_RUNNING" ]; then
        if docker start "$DRIVER" >/dev/null; then
            echo "fwcheck: driver started again"
        else
            echo "fwcheck: DRIVER NOT RESTARTED, run: docker start $DRIVER" >&2
        fi
    fi
}
trap cleanup EXIT
# A dropped terminal must not cut the check short: let it finish and restart
# the driver.
trap '' HUP

# The driver holds the STM32's UART; the bootloader needs it.
if [ -n "$WAS_RUNNING" ]; then
    echo "fwcheck: stopping driver"
    docker stop "$DRIVER" >/dev/null
fi

docker run --rm --name "$CHECK" --privileged \
    -v "$PWD":/powerlog:ro -v "$PWD/log":/log \
    "$IMAGE" python3 /powerlog/fwcheck.py
