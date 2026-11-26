#!/bin/bash
# One-time root setup on the ROSbot for the clock guard in stack.sh.
# Run on the robot:  sudo ./install.sh
# Safe to run again, and safe to run while the driver is up: it only stops NTP.
set -euo pipefail

if [ "$(id -u)" -ne 0 ]; then
    echo "install: run with sudo" >&2
    exit 1
fi

# NTP off for good: nothing may step the clock behind the driver's back.
# stack.sh sets the clock itself, only while the driver is down.
timedatectl set-ntp false

echo "install: NTP enabled: $(timedatectl show -p NTP --value)"
