#!/usr/bin/env bash
# Copy the power log to the robot and (re)start its container.
# Usage: ./deploy.sh        (ROBOT_HOST overrides the default robot address)
set -euo pipefail

ROBOT_HOST="${ROBOT_HOST:-husarion@192.168.77.2}"
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

ssh "$ROBOT_HOST" 'mkdir -p ~/powerlog/log'
scp -q "$HERE"/*.py "$HERE"/compose.yaml "$ROBOT_HOST":powerlog/
ssh "$ROBOT_HOST" 'cd ~/powerlog && docker compose up -d --force-recreate'
