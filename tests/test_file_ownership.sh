#!/bin/bash
# Files the container writes into bind mounts must belong to the host user.
#
# When the container ran as root, every map, bag and result a student saved was
# root-owned on the host and could not be edited or deleted without sudo.
# Runs on the host, not in the container, because ownership is a host property.
set -o pipefail
REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
PROBE="ownership_probe_$$"
PASS=0
FAIL=0

check_owner() {
    local label="$1" path="$2"
    local owner
    owner=$(stat -c '%u:%g' "${path}" 2>/dev/null)
    if [ "${owner}" = "$(id -u):$(id -g)" ]; then
        echo "  PASS: ${label} owned by host user (${owner})"; PASS=$((PASS + 1))
    else
        echo "  FAIL: ${label} owned by ${owner:-missing}, expected $(id -u):$(id -g)"
        FAIL=$((FAIL + 1))
    fi
}

docker run --rm \
    -v "${REPO}/shared:/shared" -v "${REPO}/ros2_ws:/ros2_ws" \
    nav2_student:humble \
    bash -c "touch /shared/${PROBE} /ros2_ws/${PROBE} && mkdir -p /shared/${PROBE}_dir"

check_owner "file written to /shared" "${REPO}/shared/${PROBE}"
check_owner "file written to /ros2_ws" "${REPO}/ros2_ws/${PROBE}"
check_owner "directory created in /shared" "${REPO}/shared/${PROBE}_dir"

# Cleanup must work without sudo; that is the whole point.
rm -rf "${REPO}/shared/${PROBE}" "${REPO}/ros2_ws/${PROBE}" "${REPO}/shared/${PROBE}_dir" \
    || echo "  (cleanup needed sudo, which proves the defect)"

echo "passed: ${PASS}  failed: ${FAIL}"
[ "${FAIL}" -eq 0 ]
