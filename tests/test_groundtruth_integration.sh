#!/bin/bash
# Integration smoke test for the groundtruth pose signal.
#
# Launches the simulation headless, starts the groundtruth node, and asserts
# that the true-pose topics and TF frame appear. Verifies the whole chain:
# libgazebo_ros_state.so -> /model_states -> groundtruth_pose.py -> /groundtruth/*
#
# Runs inside the container. Takes about 90 seconds.
set -o pipefail

PASS=0
FAIL=0

check() {
    local name="$1"
    shift
    if "$@" >/dev/null 2>&1; then
        echo "  PASS: ${name}"
        PASS=$((PASS + 1))
    else
        echo "  FAIL: ${name}"
        FAIL=$((FAIL + 1))
    fi
}

source /opt/ros/humble/setup.bash
export TURTLEBOT3_MODEL=waffle

echo "Launching simulation (headless; Gazebo needs ~60s to expose its ROS interface)..."
ros2 launch /ros2_ws/src/simulation.launch.py headless:=true rviz:=false \
    >/tmp/sim.log 2>&1 &
SIM_PID=$!
cleanup() {
    local status=$?
    pkill -f gzserver >/dev/null 2>&1
    pkill -f groundtruth_pose >/dev/null 2>&1
    kill ${SIM_PID} >/dev/null 2>&1
    exit ${status}
}
trap cleanup EXIT

# Gazebo plus robot spawn needs time before any topic is meaningful.
for _ in $(seq 1 150); do
    if ros2 topic list 2>/dev/null | grep -qx /model_states; then break; fi
    sleep 1
done

echo "Checking the Gazebo state plugin..."
check "/model_states exists (libgazebo_ros_state.so loaded)" \
    bash -c 'ros2 topic list | grep -qx /model_states'
check "/model_states carries data" \
    timeout 10 ros2 topic echo /model_states --once

echo "Starting the groundtruth node..."
python3 /ros2_ws/src/groundtruth_pose.py >/tmp/gt.log 2>&1 &
sleep 15

echo "Checking published groundtruth..."
check "/groundtruth/pose publishes" timeout 15 ros2 topic echo /groundtruth/pose --once
check "/groundtruth/odom publishes" timeout 15 ros2 topic echo /groundtruth/odom --once
check "TF world -> base_footprint_gt resolves" \
    bash -c 'timeout 15 ros2 run tf2_ros tf2_echo world base_footprint_gt 2>&1 | grep -q Translation'

echo "Measuring publish rate (target 50 Hz, tolerance 10%)..."
RATE=$(timeout 12 ros2 topic hz /groundtruth/odom 2>&1 \
    | grep -oP 'average rate: \K[0-9.]+' | tail -1)
if [ -n "${RATE}" ] && awk "BEGIN{exit !(${RATE} > 45 && ${RATE} < 55)}"; then
    echo "  PASS: rate ${RATE} Hz within 45-55"
    PASS=$((PASS + 1))
else
    echo "  FAIL: rate '${RATE:-none}' outside 45-55"
    FAIL=$((FAIL + 1))
fi

echo "Checking that an absent model fails loudly rather than silently..."
# The healthy node must go first, or it keeps publishing /groundtruth/pose and the
# "publishes nothing" assertion below passes or fails for the wrong reason.
pkill -f groundtruth_pose
sleep 3
python3 /ros2_ws/src/groundtruth_pose.py --ros-args \
    -p model_name:=no_such_robot >/tmp/gt_missing.log 2>&1 &
sleep 8
check "warns when the model is absent" \
    grep -qi "no_such_robot" /tmp/gt_missing.log
check "publishes nothing when the model is absent" \
    bash -c '! timeout 5 ros2 topic echo /groundtruth/pose --once'

echo
echo "passed: ${PASS}  failed: ${FAIL}"
if [ "${FAIL}" -eq 0 ]; then exit 0; fi
exit 1
