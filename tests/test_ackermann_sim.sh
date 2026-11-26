#!/bin/bash
# Simulation test for the Ackermann robot.
#
# The robot exists to show two things a differential robot does not have: it
# cannot rotate on the spot, and it cannot turn tighter than its minimum radius.
# This test drives it in Gazebo and checks both against Gazebo's own pose, so a
# change to the model or the controller that lets it spin or turn tighter fails.
#
# Runs inside the image. Use tests/run_ackermann_sim_test.sh from the host.

# Set by min_turn_radius in ackermann.urdf.xacro.
MIN_RADIUS=0.50
# The true radius may exceed the geometric one through tyre slip, and fall short
# of it only by measurement noise.
RADIUS_TOLERANCE="${RADIUS_TOLERANCE:-0.10}"
RADIUS_UNDERSHOOT=0.03
STARTUP_TIMEOUT_S="${STARTUP_TIMEOUT_S:-120}"

TOPICS=(
    "/cmd_vel geometry_msgs/msg/Twist"
    "/ackermann_steering_controller/odometry nav_msgs/msg/Odometry"
    "/groundtruth/odom nav_msgs/msg/Odometry"
    "/joint_states sensor_msgs/msg/JointState"
)

source /opt/ros/humble/setup.bash

failures=0
pass() { echo "PASS  $1"; }
fail() { echo "FAIL  $1"; failures=$((failures + 1)); }
# check "<awk condition over v>" <value> "<description>"
check() {
    if awk -v v="$2" "BEGIN {exit !(v != \"\" && ($1))}"; then pass "$3"; else fail "$3"; fi
}

LOG=/tmp/ackermann_sim.log
ros2 launch /ros2_ws/src/ackermann_simulation.launch.py headless:=true >"${LOG}" 2>&1 &
LAUNCH_PID=$!
trap 'kill -INT ${LAUNCH_PID} 2>/dev/null; sleep 5; kill -9 ${LAUNCH_PID} 2>/dev/null; pkill -9 -f "ign gazebo" 2>/dev/null' EXIT

deadline=$((SECONDS + STARTUP_TIMEOUT_S))
until grep -q "Configured and activated .*ackermann_steering_controller" "${LOG}"; do
    if ! kill -0 "${LAUNCH_PID}" 2>/dev/null || [ "${SECONDS}" -ge "${deadline}" ]; then
        fail "robot spawns and ackermann_steering_controller activates within ${STARTUP_TIMEOUT_S} s"
        tail -40 "${LOG}"
        echo "Result: ${failures} failure(s)"
        exit 1
    fi
    sleep 2
done
pass "robot spawns and ackermann_steering_controller activates"
sleep 5

topics="$(ros2 topic list -t)"
for entry in "${TOPICS[@]}"; do
    name="${entry% *}"
    type="${entry#* }"
    if grep -qx "${name} \[${type}\]" <<<"${topics}"; then
        pass "topic ${name} [${type}]"
    else
        fail "topic ${name} [${type}]"
    fi
done

# drive <linear> <angular> <seconds>: sets one shell variable per probe field.
drive() {
    unset truth_distance truth_rotation truth_radius odom_distance odom_rotation odom_radius
    eval "$(python3 /tests/ackermann_probe.py --linear "$1" --angular "$2" --duration "$3" | tr ' ' '\n')"
    sleep 2
}
abs() { awk -v v="$1" 'BEGIN {if (v == "") exit; print (v < 0 ? -v : v)}'; }

drive 0.3 0.0 4
check "v > 0.9" "${truth_distance}" \
    "0.3 m/s for 4 s on /cmd_vel moves the robot more than 0.9 m (true ${truth_distance} m)"
check "v > 0.9" "${odom_distance}" "wheel odometry reports the straight run (${odom_distance} m)"

drive 0.0 1.0 4
check "v < 0.02" "$(abs "${truth_rotation}")" \
    "angular.z = 1.0 with linear.x = 0 does not rotate the robot (true ${truth_rotation} rad)"
check "v < 0.02" "${truth_distance}" "and does not move it (true ${truth_distance} m)"

drive 0.3 0.3 8
check "v > 1.0 * (1 - ${RADIUS_TOLERANCE}) && v < 1.0 * (1 + ${RADIUS_TOLERANCE})" "${truth_radius}" \
    "a 1.00 m arc, wider than the minimum, is followed (true ${truth_radius} m)"

# Commanded radius 0.3 / 2.0 = 0.15 m, well inside the minimum, in both directions.
for angular in 2.0 -2.0; do
    drive 0.3 "${angular}" 8
    check "v > ${MIN_RADIUS} * (1 - ${RADIUS_UNDERSHOOT}) && v < ${MIN_RADIUS} * (1 + ${RADIUS_TOLERANCE})" \
        "${truth_radius}" \
        "commanded 0.15 m arc (angular.z = ${angular}) turns at the ${MIN_RADIUS} m minimum (true ${truth_radius} m)"
    check "v > ${MIN_RADIUS} * (1 - ${RADIUS_UNDERSHOOT}) && v < ${MIN_RADIUS} * (1 + ${RADIUS_TOLERANCE})" \
        "${odom_radius}" "wheel odometry sees the same limit (${odom_radius} m)"
done

echo "Result: ${failures} failure(s)"
[ "${failures}" -eq 0 ]
