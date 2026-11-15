#!/bin/bash
# What motion_primitives.py does when things go wrong, against tests/fake_robot.py.
#
# Every row here is a way a real robot run can fail: the operator presses
# Ctrl-C, the Wi-Fi drops odometry for a moment or for good, the robot is
# blocked, the pose estimate turns to nan or is reset under the node's feet.
# The fake robot writes down what really happened, so each row compares the
# node's exit code with where the robot actually ended.
#
# Runs inside the image. Use tests/run_motion_primitives_fault_test.sh from the host.

source /opt/ros/humble/setup.bash
NODE="python3 /ros2_ws/src/motion_primitives.py"
STATE=/tmp/fake_robot_state.json

failures=0
pass() { echo "PASS  $1"; }
fail() { echo "FAIL  $1"; failures=$((failures + 1)); }
check() {
    local label="$1"; shift
    if "$@"; then pass "${label}"; else fail "${label}"; fi
}

state() { python3 -c 'import json, sys; print(json.load(open(sys.argv[1]))[sys.argv[2]])' "${STATE}" "$1"; }
equals() { [ "$1" = "$2" ]; }
within() { awk -v v="$1" -v e="$2" -v t="$3" 'BEGIN { d = v - e; if (d < 0) d = -d; exit !(v != "" && d <= t) }'; }
below() { awk -v v="$1" -v m="$2" 'BEGIN { exit !(v != "" && v < m) }'; }

start_robot() {  # fake_robot.py arguments
    rm -f "${STATE}"
    python3 /tests/fake_robot.py "$@" >/tmp/fake_robot.log 2>&1 &
    ROBOT_PID=$!
    sleep 2
}
stop_robot() { kill -INT "${ROBOT_PID}" 2>/dev/null; wait "${ROBOT_PID}" 2>/dev/null; }

run_node() {  # seconds allowed, then the node's arguments; sets CODE
    local limit="$1"; shift
    timeout "${limit}" ${NODE} "$@" >/tmp/node.out 2>&1
    CODE=$?
    sleep 1.5   # let the robot coast to rest before its state is read
}

run_node_and_signal() {  # signal name, seconds before sending it, node arguments; sets CODE
    local signal="$1" delay="$2"; shift 2
    ${NODE} "$@" >/tmp/node.out 2>&1 &
    local pid=$!
    sleep "${delay}"
    kill "-${signal}" "${pid}"
    sleep 0.2
    # Read before the robot's own 0.5 s timeout could have stopped it anyway.
    ZERO_SEEN="$(state zero_after_motion)"
    wait "${pid}"
    CODE=$?
    sleep 1.5
}

echo "normal run:"
start_robot --mode normal
run_node 60 forward 1.0
check "forward 1.0 ends with exit code 0 (got ${CODE})" equals "${CODE}" 0
check "the robot really is at $(state x) m (1.00 +- 0.02)" within "$(state x)" 1.00 0.02
stop_robot

for signal in INT TERM; do
    echo "operator stops the node with SIG${signal}:"
    start_robot --mode normal
    run_node_and_signal "${signal}" 4 forward 2.0
    check "exit code 130 (got ${CODE})" equals "${CODE}" 130
    check "a zero command reached the robot at once" equals "${ZERO_SEEN}" True
    check "the robot is at rest (speed $(state speed))" below "$(state speed)" 0.01
    stop_robot
done

echo "odometry stops for good while moving:"
start_robot --mode silent --after 2
run_node 60 forward 2.0
check "exit code 5 (got ${CODE})" equals "${CODE}" 5
check "a zero command reached the robot" equals "$(state zero_after_motion)" True
stop_robot

echo "the robot is blocked:"
start_robot --mode frozen
run_node 30 forward 0.05
check "exit code 4 (got ${CODE})" equals "${CODE}" 4
check "a zero command reached the robot" equals "$(state zero_after_motion)" True
stop_robot

echo "odometry arrives only every 0.3 s:"
start_robot --mode normal --period 0.3
run_node 60 turn 90
check "turn 90 ends with exit code 0 (got ${CODE})" equals "${CODE}" 0
check "the robot really turned $(state yaw_deg) deg (90 +- 1.5)" within "$(state yaw_deg)" 90 1.5
stop_robot

echo "odometry turns to nan while moving:"
start_robot --mode nan --after 2
run_node 60 forward 2.0
check "exit code 5, as for odometry that stopped (got ${CODE})" equals "${CODE}" 5
check "the robot did not drive on: it is at $(state x) m (below 1.0)" below "$(state x)" 1.0
stop_robot

echo "the pose estimate is reset while moving:"
start_robot --mode reset --after 3
run_node 90 forward 2.0
check "exit code 6 (got ${CODE})" equals "${CODE}" 6
check "the robot did not drive on: it is at $(state x) m (below 1.2)" below "$(state x)" 1.2
check "a zero command reached the robot" equals "$(state zero_after_motion)" True
stop_robot

echo "a whole number as a parameter:"
start_robot --mode normal
run_node 60 turn 30 --ros-args -p max_angular:=1
check "turn 30 with max_angular:=1 ends with exit code 0 (got ${CODE})" equals "${CODE}" 0
stop_robot

echo "Result: ${failures} failure(s)"
[ "${failures}" -eq 0 ]
