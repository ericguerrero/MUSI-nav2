#!/bin/bash
# Motion primitives on the simulated ROSbot 2R: go forward, turn, go backward.
#
# Each move is checked twice. Against /odometry/filtered, which is what the
# node itself closes the loop on: this proves the loop ends where it was told
# to. And against Gazebo's own pose of the robot model, which the node never
# sees: this says how far the robot really went, wheel slip included.
#
# The two true-motion thresholds were set before anything was measured. If one
# fails, that is a finding about slip in the simulation to report, not a number
# to widen.
#
# Runs inside the image. Use tests/run_motion_primitives_sim_test.sh from the host.

EXPECT_FORWARD_M="${EXPECT_FORWARD_M:-1.00}"   # set to something else to see the test fail
STARTUP_TIMEOUT_S="${STARTUP_TIMEOUT_S:-150}"
MODEL="rosbot"
NODE="python3 /ros2_ws/src/motion_primitives.py"

source /opt/ros/humble/setup.bash
if [ -f /opt/rosbot_ws/install/local_setup.bash ]; then
    source /opt/rosbot_ws/install/local_setup.bash
fi

failures=0
pass() { echo "PASS  $1"; }
fail() { echo "FAIL  $1"; failures=$((failures + 1)); }
check() {  # label, then a command whose exit code decides
    local label="$1"; shift
    if "$@"; then pass "${label}"; else fail "${label}"; fi
}

# "x y yaw" as the node sees it.
odom_pose() {
    timeout 8 ros2 topic echo --once /odometry/filtered --field pose.pose 2>/dev/null \
        | python3 -c '
import math, sys, yaml
pose = yaml.safe_load(sys.stdin.read().split("---")[0])
p, q = pose["position"], pose["orientation"]
yaw = math.atan2(2 * (q["w"] * q["z"] + q["x"] * q["y"]), 1 - 2 * (q["y"] ** 2 + q["z"] ** 2))
print(p["x"], p["y"], yaw)'
}

# "x y yaw" as Gazebo knows it.
true_pose() {
    timeout 10 ign model -m "${MODEL}" -p 2>/dev/null | awk '
        /Pose/ { getline xyz; getline rpy
                 gsub(/[\[\]]/, "", xyz); gsub(/[\[\]]/, "", rpy)
                 split(xyz, p, " "); split(rpy, r, " "); print p[1], p[2], r[3] }'
}

# Signed distance covered along the heading the robot had at the start.
moved() {
    python3 -c '
import math, sys
x0, y0, a0, x1, y1, _ = map(float, sys.argv[1:])
print(f"{(x1 - x0) * math.cos(a0) + (y1 - y0) * math.sin(a0):.4f}")' $1 $2
}

# Degrees turned, the short way; fine for the 90 degrees used here.
turned() {
    python3 -c '
import math, sys
_, _, a0, _, _, a1 = map(float, sys.argv[1:])
print(f"{math.degrees(math.atan2(math.sin(a1 - a0), math.cos(a1 - a0))):.2f}")' $1 $2
}

within() {  # value, expected, tolerance
    awk -v v="$1" -v e="$2" -v t="$3" 'BEGIN { d = v - e; if (d < 0) d = -d; exit !(v != "" && d <= t) }'
}

exits_with() {  # expected code, then the command
    local expected="$1"; shift
    "$@" >/tmp/primitive.out 2>&1
    local code=$?
    [ "${code}" -eq "${expected}" ] || { cat /tmp/primitive.out; echo "exit code ${code}"; return 1; }
}

LOG=/tmp/motion_primitives_sim.log
ros2 launch /ros2_ws/src/rosbot_simulation.launch.py world:=empty_with_plugins headless:=true >"${LOG}" 2>&1 &
LAUNCH_PID=$!
trap 'kill -INT ${LAUNCH_PID} 2>/dev/null; sleep 5; kill -9 ${LAUNCH_PID} 2>/dev/null; pkill -9 -f "ign gazebo" 2>/dev/null' EXIT

deadline=$((SECONDS + STARTUP_TIMEOUT_S))
until grep -q "Configured and activated .*rosbot_base_controller" "${LOG}"; do
    if ! kill -0 "${LAUNCH_PID}" 2>/dev/null || [ "${SECONDS}" -ge "${deadline}" ]; then
        fail "simulation starts and activates rosbot_base_controller within ${STARTUP_TIMEOUT_S} s"
        tail -40 "${LOG}"
        echo "Result: ${failures} failure(s)"
        exit 1
    fi
    sleep 2
done
sleep 10

check "bad input is refused with exit code 2" exits_with 2 ${NODE} jump 1
check "no odometry at start gives exit code 3" \
    exits_with 3 timeout 15 ${NODE} forward 1.0 --ros-args -p feedback_topic:=/nothing_here

odom_before="$(odom_pose)"; true_before="$(true_pose)"
check "forward 1.0 ends with exit code 0" exits_with 0 timeout 120 ${NODE} forward 1.0
sleep 2
odom_after="$(odom_pose)"; true_after="$(true_pose)"
odom_m="$(moved "${odom_before}" "${odom_after}")"
true_m="$(moved "${true_before}" "${true_after}")"
check "forward: odometry moved ${odom_m} m (${EXPECT_FORWARD_M} +- 0.03)" within "${odom_m}" "${EXPECT_FORWARD_M}" 0.03
check "forward: the robot really moved ${true_m} m (1.00 +- 10 %)" within "${true_m}" 1.00 0.10

odom_before="${odom_after}"; true_before="${true_after}"
check "turn 90 ends with exit code 0" exits_with 0 timeout 120 ${NODE} turn 90
sleep 2
odom_after="$(odom_pose)"; true_after="$(true_pose)"
odom_deg="$(turned "${odom_before}" "${odom_after}")"
true_deg="$(turned "${true_before}" "${true_after}")"
check "turn: odometry turned ${odom_deg} deg (90 +- 3)" within "${odom_deg}" 90 3
check "turn: the robot really turned ${true_deg} deg (90 +- 20 %)" within "${true_deg}" 90 18

odom_before="${odom_after}"; true_before="${true_after}"
check "forward -0.5 ends with exit code 0" exits_with 0 timeout 120 ${NODE} forward -0.5
sleep 2
odom_after="$(odom_pose)"; true_after="$(true_pose)"
odom_m="$(moved "${odom_before}" "${odom_after}")"
true_m="$(moved "${true_before}" "${true_after}")"
check "backward: odometry moved ${odom_m} m (-0.50 +- 0.03)" within "${odom_m}" -0.50 0.03
echo "INFO  backward: the robot really moved ${true_m} m"

speed="$(timeout 8 ros2 topic echo --once /odometry/filtered --field twist.twist.linear.x 2>/dev/null | head -1)"
check "the robot is at rest afterwards (speed ${speed} m/s)" within "${speed}" 0 0.01

echo "Result: ${failures} failure(s)"
[ "${failures}" -eq 0 ]
