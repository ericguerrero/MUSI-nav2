#!/bin/bash
# drive_test.py against one simulated robot: turtlebot3, rosbot,
# rosbot_mecanum or ackermann, chosen with ROBOT.
#
# The checks are the drive properties the lab is about, with wide margins: every
# robot drives straight, all but the Ackermann robot spin on the spot, only the
# mecanum robot moves sideways, and the Ackermann robot turns no tighter than
# its minimum radius. They are not the reference numbers of the worksheet; those
# vary by a few percent from run to run.
#
# Runs inside the image. Use tests/run_drive_test_sim_test.sh from the host.

ROBOT="${ROBOT:-rosbot}"
SPINS=yes
# The arc is commanded at 0.2 m/s and 0.5 rad/s, a radius of 0.40 m.
ARC_RADIUS_M=0.40
STARTUP_TIMEOUT_S="${STARTUP_TIMEOUT_S:-150}"
NODE="python3 /ros2_ws/src/drive_test.py --ros-args -p robot:=${ROBOT}"

source /opt/ros/humble/setup.bash
if [ -f /opt/rosbot_ws/install/local_setup.bash ]; then
    source /opt/rosbot_ws/install/local_setup.bash
fi

case "${ROBOT}" in
    turtlebot3)
        export TURTLEBOT3_MODEL=waffle
        LAUNCH="/ros2_ws/src/simulation.launch.py world:=empty_world headless:=true rviz:=false"
        READY="spawn_entity.*process has finished cleanly"
        SPEED_TOPIC=/odom
        SIDEWAYS_M=0.0 ;;
    rosbot)
        LAUNCH="/ros2_ws/src/rosbot_simulation.launch.py world:=empty_with_plugins headless:=true"
        READY="Configured and activated .*rosbot_base_controller"
        SPEED_TOPIC=/rosbot_base_controller/odom
        SIDEWAYS_M=0.0 ;;
    rosbot_mecanum)
        LAUNCH="/ros2_ws/src/rosbot_simulation.launch.py world:=empty_with_plugins headless:=true mecanum:=true"
        READY="Configured and activated .*rosbot_base_controller"
        SPEED_TOPIC=/rosbot_base_controller/odom
        SIDEWAYS_M=1.0 ;;
    ackermann)
        LAUNCH="/ros2_ws/src/ackermann_simulation.launch.py headless:=true"
        READY="Configured and activated .*ackermann_steering_controller"
        SPEED_TOPIC=/ackermann_steering_controller/odometry
        SIDEWAYS_M=0.0
        SPINS=no
        # min_turn_radius in ackermann.urdf.xacro: the commanded 0.40 m is
        # tighter than the steering allows.
        ARC_RADIUS_M=0.50 ;;
    *)
        echo "ROBOT must be turtlebot3, rosbot, rosbot_mecanum or ackermann, got '${ROBOT}'"
        exit 2 ;;
esac

failures=0
pass() { echo "PASS  $1"; }
fail() { echo "FAIL  $1"; failures=$((failures + 1)); }
check() {  # label, then a command whose exit code decides
    local label="$1"; shift
    if "$@"; then pass "${label}"; else fail "${label}"; fi
}

within() {  # value, expected, tolerance
    awk -v v="$1" -v e="$2" -v t="$3" 'BEGIN { d = v - e; if (d < 0) d = -d; exit !(v != "" && d <= t) }'
}

at_least() {  # value, minimum
    awk -v v="$1" -v m="$2" 'BEGIN { exit !(v != "" && v >= m) }'
}

exits_with() {  # expected code, then the command; output kept in /tmp/drive_test.out
    local expected="$1"; shift
    "$@" >/tmp/drive_test.out 2>&1
    local code=$?
    [ "${code}" -eq "${expected}" ] || { cat /tmp/drive_test.out; echo "exit code ${code}"; return 1; }
}

# One number from the table: manoeuvre, block (2 commanded, 3 true, 4 wheel
# odometry), column inside the block (1 forward, 2 left, 3 turn).
cell() {
    awk -F'|' -v name="$1" -v block="$2" -v column="$3" \
        '$1 ~ "^" name " " { split($block, n, " "); print n[column] }' /tmp/drive_test.out
}

# Radius of the circle through the start and the end of the arc, from the true
# motion: chord / (2 sin(turn / 2)).
arc_radius() {
    awk -v f="$(cell arc 3 1)" -v l="$(cell arc 3 2)" -v deg="$(cell arc 3 3)" 'BEGIN {
        half = deg * 3.14159265 / 360
        if (f == "" || half < 0.05) exit
        printf "%.3f", sqrt(f * f + l * l) / (2 * sin(half)) }'
}

# Largest of the robot's linear and angular speed, from one odometry message.
speed() {
    timeout 10 python3 -c '
import sys, rclpy
from nav_msgs.msg import Odometry
from rclpy.qos import qos_profile_sensor_data
rclpy.init()
node = rclpy.create_node("speed_probe")
seen = []
node.create_subscription(Odometry, sys.argv[1], seen.append, qos_profile_sensor_data)
while not seen:
    rclpy.spin_once(node, timeout_sec=0.2)
t = seen[-1].twist.twist
print(max(abs(t.linear.x), abs(t.linear.y), abs(t.angular.z)))' "${SPEED_TOPIC}" 2>/dev/null
}

check "an unknown robot is refused with exit code 2" \
    exits_with 2 python3 /ros2_ws/src/drive_test.py --ros-args -p robot:=tank
check "no simulation gives exit code 3" exits_with 3 timeout 60 ${NODE}

LOG=/tmp/drive_test_sim.log
ros2 launch ${LAUNCH} >"${LOG}" 2>&1 &
LAUNCH_PID=$!
trap 'kill -INT ${LAUNCH_PID} 2>/dev/null; sleep 5; kill -9 ${LAUNCH_PID} 2>/dev/null; pkill -9 -f "ign gazebo" 2>/dev/null; pkill -9 gzserver 2>/dev/null' EXIT

deadline=$((SECONDS + STARTUP_TIMEOUT_S))
until grep -q "${READY}" "${LOG}"; do
    if ! kill -0 "${LAUNCH_PID}" 2>/dev/null || [ "${SECONDS}" -ge "${deadline}" ]; then
        fail "simulation of ${ROBOT} starts within ${STARTUP_TIMEOUT_S} s"
        tail -40 "${LOG}"
        echo "Result: ${failures} failure(s)"
        exit 1
    fi
    sleep 2
done
sleep 10

check "all four manoeuvres end with exit code 0" exits_with 0 timeout 180 ${NODE}
cat /tmp/drive_test.out

for manoeuvre in straight spin arc sideways; do
    check "table has a row for ${manoeuvre}" test -n "$(cell ${manoeuvre} 3 1)"
done

true_m="$(cell straight 3 1)"; odom_m="$(cell straight 4 1)"
check "straight: really moved ${true_m} m forward (1.00 +- 0.10)" within "${true_m}" 1.0 0.10
check "straight: wheel odometry ${odom_m} m agrees with the truth (+- 0.05)" within "${odom_m}" "${true_m}" 0.05

true_deg="$(cell spin 3 3)"
if [ "${SPINS}" = yes ]; then
    check "spin: really turned ${true_deg} deg (at least 90)" at_least "${true_deg}" 90
else
    check "spin: did not turn, ${true_deg} deg (0 +- 3)" within "${true_deg}" 0 3
fi
check "spin: stayed on the spot, forward $(cell spin 3 1) m (0 +- 0.10)" within "$(cell spin 3 1)" 0 0.10
check "spin: stayed on the spot, left $(cell spin 3 2) m (0 +- 0.10)" within "$(cell spin 3 2)" 0 0.10

check "arc: really turned $(cell arc 3 3) deg (at least 60)" at_least "$(cell arc 3 3)" 60
check "arc: really moved $(cell arc 3 2) m to the left (at least 0.3)" at_least "$(cell arc 3 2)" 0.3
radius="$(arc_radius)"
check "arc: true radius ${radius} m (${ARC_RADIUS_M} +- 0.08)" within "${radius}" "${ARC_RADIUS_M}" 0.08
if [ "${SPINS}" = no ]; then
    # Below the minimum only by measurement noise, as in test_ackermann_sim.sh.
    check "arc: true radius ${radius} m is not below the minimum (0.50 - 0.03)" at_least "${radius}" 0.47
fi

true_m="$(cell sideways 3 2)"
check "sideways: really moved ${true_m} m to the left (${SIDEWAYS_M} +- 0.15)" within "${true_m}" "${SIDEWAYS_M}" 0.15

speed_after="$(speed)"
check "the robot is at rest afterwards (speed ${speed_after})" within "${speed_after}" 0 0.01

# An interrupt in the middle of the straight line must leave the robot stopped.
# The TurtleBot3 keeps its last command for ever, so this fails without the
# zero commands drive_test.py sends on its way out.
${NODE} >/tmp/drive_test_interrupted.out 2>&1 &
NODE_PID=$!
# The header is printed just before the first command; the straight line then
# lasts 5 s, which leaves room for the 2 s here and the probe's own start.
timeout 60 bash -c 'until grep -q "^manoeuvre" /tmp/drive_test_interrupted.out; do sleep 0.2; done'
sleep 2
moving="$(speed)"
kill -INT "${NODE_PID}"
wait "${NODE_PID}"
code=$?
check "interrupted run ends with exit code 130 (got ${code})" test "${code}" -eq 130
check "the robot was moving when interrupted (speed ${moving})" at_least "${moving}" 0.05
sleep 3
speed_after="$(speed)"
check "the robot is at rest after the interrupt (speed ${speed_after})" within "${speed_after}" 0 0.01

echo "Result: ${failures} failure(s)"
[ "${failures}" -eq 0 ]
