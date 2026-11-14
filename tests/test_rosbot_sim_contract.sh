#!/bin/bash
# Contract test for the ROSbot 2R simulation.
#
# Student controllers must run unchanged on the lab robot, so the simulation
# has to publish what the robot publishes. The topic list below was measured on
# the physical ROSbot 2R on 2026-11-14 (driver husarion/rosbot:humble-0.14.0),
# not taken from Husarion's ROS_API.md, which names topics the robot does not
# use (imu/data, battery_state). /scan is included although the robot had no
# LiDAR driver running yet: the RPLIDAR A2 publishes it once its container runs.
# /battery is hardware-only and deliberately absent.
#
# Runs inside the image. Use tests/run_rosbot_sim_contract_test.sh from the host.

MIN_ODOM_HZ="${MIN_ODOM_HZ:-15}"
MIN_RTF="${MIN_RTF:-0.7}"
WORLD="${WORLD:-husarion_world}"
STARTUP_TIMEOUT_S="${STARTUP_TIMEOUT_S:-150}"

CONTRACT_TOPICS=(
    "/cmd_vel geometry_msgs/msg/Twist"
    "/odometry/filtered nav_msgs/msg/Odometry"
    "/imu_broadcaster/imu sensor_msgs/msg/Imu"
    "/joint_states sensor_msgs/msg/JointState"
    "/scan sensor_msgs/msg/LaserScan"
    "/tf tf2_msgs/msg/TFMessage"
    "/tf_static tf2_msgs/msg/TFMessage"
)

source /opt/ros/humble/setup.bash
if [ -f /opt/rosbot_ws/install/local_setup.bash ]; then
    source /opt/rosbot_ws/install/local_setup.bash
fi

failures=0
pass() { echo "PASS  $1"; }
fail() { echo "FAIL  $1"; failures=$((failures + 1)); }

if ! ros2 pkg prefix rosbot_gazebo >/dev/null 2>&1; then
    fail "rosbot_gazebo is installed in the image"
    echo "Result: ${failures} failure(s)"
    exit 1
fi
pass "rosbot_gazebo is installed in the image"

LOG=/tmp/rosbot_sim_contract.log
ros2 launch /ros2_ws/src/rosbot_simulation.launch.py world:="${WORLD}" headless:=true >"${LOG}" 2>&1 &
LAUNCH_PID=$!
trap 'kill -INT ${LAUNCH_PID} 2>/dev/null; sleep 5; kill -9 ${LAUNCH_PID} 2>/dev/null; pkill -9 -f "ign gazebo" 2>/dev/null' EXIT

# The base controller is the last thing to come up, and /odometry/filtered
# depends on it through the EKF.
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
pass "simulation starts and activates rosbot_base_controller"
sleep 10

topics="$(ros2 topic list -t)"
for entry in "${CONTRACT_TOPICS[@]}"; do
    name="${entry% *}"
    type="${entry#* }"
    if grep -qx "${name} \[${type}\]" <<<"${topics}"; then
        pass "topic ${name} [${type}]"
    else
        fail "topic ${name} [${type}]"
    fi
done

odom_hz="$(timeout 8 ros2 topic hz /odometry/filtered 2>/dev/null | awk '/average rate/ {r=$3} END {print r+0}')"
if awk -v r="${odom_hz}" -v m="${MIN_ODOM_HZ}" 'BEGIN {exit !(r >= m)}'; then
    pass "/odometry/filtered at ${odom_hz} Hz (>= ${MIN_ODOM_HZ})"
else
    fail "/odometry/filtered at ${odom_hz} Hz (>= ${MIN_ODOM_HZ})"
fi

rtf="$(timeout 10 ign topic -e -t "/world/${WORLD}/stats" -n 10 2>/dev/null \
    | awk '/real_time_factor/ {s += $2; n++} END {if (n) printf "%.2f", s / n; else print 0}')"
if awk -v r="${rtf}" -v m="${MIN_RTF}" 'BEGIN {exit !(r >= m)}'; then
    pass "real-time factor ${rtf} (>= ${MIN_RTF})"
else
    fail "real-time factor ${rtf} (>= ${MIN_RTF})"
fi

# A controller only works if /cmd_vel moves the robot and the motion shows up
# in /odometry/filtered, the topic it reads back on the real robot.
odom_x() {
    timeout 5 ros2 topic echo --once /odometry/filtered 2>/dev/null \
        | awk '/position:/ {p = 1} p && /x:/ {print $2; exit}'
}
x_before="$(odom_x)"
timeout 8 ros2 topic pub -r 10 -t 30 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.2}}" >/dev/null
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{}" >/dev/null
x_after="$(odom_x)"
if awk -v a="${x_before}" -v b="${x_after}" 'BEGIN {exit !(a != "" && b != "" && b - a > 0.3)}'; then
    pass "0.2 m/s for 3 s on /cmd_vel moves /odometry/filtered x from ${x_before} to ${x_after}"
else
    fail "0.2 m/s for 3 s on /cmd_vel moves /odometry/filtered x more than 0.3 m (${x_before} -> ${x_after})"
fi

echo "Result: ${failures} failure(s)"
[ "${failures}" -eq 0 ]
