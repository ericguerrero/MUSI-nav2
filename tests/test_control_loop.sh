#!/bin/bash
# End-to-end test of the control loop.
#
# Runs simulation -> groundtruth -> controller -> analyzer -> visualizer and
# checks that the robot moved, the run was scored, and a plot came out.
#
# Runs inside the container. Takes about 3 minutes.
set -o pipefail

PASS=0
FAIL=0
OUT=/shared/student_work/control_loop_test

check() {
    local name="$1"; shift
    if "$@" >/dev/null 2>&1; then
        echo "  PASS: ${name}"; PASS=$((PASS + 1))
    else
        echo "  FAIL: ${name}"; FAIL=$((FAIL + 1))
    fi
}

source /opt/ros/humble/setup.bash
export TURTLEBOT3_MODEL=waffle
mkdir -p /shared/student_work
rm -f "${OUT}".csv "${OUT}".json "${OUT}".png

echo "Launching simulation..."
ros2 launch /ros2_ws/src/simulation.launch.py world:=empty_world headless:=true \
    rviz:=false >/tmp/sim.log 2>&1 &
for _ in $(seq 1 150); do
    ros2 topic list 2>/dev/null | grep -qx /model_states && break
    sleep 1
done

python3 /ros2_ws/src/groundtruth_pose.py >/tmp/gt.log 2>&1 &
sleep 10

# Record the starting pose so we can prove the controller actually drove.
START=$(timeout 15 ros2 topic echo /groundtruth/pose --once 2>/dev/null \
        | grep -A2 position | grep -m1 'x:' | awk '{print $2}')

echo "Starting controller and analyzer..."
python3 /ros2_ws/src/trajectory_analyzer.py --ros-args \
    -p output:="${OUT}" >/tmp/analyzer.log 2>&1 &
ANALYZER_PID=$!
python3 /ros2_ws/src/controller_template.py --ros-args \
    -p reference:=circle -p radius:=1.0 -p speed:=0.15 >/tmp/controller.log 2>&1 &
CONTROLLER_PID=$!

sleep 40

echo "Checking the live loop..."
check "/cmd_vel publishes" timeout 10 ros2 topic echo /cmd_vel --once
check "/reference/pose publishes" timeout 10 ros2 topic echo /reference/pose --once

END=$(timeout 15 ros2 topic echo /groundtruth/pose --once 2>/dev/null \
      | grep -A2 position | grep -m1 'x:' | awk '{print $2}')
if [ -n "${START}" ] && [ -n "${END}" ] && \
   awk "BEGIN{exit !(($END - $START) < -0.05 || ($END - $START) > 0.05)}"; then
    echo "  PASS: robot moved (x ${START} -> ${END})"; PASS=$((PASS + 1))
else
    echo "  FAIL: robot did not move (x ${START:-?} -> ${END:-?})"; FAIL=$((FAIL + 1))
fi

# SIGINT, not SIGKILL: the analyzer scores the run on clean shutdown.
kill -INT "${CONTROLLER_PID}" 2>/dev/null
kill -INT "${ANALYZER_PID}" 2>/dev/null
sleep 8

echo "Checking the scored run..."
check "analyzer wrote CSV" test -s "${OUT}.csv"
check "analyzer wrote metrics JSON" test -s "${OUT}.json"
check "metrics contain a finite RMSE" bash -c \
    "python3 -c \"import json,math; v=json.load(open('${OUT}.json'))['position_rmse_m']; assert math.isfinite(v) and v>=0\""

python3 /ros2_ws/src/visualize_trajectory.py "${OUT}.csv" >/tmp/viz.log 2>&1
check "visualizer wrote PNG" test -s "${OUT}.png"

echo
echo "passed: ${PASS}  failed: ${FAIL}"
[ -f "${OUT}.json" ] && cat "${OUT}.json"
pkill -f gzserver >/dev/null 2>&1
if [ "${FAIL}" -eq 0 ]; then exit 0; fi
exit 1
