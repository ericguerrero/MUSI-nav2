#!/bin/bash
# Verify ONE demo inside a fresh container. Argument: the demo name.
#
# One container per demo, because sharing a container between demos means
# killing the previous demo's processes, and a broad pkill takes out the ROS
# daemon and the probe commands too. A fresh container is also what a student
# actually does.
set -o pipefail

source /opt/ros/humble/setup.bash
export TURTLEBOT3_MODEL=waffle

DEMO="$1"
LOGDIR=/shared/verification_logs
mkdir -p "${LOGDIR}"
LOG="${LOGDIR}/${DEMO}.log"

wait_for_topic() {
    for _ in $(seq 1 "${2:-150}"); do
        ros2 topic list 2>/dev/null | grep -qx "$1" && return 0
        sleep 1
    done
    return 1
}

has_data() { timeout "${2:-25}" ros2 topic echo "$1" --once >/dev/null 2>&1; }

start_sim() {
    ros2 launch /ros2_ws/src/simulation.launch.py \
        world:="${1:-landmark}" headless:=true rviz:=false >>"${LOG}" 2>&1 &
    wait_for_topic /model_states 150
}

case "${DEMO}" in
    sim_landmark|sim_turtlebot3_world|sim_turtlebot3_house|sim_empty_world)
        WORLD="${DEMO#sim_}"
        start_sim "${WORLD}" || { echo "no /model_states"; exit 1; }
        has_data /scan 30 && has_data /odom 30
        ;;
    groundtruth)
        start_sim landmark || { echo "no /model_states"; exit 1; }
        python3 /ros2_ws/src/groundtruth_pose.py >>"${LOG}" 2>&1 &
        sleep 12
        has_data /groundtruth/pose 25 && has_data /groundtruth/odom 25
        ;;
    nav2)
        start_sim landmark || { echo "no /model_states"; exit 1; }
        ros2 launch /ros2_ws/src/nav2.launch.py use_sim_time:=True >>"${LOG}" 2>&1 &
        sleep 45
        ros2 node list 2>/dev/null | grep -q bt_navigator && has_data /map 30
        ;;
    slam_nav2)
        start_sim landmark || { echo "no /model_states"; exit 1; }
        ros2 launch /ros2_ws/src/slam_nav2.launch.py use_sim_time:=True >>"${LOG}" 2>&1 &
        sleep 50
        has_data /map 40
        ;;
    slam_cartographer)
        start_sim landmark || { echo "no /model_states"; exit 1; }
        ros2 launch /ros2_ws/src/slam_cartographer.launch.py use_sim_time:=True >>"${LOG}" 2>&1 &
        sleep 50
        has_data /map 40
        ;;
    replay_bag)
        BAG=$(ls -d /shared/bags/*/ 2>/dev/null | head -1)
        [ -z "${BAG}" ] && { echo "no bag present"; exit 1; }
        ros2 launch /ros2_ws/src/replay_bag.launch.py bag_path:="${BAG%/}" >>"${LOG}" 2>&1 &
        sleep 20
        has_data /scan 25
        ;;
    *)
        echo "unknown demo: ${DEMO}"; exit 2
        ;;
esac
