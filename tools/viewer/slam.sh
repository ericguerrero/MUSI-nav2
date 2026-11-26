#!/bin/bash
# slam_toolbox for the isolated ROSbot simulation, under a supervisor that can
# forget the map. Needs musi_sim already running.
#
#   ./slam.sh up       start SLAM
#   ./slam.sh forget   drop the map and start mapping again
#   ./slam.sh down     remove SLAM
#
# In the browser, forget is the "Forget map" button in the layout: it
# publishes std_msgs/Empty on /slam/forget_map.
set -euo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO="$(cd "${HERE}/../.." && pwd)"
CONFIGS="${CONFIGS:-${REPO}/shared/configs}"
IMAGE="nav2_student:humble"
NAME="musi_sim_slam"
SETUP="source /opt/ros/humble/setup.bash && source /opt/rosbot_ws/install/local_setup.bash"

case "${1:-}" in
    up)
        docker rm -f "${NAME}" > /dev/null 2>&1 || true
        # slam_toolbox_params.yaml is tuned for the TurtleBot3; the overrides
        # are what the ROSbot needs.
        #
        # SLAM reads /scan_free_space, where rays with no return carry a
        # reading just under the lidar's 10 m maximum. max_laser_range has to
        # stay below that reading for slam_toolbox to mark those rays free.
        docker run -d --name "${NAME}" \
            --network container:musi_sim --ipc container:musi_sim \
            -e ROS_DOMAIN_ID=77 -e ROS_LOCALHOST_ONLY=1 \
            -v "${CONFIGS}:/shared/configs:ro" \
            -v "${HERE}/slam_supervisor.py:/slam_supervisor.py:ro" \
            -v "${HERE}/scan_free_space.py:/scan_free_space.py:ro" \
            "${IMAGE}" bash -c "${SETUP} && \
                (python3 /scan_free_space.py --ros-args -p use_sim_time:=true &) && \
                exec python3 /slam_supervisor.py \
                ros2 run slam_toolbox async_slam_toolbox_node --ros-args \
                --params-file /shared/configs/slam_toolbox_params.yaml \
                -p use_sim_time:=true -p base_frame:=base_link \
                -p scan_topic:=/scan_free_space -p max_laser_range:=9.9 \
                -p map_update_interval:=2.0" > /dev/null
        ;;
    forget)
        docker exec "${NAME}" bash -c "${SETUP} && \
            ros2 topic pub --once /slam/forget_map std_msgs/msg/Empty '{}'" > /dev/null
        ;;
    down)
        docker rm -f "${NAME}" > /dev/null 2>&1 || true
        ;;
    *)
        echo "usage: $0 up | forget | down" >&2
        exit 2
        ;;
esac
