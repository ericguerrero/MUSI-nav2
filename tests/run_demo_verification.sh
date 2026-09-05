#!/bin/bash
# Verify every demo, each in its own container, and emit a Markdown table.
# Takes roughly 20 minutes. Logs land in shared/verification_logs/.
REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

DEMOS=(sim_landmark sim_turtlebot3_world sim_turtlebot3_house sim_empty_world
       groundtruth nav2 slam_nav2 slam_cartographer replay_bag)

declare -A RESULT
for demo in "${DEMOS[@]}"; do
    echo "== ${demo}" >&2
    if timeout 400 docker run --rm --network host \
        -v "${REPO}/ros2_ws:/ros2_ws" \
        -v "${REPO}/shared:/shared" \
        -v "${REPO}/tests:/tests:ro" \
        -e TURTLEBOT3_MODEL=waffle \
        nav2_student:humble \
        bash /tests/verify_one_demo.sh "${demo}" >/dev/null 2>&1
    then
        RESULT[${demo}]=PASS
        echo "   PASS" >&2
    else
        RESULT[${demo}]=FAIL
        echo "   FAIL" >&2
    fi
done

echo
echo "| Demo | Result |"
echo "|---|---|"
for demo in "${DEMOS[@]}"; do
    echo "| \`${demo}\` | ${RESULT[${demo}]} |"
done
