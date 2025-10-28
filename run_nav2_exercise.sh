#!/bin/bash
# Helper script to run Exercise 3b: Nav2 Waypoint Following

set -e

echo "=========================================="
echo "Exercise 3b: Nav2 Waypoint Following"
echo "=========================================="
echo ""
echo "This will launch:"
echo "  1. Gazebo simulation (turtlebot3_world)"
echo "  2. Nav2 navigation stack"
echo "  3. Waypoint follower with drift tracking"
echo ""
echo "The robot will navigate a 2m × 2m square and"
echo "demonstrate drift measurement (odom vs map)."
echo ""
echo "Press Ctrl+C to stop at any time."
echo "=========================================="
echo ""

# Check if container is running
if ! docker ps | grep -q nav2_workspace; then
    echo "Error: Docker container not running!"
    echo "Please run: ./run.sh"
    exit 1
fi

echo "Starting navigation..."
echo ""

# Run launch file inside container
docker exec -it nav2_workspace bash -c "
    source /opt/ros/humble/setup.bash &&
    cd /ros2_ws/src &&
    ros2 launch ex3_nav2_drift.launch.py
"
