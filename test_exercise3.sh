#!/bin/bash
# Exercise 3 Testing Script
# This script runs a complete test of the odometry drift waypoint follower

set -e

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
BLUE='\033[0;34m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

echo -e "${BLUE}========================================${NC}"
echo -e "${BLUE}Exercise 3: Odometry Drift Test${NC}"
echo -e "${BLUE}========================================${NC}"
echo ""

# Check if container is running
if ! docker ps | grep -q nav2_workspace; then
    echo -e "${RED}Error: nav2_workspace container is not running${NC}"
    echo -e "${YELLOW}Run: ./run.sh${NC}"
    exit 1
fi

echo -e "${GREEN}✓ Docker container is running${NC}"

# Check if script exists
if [ ! -f "ros2_ws/src/simple_odom_waypoint.py" ]; then
    echo -e "${RED}Error: simple_odom_waypoint.py not found${NC}"
    exit 1
fi

echo -e "${GREEN}✓ Waypoint follower script exists${NC}"

# Check if RViz config exists
if [ ! -f "shared/configs/ex3_odometry_drift.rviz" ]; then
    echo -e "${RED}Error: RViz config not found${NC}"
    exit 1
fi

echo -e "${GREEN}✓ RViz configuration exists${NC}"
echo ""

echo -e "${YELLOW}========================================${NC}"
echo -e "${YELLOW}Manual Testing Instructions:${NC}"
echo -e "${YELLOW}========================================${NC}"
echo ""

echo -e "${BLUE}Step 1: Launch Gazebo Simulation${NC}"
echo -e "Open a new terminal and run:"
echo -e "${GREEN}docker exec -it nav2_workspace bash${NC}"
echo -e "${GREEN}ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py${NC}"
echo ""
echo -e "Wait until you see: ${YELLOW}[INFO] [gazebo_ros_node]: ROS was initialized${NC}"
echo ""

echo -e "${BLUE}Step 2: Launch RViz2 (Optional but Recommended)${NC}"
echo -e "Open another terminal and run:"
echo -e "${GREEN}docker exec -it nav2_workspace bash${NC}"
echo -e "${GREEN}rviz2 -d /shared/configs/ex3_odometry_drift.rviz${NC}"
echo ""
echo -e "You should see:"
echo -e "  - Grid with 1m cells"
echo -e "  - TurtleBot3 robot at origin"
echo -e "  - LaserScan data"
echo ""

echo -e "${BLUE}Step 3: Run Waypoint Follower${NC}"
echo -e "Open a third terminal and run:"
echo -e "${GREEN}docker exec -it nav2_workspace bash${NC}"
echo -e "${GREEN}cd /ros2_ws/src${NC}"
echo -e "${GREEN}python3 simple_odom_waypoint.py${NC}"
echo ""

echo -e "${YELLOW}========================================${NC}"
echo -e "${YELLOW}Expected Behavior:${NC}"
echo -e "${YELLOW}========================================${NC}"
echo ""

echo -e "1. Robot navigates a 1m × 1m square:"
echo -e "   ${GREEN}→${NC} Forward 1m (Point A)"
echo -e "   ${GREEN}→${NC} Right 1m (Point B)"
echo -e "   ${GREEN}→${NC} Backward 1m (Point C)"
echo -e "   ${GREEN}→${NC} Left 1m to start (Point D)"
echo ""

echo -e "2. Console output shows:"
echo -e "   ${GREEN}✓${NC} Start position recorded"
echo -e "   ${GREEN}✓${NC} Waypoint progress (1/4, 2/4, 3/4, 4/4)"
echo -e "   ${GREEN}✓${NC} Navigation status (distance, angle)"
echo ""

echo -e "3. Final drift analysis:"
echo -e "   ${GREEN}✓${NC} Expected final: (0.000, 0.000)"
echo -e "   ${YELLOW}!${NC} Actual final: (~0.3, ~-0.2) ${RED}<-- DRIFT!${NC}"
echo -e "   ${RED}✓${NC} Total drift: ~0.3-0.4 meters"
echo ""

echo -e "4. In RViz2 (if running):"
echo -e "   ${GREEN}✓${NC} Red arrow trail showing robot path"
echo -e "   ${GREEN}✓${NC} Covariance ellipses (uncertainty)"
echo -e "   ${YELLOW}!${NC} Final position NOT at origin!"
echo ""

echo -e "${YELLOW}========================================${NC}"
echo -e "${YELLOW}Success Criteria:${NC}"
echo -e "${YELLOW}========================================${NC}"
echo ""

echo -e "  ${GREEN}[✓]${NC} Script executes without errors"
echo -e "  ${GREEN}[✓]${NC} Robot completes all 4 waypoints"
echo -e "  ${GREEN}[✓]${NC} Drift analysis shows 0.2-0.5m error"
echo -e "  ${GREEN}[✓]${NC} Educational logging is clear"
echo -e "  ${GREEN}[✓]${NC} Script exits cleanly"
echo ""

echo -e "${BLUE}========================================${NC}"
echo -e "${BLUE}Verification Commands:${NC}"
echo -e "${BLUE}========================================${NC}"
echo ""

echo -e "Check ROS2 topics (in container):"
echo -e "${GREEN}ros2 topic list | grep -E '(odom|cmd_vel|scan)'${NC}"
echo ""
echo -e "Expected output:"
echo -e "  /cmd_vel"
echo -e "  /odom"
echo -e "  /scan"
echo ""

echo -e "Echo odometry (watch robot position):"
echo -e "${GREEN}ros2 topic echo /odom --once${NC}"
echo ""

echo -e "Monitor velocity commands:"
echo -e "${GREEN}ros2 topic echo /cmd_vel${NC}"
echo ""

echo -e "${YELLOW}========================================${NC}"
echo -e "${YELLOW}Troubleshooting:${NC}"
echo -e "${YELLOW}========================================${NC}"
echo ""

echo -e "Robot doesn't move:"
echo -e "  → Check Gazebo is fully loaded"
echo -e "  → Verify /cmd_vel is being published"
echo -e "  → Check /odom is publishing (ros2 topic hz /odom)"
echo ""

echo -e "No GUI windows:"
echo -e "  → Run: xhost +local:docker"
echo -e "  → Check DISPLAY: echo \$DISPLAY"
echo -e "  → Test X11: docker exec nav2_workspace xeyes"
echo ""

echo -e "Script exits immediately:"
echo -e "  → Wait 5-10 seconds for Gazebo to publish /odom"
echo -e "  → Check logs: ros2 topic hz /odom"
echo ""

echo -e "${GREEN}========================================${NC}"
echo -e "${GREEN}Ready to test! Follow the steps above.${NC}"
echo -e "${GREEN}========================================${NC}"
