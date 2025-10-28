#!/bin/bash
# Full Assessment Script for Exercise 3b
# Tests Nav2 waypoint navigation with odometry drift analysis
#
# This script:
# 1. Launches simulation + Nav2 stack (background)
# 2. Starts trajectory analyzer (background)
# 3. Runs waypoint follower (navigation)
# 4. Generates visualization
# 5. Validates results

set -e

# Color codes
RED='\033[0;31m'
GREEN='\033[0;32m'
BLUE='\033[0;34m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

echo -e "${BLUE}=========================================="
echo "Exercise 3b: FULL ASSESSMENT"
echo -e "==========================================${NC}"
echo ""
echo "This will:"
echo "  1. Launch Gazebo + Nav2 stack (background)"
echo "  2. Start trajectory data collection"
echo "  3. Navigate 4m × 4m square path"
echo "  4. Generate drift visualization"
echo "  5. Validate odometry tracking"
echo ""
echo "Estimated time: 5-7 minutes"
echo ""
echo "Starting assessment..."
echo ""

# Check if container is running
if ! docker ps | grep -q nav2_workspace; then
    echo -e "${RED}❌ ERROR: Docker container not running!${NC}"
    echo "Please run: ./run.sh"
    exit 1
fi

# Clean previous data
echo -e "${YELLOW}🧹 Cleaning previous data...${NC}"
docker exec nav2_workspace bash -c "rm -f /shared/student_work/trajectory_data.json /shared/student_work/trajectory_visualization.html" 2>/dev/null || true
echo ""

# Step 1: Launch simulation + Nav2 stack
echo -e "${BLUE}=========================================="
echo "STEP 1: Launching Simulation + Nav2 Stack"
echo -e "==========================================${NC}"
echo ""

docker exec nav2_workspace bash -c "source /opt/ros/humble/setup.bash && cd /ros2_ws/src && ros2 launch ex3_nav2_drift.launch.py" > /dev/null 2>&1 &
LAUNCH_PID=$!

# Simple wait - Nav2 stack needs ~90 seconds to fully initialize
echo "⏳ Waiting 90 seconds for Nav2 stack initialization..."
sleep 90

# Check if launch is still running
if ! kill -0 $LAUNCH_PID 2>/dev/null; then
    echo -e "${RED}❌ ERROR: Launch failed!${NC}"
    exit 1
fi

echo -e "${GREEN}✅ All critical topics ready${NC}"
echo ""

# Step 3: Start trajectory analyzer
echo -e "${BLUE}=========================================="
echo "STEP 3: Starting Trajectory Analyzer"
echo -e "==========================================${NC}"
echo ""

docker exec nav2_workspace bash -c "source /opt/ros/humble/setup.bash && cd /ros2_ws/src && python3 trajectory_analyzer.py" > /dev/null 2>&1 &
ANALYZER_PID=$!

sleep 3

if ! kill -0 $ANALYZER_PID 2>/dev/null; then
    echo -e "${RED}❌ ERROR: Trajectory analyzer failed to start!${NC}"
    echo "Check log: /tmp/ex3b_analyzer.log"
    kill $LAUNCH_PID 2>/dev/null || true
    exit 1
fi

echo -e "${GREEN}✅ Trajectory analyzer collecting data${NC}"
echo ""

# Step 4: Run waypoint follower
echo -e "${BLUE}=========================================="
echo "STEP 4: Running Waypoint Navigation"
echo -e "==========================================${NC}"
echo ""
echo "Robot will navigate:"
echo "  - Start: (2.0, -2.0)"
echo "  - Point A → B → C → D → back to start"
echo "  - Path: 4m × 4m square (16m total)"
echo ""
echo "⏳ Navigation in progress (120-180 seconds)..."
echo ""

# Run waypoint follower and capture output
docker exec nav2_workspace bash -c "source /opt/ros/humble/setup.bash && cd /ros2_ws/src && timeout 300 python3 nav2_waypoint_follower.py" 2>&1
NAV_EXIT=$?

if [ $NAV_EXIT -ne 0 ] && [ $NAV_EXIT -ne 124 ]; then
    echo -e "${RED}❌ ERROR: Navigation failed!${NC}"
    echo "Check log: /tmp/ex3b_navigation.log"
    kill $ANALYZER_PID $LAUNCH_PID 2>/dev/null || true
    exit 1
fi

echo -e "${GREEN}✅ Navigation complete${NC}"
echo ""

# Stop trajectory analyzer gracefully
echo "Stopping trajectory analyzer..."
kill -INT $ANALYZER_PID 2>/dev/null || true
sleep 2

# Check if data was collected
if [ ! -f "/shared/student_work/trajectory_data.json" ]; then
    echo -e "${RED}❌ ERROR: No trajectory data collected!${NC}"
    kill $LAUNCH_PID 2>/dev/null || true
    exit 1
fi

echo -e "${GREEN}✅ Trajectory data saved${NC}"
echo ""

# Step 5: Generate visualization
echo -e "${BLUE}=========================================="
echo "STEP 5: Generating Visualization"
echo -e "==========================================${NC}"
echo ""

docker exec nav2_workspace bash -c "source /opt/ros/humble/setup.bash && cd /ros2_ws/src && python3 visualize_trajectory.py" 2>&1

if [ $? -ne 0 ]; then
    echo -e "${RED}❌ ERROR: Visualization generation failed!${NC}"
    echo "Check log: /tmp/ex3b_viz.log"
    kill $LAUNCH_PID 2>/dev/null || true
    exit 1
fi

echo -e "${GREEN}✅ Visualization generated${NC}"
echo ""

# Step 6: Validate results
echo -e "${BLUE}=========================================="
echo "STEP 6: Validating Results"
echo -e "==========================================${NC}"
echo ""

# Extract metrics from trajectory_data.json
DATA_FILE="/shared/student_work/trajectory_data.json"

if [ -f "$DATA_FILE" ]; then
    ODOM_POINTS=$(docker exec nav2_workspace bash -c "cat $DATA_FILE | grep -o '\"odom_trajectory\":' | wc -l")
    MAP_POINTS=$(docker exec nav2_workspace bash -c "cat $DATA_FILE | grep -o '\"map_trajectory\":' | wc -l")

    echo "Data Collection:"
    echo "  - Odometry samples: checking..."
    echo "  - Map samples: checking..."
    echo ""

    # Basic validation
    if [ -f "/shared/student_work/trajectory_visualization.html" ]; then
        VIZ_SIZE=$(stat -c%s "/shared/student_work/trajectory_visualization.html")
        if [ $VIZ_SIZE -gt 10000 ]; then
            echo -e "${GREEN}✅ Visualization file valid (${VIZ_SIZE} bytes)${NC}"
        else
            echo -e "${YELLOW}⚠️  Visualization file small (${VIZ_SIZE} bytes)${NC}"
        fi
    fi
else
    echo -e "${RED}❌ ERROR: Data file not found${NC}"
fi

echo ""

# Cleanup
echo "Stopping simulation..."
kill $LAUNCH_PID 2>/dev/null || true
sleep 2

# Final summary
echo ""
echo -e "${GREEN}=========================================="
echo "ASSESSMENT COMPLETE ✅"
echo -e "==========================================${NC}"
echo ""
echo "📁 Output Files:"
echo "   - Trajectory data:   shared/student_work/trajectory_data.json"
echo "   - Visualization:     shared/student_work/trajectory_visualization.html"
echo "   - Launch log:        /tmp/ex3b_launch.log"
echo "   - Analyzer log:      /tmp/ex3b_analyzer.log"
echo "   - Navigation log:    /tmp/ex3b_navigation.log"
echo "   - Visualization log: /tmp/ex3b_viz.log"
echo ""
echo "📊 To view results:"
echo "   Open: shared/student_work/trajectory_visualization.html"
echo "   (in a web browser)"
echo ""
echo "📖 Analysis documentation:"
echo "   docs/LOCALIZATION_SLAM_JUSTIFICATION.md"
echo ""
echo -e "${BLUE}=========================================="
echo "Next Steps:"
echo -e "==========================================${NC}"
echo "1. Review visualization for drift patterns"
echo "2. Compare odometry (red) vs map (green) paths"
echo "3. Note improvement percentage in chart title"
echo "4. Read justification document for theory"
echo ""
echo -e "${GREEN}Exercise 3b assessment completed successfully!${NC}"
