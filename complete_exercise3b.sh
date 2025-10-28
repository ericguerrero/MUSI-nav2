#!/bin/bash
# Complete Exercise 3b Analysis Script
#
# This script:
# 1. Waits for trajectory data collection to complete
# 2. Generates Plotly visualization
# 3. Displays analysis results

set -e

echo "=================================================="
echo "Exercise 3b: Complete Analysis"
echo "=================================================="
echo ""

# Check if data file exists
DATA_FILE="/shared/student_work/trajectory_data.json"

if [ ! -f "$DATA_FILE" ]; then
    echo "⏳ Waiting for trajectory data collection..."
    echo "   (This takes ~2 minutes during navigation)"
    echo ""

    # Wait up to 150 seconds for data file
    for i in {1..30}; do
        if [ -f "$DATA_FILE" ]; then
            break
        fi
        echo -n "."
        sleep 5
    done
    echo ""
fi

if [ ! -f "$DATA_FILE" ]; then
    echo "❌ ERROR: Trajectory data not found!"
    echo "   Make sure nav2_waypoint_follower.py completed navigation"
    exit 1
fi

echo "✅ Trajectory data collected"
echo ""

# Generate visualization
echo "📊 Generating Plotly visualization..."
docker exec nav2_workspace bash -c "
    source /opt/ros/humble/setup.bash &&
    cd /ros2_ws/src &&
    python3 visualize_trajectory.py
"

if [ $? -eq 0 ]; then
    echo ""
    echo "✅ Visualization complete!"
    echo ""
    echo "=================================================="
    echo "Results:"
    echo "=================================================="
    echo ""
    echo "📁 Output files:"
    echo "   - Trajectory data: /shared/student_work/trajectory_data.json"
    echo "   - Visualization:   /shared/student_work/trajectory_visualization.html"
    echo "   - Analysis doc:    docs/LOCALIZATION_SLAM_JUSTIFICATION.md"
    echo ""
    echo "📖 To view visualization:"
    echo "   Open: shared/student_work/trajectory_visualization.html"
    echo "   (in a web browser)"
    echo ""
    echo "=================================================="
    echo "Exercise 3b: COMPLETE ✅"
    echo "=================================================="
else
    echo "❌ ERROR: Visualization generation failed"
    exit 1
fi
