#!/bin/bash
# Record ROS2 bag for repeatable SLAM experiments
# Usage: ./record_mapping_session.sh <bag_name>

set -e

if [ $# -eq 0 ]; then
    echo "Usage: $0 <bag_name>"
    echo "Example: $0 office_exploration"
    exit 1
fi

BAG_NAME="$1"
BAG_DIR="/shared/bags"
TIMESTAMP=$(date +%Y%m%d_%H%M%S)
FULL_BAG_NAME="${BAG_NAME}_${TIMESTAMP}"

echo "========================================="
echo "ROS2 Bag Recording for SLAM Experiments"
echo "========================================="
echo "Bag name: ${FULL_BAG_NAME}"
echo "Output directory: ${BAG_DIR}"
echo ""
echo "Recording topics:"
echo "  - /scan (LiDAR)"
echo "  - /odom (Odometry)"
echo "  - /tf (Transforms)"
echo "  - /tf_static (Static transforms)"
echo ""
echo "Press CTRL+C to stop recording"
echo "========================================="
echo ""

cd ${BAG_DIR}

ros2 bag record \
    -o ${FULL_BAG_NAME} \
    /scan \
    /odom \
    /tf \
    /tf_static

echo ""
echo "Recording saved to: ${BAG_DIR}/${FULL_BAG_NAME}"
echo ""
echo "To replay this bag for SLAM testing:"
echo "  ros2 bag play ${BAG_DIR}/${FULL_BAG_NAME}"
