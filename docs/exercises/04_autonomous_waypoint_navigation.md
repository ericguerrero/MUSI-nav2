# Exercise 4: Autonomous Waypoint Navigation

**Duration**: 30-45 minutes
**Prerequisites**: Completed Exercise 3 (Navigation Basics)
**Learning Objectives**:
- Understand sequential waypoint navigation
- Program autonomous patrol routes
- Modify waypoint sequences for custom behaviors

---

## Overview

This exercise introduces autonomous waypoint following. Instead of manually sending individual navigation goals, the robot will follow a sequence of waypoints automatically - useful for patrol routes, delivery tasks, and inspection tours.

---

## Setup (Quick Review from Exercise 3)

### Terminal 1: Start Simulation
```bash
docker exec -it nav2_workspace bash
ros2 launch /ros2_ws/src/simulation.launch.py headless:=true
```

### Terminal 2: Start Nav2 Stack
```bash
docker exec -it nav2_workspace bash
ros2 launch /ros2_ws/src/nav2.launch.py
```

Wait for: `[lifecycle_manager_navigation]: Activating` message

### Terminal 3: Initialize Robot Pose
```bash
docker exec -it nav2_workspace bash
source /opt/ros/humble/setup.bash
ros2 topic pub /initialpose geometry_msgs/msg/PoseWithCovarianceStamped '{
  header: {frame_id: "map"},
  pose: {
    pose: {
      position: {x: -2.0, y: 2.0, z: 0.0},
      orientation: {w: 1.0}
    }
  }
}' --rate 1 --times 3
```

Verify in Terminal 2: `[amcl]: initialPoseReceived`

### Terminal 4: Launch Waypoint Navigation
```bash
docker exec -it nav2_workspace bash
ros2 launch /ros2_ws/src/waypoint_navigation.launch.py
```

**What happens:**
- Robot autonomously navigates through 5 predefined waypoints
- Progress shown in terminal: "Navigating to waypoint X/5"
- Returns to starting position
- Takes approximately 2-3 minutes to complete

**Success message:**
```
[INFO] [waypoint_follower]: SUCCESS! Completed all 5 waypoints!
```
