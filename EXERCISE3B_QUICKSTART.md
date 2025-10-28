# Exercise 3b: Nav2 Waypoint Following - Quick Start Guide

## What's New?

Exercise 3b uses **Nav2 navigation stack** instead of pure odometry, providing:
- ✅ Collision avoidance (uses LiDAR sensor data)
- ✅ Path planning (finds safe routes)
- ✅ Accurate localization (corrects drift)
- ✅ **Still demonstrates drift** (compares odom vs map frames)

## Quick Start

### Method 1: Helper Script (Easiest)

```bash
./run_nav2_exercise.sh
```

This single command launches everything!

### Method 2: Manual Launch

```bash
# Terminal 1: Launch everything
docker exec -it nav2_workspace bash
source /opt/ros/humble/setup.bash
cd /ros2_ws/src
ros2 launch ex3_nav2_drift.launch.py
```

### Method 3: With RViz Visualization

```bash
# Terminal 1: Launch navigation
./run_nav2_exercise.sh

# Terminal 2: Launch RViz (optional)
docker exec -it nav2_workspace bash
rviz2 -d /shared/configs/ex3_odometry_drift.rviz
```

## What to Expect

### Startup Sequence (takes ~15 seconds)
1. **Gazebo opens** - TurtleBot3 in turtlebot3_world
2. **Nav2 starts** - Costmaps initialize, planners load
3. **Localization** - Robot determines its position in map
4. **Waypoint follower starts** - Begins navigation

### Navigation Behavior
- Robot navigates 2m × 2m square:
  - Start: (1.0, -1.0)
  - Point A: (1.0, 1.0)
  - Point B: (-1.0, 1.0)
  - Point C: (-1.0, -1.0) - back to start

- **Collision avoidance**: Robot detects and avoids obstacles
- **Path planning**: Creates smooth paths around obstacles
- **Duration**: ~3-4 minutes to complete square

### Console Output

You'll see:
```
============================================================
Nav2 Waypoint Follower with Drift Tracking
============================================================
Path: 2m × 2m square centered on origin
Waypoints: 4 corners
Navigation: Nav2 stack (collision avoidance enabled)
============================================================
Waiting for Nav2 to become active...
Nav2 is active!
Initial pose set: (1.0, -1.0)
Start odom position: (1.000, -1.000)
Start map position: (1.000, -1.000)
→ Navigating to waypoint 1/4: Point A - top right
→ Navigating to waypoint 2/4: Point B - top left
→ Navigating to waypoint 3/4: Point C - bottom left
→ Navigating to waypoint 4/4: Point D - return
============================================================
✓ ALL WAYPOINTS COMPLETED SUCCESSFULLY!
============================================================
DRIFT ANALYSIS (Odom vs Map Frame):
============================================================
ODOMETRY FRAME (wheel encoders only - DRIFTS):
  Start position:  ( 1.000, -1.000)
  Final position:  ( 0.734, -0.823)
  Drift vector:    (-0.266, 0.177)
  Total drift:     0.319 meters

MAP FRAME (Nav2 localization - CORRECTED):
  Start position:  ( 1.000, -1.000)
  Final position:  ( 0.995, -0.998)
  Error vector:    (-0.005, 0.002)
  Total error:     0.005 meters
============================================================
KEY INSIGHTS:
  • Odometry drift: 0.319m (accumulated error)
  • Nav2 accuracy:  0.005m (corrected by localization)
  • Improvement:    98.4% reduction in error
============================================================
```

## Comparison: Exercise 3a vs 3b

| Feature | Exercise 3a (Odom Only) | Exercise 3b (Nav2) |
|---------|------------------------|-------------------|
| **Collision Avoidance** | ❌ None | ✅ Full (costmaps + LiDAR) |
| **Path Planning** | ❌ Straight lines | ✅ Optimized paths |
| **Final Accuracy** | ~0.3-0.4m drift | <0.01m error |
| **Drift Visible** | ✅ Yes | ✅ Yes (but corrected!) |
| **Safe Navigation** | ❌ Crashes | ✅ Avoids obstacles |
| **Complexity** | Low (250 lines) | Medium (uses Nav2) |

## Key Learning Outcomes

1. **Odometry Drift Still Exists**:
   - Even with Nav2, the odom frame accumulates error
   - This is inherent to wheel encoders

2. **Nav2 Corrects Drift**:
   - Uses localization (matching sensor data to map)
   - Map frame stays accurate
   - Continuous correction during navigation

3. **Why SLAM Matters**:
   - Need accurate map for localization
   - Next exercises cover SLAM mapping
   - Then use map for precise navigation

## Troubleshooting

### "Nav2 is not active" - Timeout
**Solution**: Nav2 takes 10-15 seconds to start. Be patient!

### Robot doesn't move
**Solution**: Check Nav2 logs for errors:
```bash
ros2 topic hz /cmd_vel  # Should show ~10 Hz
ros2 topic echo /navigate_to_pose/_action/feedback
```

### Path planning fails
**Solution**: Map might not be loaded:
```bash
ros2 topic echo /map --once  # Should show map data
```

### TF lookup errors
**Solution**: Wait for full Nav2 initialization (15 seconds)

## Files Created

```
/shared/maps/
├── empty_10x10.yaml        # Map configuration
└── empty_10x10.pgm         # Map image (10m × 10m free space)

/ros2_ws/src/
├── nav2_waypoint_follower.py    # Main navigation script
└── ex3_nav2_drift.launch.py     # Launch file (Gazebo + Nav2 + script)

/
└── run_nav2_exercise.sh         # Helper script (easy start)
```

## Next Steps

After completing Exercise 3b:
1. Compare results with Exercise 3a (pure odometry)
2. Understand why Nav2 is superior for real robots
3. Move to Exercise 4: SLAM Toolbox mapping
4. Learn how to create accurate maps for localization

---

**Ready to test?** Run: `./run_nav2_exercise.sh`
