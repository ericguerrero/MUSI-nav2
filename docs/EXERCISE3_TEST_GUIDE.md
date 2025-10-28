# Exercise 3 Testing Guide

**Test Date**: 2025-10-28
**Test Environment**: Docker (nav2_workspace), TurtleBot3 Waffle, Gazebo
**Test Objective**: Validate odometry-only waypoint navigation demonstrates drift

---

## Quick Start

```bash
# 1. Run the test setup script
./test_exercise3.sh

# 2. Follow the 3-terminal setup displayed by the script
```

---

## What We're Testing

Exercise 3 demonstrates that **odometry-only navigation accumulates errors**. The robot attempts to navigate a 1m × 1m square and return to origin, but actually ends up ~0.3-0.4m away due to drift.

**Why this matters**: This motivates students to learn SLAM and localization in subsequent exercises.

---

## Test Files

- `/ros2_ws/src/simple_odom_waypoint.py` - Waypoint follower script (340 lines)
- `/shared/configs/ex3_odometry_drift.rviz` - RViz2 visualization config
- `/docs/exercises/03_odometry_waypoint_drift.md` - Student exercise guide
- `/shared/student_work/ex3_odometry_drift/` - Submission directory

---

## Manual Testing Procedure

### Terminal 1: Launch Gazebo

```bash
docker exec -it nav2_workspace bash
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
```

**Wait for**: `[INFO] [gazebo_ros_node]: ROS was initialized`

### Terminal 2: Launch RViz2 (Optional)

```bash
docker exec -it nav2_workspace bash
rviz2 -d /shared/configs/ex3_odometry_drift.rviz
```

**Verify you see**:
- Grid (1m cells)
- TurtleBot3 robot at origin
- LaserScan data

### Terminal 3: Run Waypoint Follower

```bash
docker exec -it nav2_workspace bash
cd /ros2_ws/src
python3 simple_odom_waypoint.py
```

---

## Expected Behavior

### 1. Robot Navigation (2-4 minutes)

The robot should navigate:
- **Point A**: Forward 1m → Position (1.0, 0.0)
- **Point B**: Right 1m → Position (1.0, 1.0)
- **Point C**: Backward 1m → Position (0.0, 1.0)
- **Point D**: Left to start → Position (~0.3, ~-0.2) ← **DRIFT!**

### 2. Console Output

**Start:**
```
============================================================
Simple Odometry Waypoint Follower Started
============================================================
This script demonstrates ODOMETRY DRIFT
Expected: Return to (0.0, 0.0)
Reality: Will drift to (~0.3, ~-0.2)
============================================================
Total waypoints: 4
Start position recorded: (0.000, 0.000)
```

**Progress:**
```
✓ Waypoint 1/4 reached: Point A - 1m forward
  Position: (1.000, 0.000), Yaw: 0.0°
...
```

**Completion:**
```
============================================================
ALL WAYPOINTS COMPLETED!
============================================================
DRIFT ANALYSIS:
  Start position:    (0.000, 0.000)
  Expected final:    (0.000, 0.000)
  Actual final:      (0.312, -0.187)    ← THE KEY FINDING
  Drift (X, Y):      (0.312, -0.187)
  Total drift:       0.364 meters       ← ~36cm error
============================================================
WHY DID THIS HAPPEN?
  - Wheel slippage on smooth floor
  - Odometry measurement noise
  - Accumulated rounding errors
============================================================
SOLUTION: We need SLAM and Localization!
  → Exercise 4: SLAM Toolbox mapping
  → Exercise 6: Sensor fusion (EKF)
  → Exercise 7: SLAM Toolbox localization
============================================================
```

### 3. RViz2 Visualization

**You should see**:
- Red arrow trail showing approximate square path
- Robot does NOT return exactly to origin
- Covariance ellipses (position uncertainty)

---

## Success Criteria

### ✅ Functional
- [ ] Script executes without errors
- [ ] Robot completes all 4 waypoints
- [ ] Drift analysis is displayed
- [ ] Drift magnitude: 0.2-0.5m (typical)
- [ ] Script exits cleanly

### ✅ Educational
- [ ] Logging is clear and informative
- [ ] Drift causes are explained
- [ ] Solutions are previewed (SLAM, EKF, localization)
- [ ] Visual confirmation in RViz2

---

## Verification Commands

```bash
# Check ROS2 topics (inside container)
ros2 topic list | grep -E '(odom|cmd_vel|scan)'

# Monitor odometry position
ros2 topic echo /odom --field pose.pose.position

# Monitor velocity commands
ros2 topic echo /cmd_vel
```

**Expected topics**:
- `/odom` (30-50 Hz)
- `/cmd_vel` (10-30 Hz)
- `/scan` (5 Hz)

---

## Troubleshooting

### Robot doesn't move
**Symptom**: Script runs but robot stays still

**Check**:
```bash
ros2 topic hz /odom    # Should show ~30 Hz
ros2 topic info /cmd_vel  # Should have 1 subscriber (Gazebo)
```

**Fix**: Wait 5-10 seconds for Gazebo to fully initialize

---

### Script exits immediately
**Symptom**: Python script exits without navigating

**Check**:
```bash
ros2 topic hz /odom    # Should be publishing
```

**Fix**: Ensure Gazebo is fully loaded before running script

---

### No GUI windows (Gazebo/RViz2)
**Symptom**: Launched but no windows appear

**Check**:
```bash
echo $DISPLAY                        # Should show :0 or similar
docker exec nav2_workspace xeyes     # Test X11
```

**Fix**:
```bash
xhost +local:docker
export DISPLAY=:0
```

---

### Robot spins in place
**Symptom**: Robot rotates continuously

**Fix**: Already handled in code (quaternion-to-yaw conversion)

---

## Testing Multiple Trials

### Trial 1: Baseline (1m × 1m)

Run the script and record:
```
Final Position: (_____,  _____)
Total Drift: _____ meters
```

### Trial 2: Repeatability

Restart Gazebo and run again:
```
Final Position: (_____,  _____)
Total Drift: _____ meters
Variance from Trial 1: _____ meters
```

### Trial 3: Larger Path (2m × 2m)

Edit `simple_odom_waypoint.py` line 39-44:
```python
self.waypoints = [
    {'x': 2.0, 'y': 0.0, 'name': 'Point A - 2m forward'},
    {'x': 2.0, 'y': 2.0, 'name': 'Point B - 2m right'},
    {'x': 0.0, 'y': 2.0, 'name': 'Point C - 2m backward'},
    {'x': 0.0, 'y': 0.0, 'name': 'Point D - return to start'},
]
```

Record:
```
Final Position: (_____,  _____)
Total Drift: _____ meters
Drift ratio (2m vs 1m): _____
```

**Question**: Does error scale linearly with path length?

---

## Expected Results Summary

| Path Size | Expected Drift | Typical Range |
|-----------|---------------|---------------|
| 1m × 1m   | ~0.35m        | 0.2-0.5m      |
| 2m × 2m   | ~0.7m         | 0.5-1.0m      |

---

## Next Steps

### If Test Passes ✅
1. Document actual drift values in exercise guide
2. Create student submission examples
3. Proceed to Exercise 4 (SLAM Toolbox mapping)

### If Test Fails ❌
1. Check container logs: `docker logs nav2_workspace`
2. Verify ROS2 environment: `ros2 doctor --report`
3. Test basic ROS2 communication (see Verification Commands)

---

## Performance Notes

- **Navigation time**: 2-4 minutes per trial
- **CPU usage**: 80-150% (Gazebo + physics)
- **RAM usage**: 2-3 GB (Gazebo + RViz2)
- **Topics**: 17+ active during navigation

---

## Files Created for This Test

```
MUSI-nav2/
├── test_exercise3.sh                           # Automated test setup script
├── docs/
│   ├── exercises/
│   │   └── 03_odometry_waypoint_drift.md       # Complete exercise guide
│   └── EXERCISE3_TEST_GUIDE.md                 # This file
├── ros2_ws/src/
│   └── simple_odom_waypoint.py                 # Waypoint follower (340 lines)
├── shared/
│   ├── configs/
│   │   └── ex3_odometry_drift.rviz             # RViz2 configuration
│   └── student_work/
│       └── ex3_odometry_drift/
│           └── README.md                       # Submission instructions
```

---

**Testing Status**: Ready for validation
**Last Updated**: 2025-10-28
