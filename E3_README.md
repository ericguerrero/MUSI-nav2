# Exercise 3 - Nav2 Waypoint Navigation with Drift Tracking

Clean, minimal setup for demonstrating odometry drift vs Nav2 localization.

---

## Quick Start (3 Terminals)

```bash
# Terminal 1: Simulation (Gazebo + Robot + RViz)
docker exec -it nav2_workspace bash
source /opt/ros/humble/setup.bash
cd /ros2_ws/src
ros2 launch ex3_1_simulation.launch.py

# Terminal 2: Nav2 Stack (wait 10s)
docker exec -it nav2_workspace bash
source /opt/ros/humble/setup.bash
cd /ros2_ws/src
ros2 launch ex3_2_nav2.launch.py

# Terminal 3: Navigation (wait 20s after Terminal 2)
docker exec -it nav2_workspace bash
source /opt/ros/humble/setup.bash
cd /ros2_ws/src
ros2 topic pub --once /initialpose geometry_msgs/msg/PoseWithCovarianceStamped "header: {frame_id: 'map'} pose: {pose: {position: {x: 2.1, y: -1.9, z: 0.0}, orientation: {w: 1.0}}, covariance: [0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.06853891909122467]}"
sleep 5
ros2 launch ex3_3_waypoint_follower.launch.py
```

**Total time**: ~5 minutes (10s setup + 20s Nav2 init + ~4min navigation)

---

## Files

### Launch Files (3)
- `ex3_1_simulation.launch.py` - Gazebo + Robot + RViz
- `ex3_2_nav2.launch.py` - Nav2 navigation stack
- `ex3_3_waypoint_follower.launch.py` - Waypoint navigation script

### Python Scripts (1)
- `nav2_waypoint_follower.py` - Custom waypoint follower with drift tracking

### Configuration Files
- `/shared/maps/landmark_map.yaml` - Map configuration
- `/shared/maps/landmark_map.pgm` - Map image (10m × 10m)
- `/shared/worlds/landmark_world.world` - Gazebo world (5 landmarks)
- `/shared/configs/nav2_params_ex3.yaml` - Nav2 parameters

### Documentation (1)
- `E3_SIMPLE_GUIDE.md` - Complete setup guide

---

## Launch Options

### ex3_1_simulation.launch.py

```bash
# Default (Gazebo GUI + RViz)
ros2 launch ex3_1_simulation.launch.py

# Headless mode (no Gazebo GUI, RViz only)
ros2 launch ex3_1_simulation.launch.py headless:=true

# No visualization (simulation only)
ros2 launch ex3_1_simulation.launch.py headless:=true rviz:=false

# Custom spawn position
ros2 launch ex3_1_simulation.launch.py x:=0.0 y:=0.0

# Custom RViz config
ros2 launch ex3_1_simulation.launch.py rviz_config:=/path/to/config.rviz
```

### ex3_2_nav2.launch.py

```bash
# Default
ros2 launch ex3_2_nav2.launch.py

# Custom map
ros2 launch ex3_2_nav2.launch.py map:=/shared/maps/other_map.yaml

# Custom parameters
ros2 launch ex3_2_nav2.launch.py params_file:=/path/to/params.yaml
```

### ex3_3_waypoint_follower.launch.py

```bash
# Default (5 second delay)
ros2 launch ex3_3_waypoint_follower.launch.py

# Custom delay
ros2 launch ex3_3_waypoint_follower.launch.py delay:=10.0

# No delay (immediate start)
ros2 launch ex3_3_waypoint_follower.launch.py delay:=0.0
```

---

## Expected Results

After navigation completes, you should see:

```
============================================================
✓ ALL WAYPOINTS COMPLETED SUCCESSFULLY!
============================================================
DRIFT ANALYSIS (Odom vs Map Frame):
============================================================
ODOMETRY FRAME (wheel encoders only - DRIFTS):
  Total drift:     0.2-0.5 meters (typical)

MAP FRAME (Nav2 localization - CORRECTED):
  Total error:     < 0.05 meters

Improvement: > 90% error reduction
============================================================
```

---

## What's Being Demonstrated

1. **Odometry Drift**: Wheel encoders accumulate error over time
   - Robot thinks it returns to start (0, 0)
   - Actually ends up 0.2-0.5m away

2. **Nav2 Localization**: AMCL corrects drift continuously
   - Uses LiDAR + map matching
   - Maintains < 0.05m accuracy
   - 90%+ improvement over odometry alone

3. **Collision Avoidance**: Nav2 navigates safely around landmarks

4. **Path Planning**: Global and local planners work together

---

## Troubleshooting

### Gazebo doesn't open
```bash
echo $DISPLAY
xhost +local:docker
```

### "Address already in use"
```bash
killall gzserver gzclient
```

### Nav2 takes too long
- Be patient (15-20 seconds is normal)
- Check for errors in Terminal 2

### Robot doesn't move
```bash
ros2 topic hz /cmd_vel  # Should show ~10 Hz
```

### More help
See `E3_SIMPLE_GUIDE.md` for detailed troubleshooting.

---

## Shutdown

```bash
# Quick (from host)
./stop.sh

# Manual (Ctrl+C in each terminal, reverse order: 3, 2, 1)
```

---

## Architecture

```
Terminal 1: ex3_1_simulation.launch.py
  ├─ gzserver (physics simulation)
  ├─ gzclient (GUI, optional with headless:=true)
  ├─ robot_state_publisher (TF transforms)
  ├─ spawn_entity.py (TurtleBot3 model)
  └─ rviz2 (visualization, optional with rviz:=false)

Terminal 2: ex3_2_nav2.launch.py
  ├─ map_server (landmark_map.yaml)
  ├─ amcl (localization)
  ├─ controller_server (DWB local planner)
  ├─ planner_server (NavFn global planner)
  ├─ behavior_server (recovery behaviors)
  ├─ bt_navigator (behavior tree coordination)
  └─ waypoint_follower (sequential navigation)

Terminal 3: ex3_3_waypoint_follower.launch.py
  └─ nav2_waypoint_follower.py (custom script)
      ├─ Subscribes: /odom
      ├─ Publishes: navigation goals via BasicNavigator
      ├─ Monitors: odom frame (drifts)
      └─ Monitors: map frame (corrected)
```

---

## Educational Value

**Before this exercise**: Students don't understand why SLAM matters

**After this exercise**: Students see concrete proof that:
- Pure odometry fails (0.3m error on 4m path = 7.5% error)
- Nav2 localization works (0.01m error = 0.25% error)
- SLAM/localization is essential for real robots

**Next steps**:
- Exercise 4: Create maps with SLAM Toolbox
- Exercise 5: Understand AMCL localization
- Exercise 6: Sensor fusion with EKF

---

**Status**: ✅ Ready for testing
**Last updated**: 2025-10-28
**Estimated time**: 5 minutes
