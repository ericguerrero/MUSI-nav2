# Exercise 3 - Simple 3-Terminal Setup

**Purpose**: Run Exercise 3 with minimal terminals
**Time**: 10 minutes
**Terminals needed**: 3

---

## Overview

Exercise 3 is now simplified to **3 launch files**:

1. **ex3_1_simulation.launch.py** - Gazebo + Robot + RViz (all-in-one)
2. **ex3_2_nav2.launch.py** - Nav2 navigation stack
3. **ex3_3_waypoint_follower.launch.py** - Waypoint navigation script

---

## Prerequisites

Start Docker environment:

```bash
./run.sh
```

---

## Terminal 1: Simulation Environment

**What it does**: Launches Gazebo, spawns robot, starts RViz

```bash
docker exec -it nav2_workspace bash
source /opt/ros/humble/setup.bash
cd /ros2_ws/src
ros2 launch ex3_1_simulation.launch.py
```

**Expected** (takes ~10 seconds):
- Gazebo window opens with landmark world (5 cylindrical obstacles)
- TurtleBot3 spawns at (2.0, -2.0) - southeast corner
- RViz window opens showing map and robot

**Verify**:
- ✅ Gazebo shows robot at southeast corner
- ✅ RViz displays map, robot model, LaserScan
- ✅ Console shows: "Successfully spawned entity"

**Optional arguments**:
```bash
# Spawn robot at different position
ros2 launch ex3_1_simulation.launch.py x:=0.0 y:=0.0

# Headless mode (no Gazebo GUI, RViz only)
ros2 launch ex3_1_simulation.launch.py headless:=true

# Skip RViz (Gazebo only)
ros2 launch ex3_1_simulation.launch.py rviz:=false

# Headless + no RViz (simulation only, no visualization)
ros2 launch ex3_1_simulation.launch.py headless:=true rviz:=false

# Use custom RViz config
ros2 launch ex3_1_simulation.launch.py rviz_config:=/shared/configs/ex3_odometry_drift.rviz
```

---

## Terminal 2: Nav2 Navigation Stack

**What it does**: Starts Nav2 (map server, AMCL, planners, controllers)

**Wait**: Terminal 1 shows robot spawned (~10 seconds)

```bash
docker exec -it nav2_workspace bash
source /opt/ros/humble/setup.bash
cd /ros2_ws/src
ros2 launch ex3_2_nav2.launch.py
```

**Expected** (takes 15-20 seconds):
```
[INFO] [lifecycle_manager]: Creating lifecycle_manager
[INFO] [map_server]: Creating map server
[INFO] [amcl]: Creating AMCL
[INFO] [controller_server]: Creating controller server
[INFO] [planner_server]: Creating planner server
[INFO] [behavior_server]: Creating behavior server
[INFO] [bt_navigator]: Creating BT Navigator
...
[INFO] [lifecycle_manager]: Managed nodes are active
```

**Verify**:
- ✅ "Managed nodes are active" appears
- ✅ RViz shows costmaps (blue/purple around landmarks)
- ✅ No error messages

**Optional arguments**:
```bash
# Use different map
ros2 launch ex3_2_nav2.launch.py map:=/shared/maps/other_map.yaml

# Use custom parameters
ros2 launch ex3_2_nav2.launch.py params_file:=/path/to/params.yaml
```

---

## Terminal 3: Waypoint Navigation

**What it does**: Sets initial pose, navigates 4 waypoints, measures drift

**Wait**: Terminal 2 shows "Managed nodes are active" (~20 seconds)

```bash
docker exec -it nav2_workspace bash
source /opt/ros/humble/setup.bash
cd /ros2_ws/src

# Step 1: Set initial pose (tell AMCL where robot is)
ros2 topic pub --once /initialpose geometry_msgs/msg/PoseWithCovarianceStamped "
header:
  frame_id: 'map'
pose:
  pose:
    position: {x: 2.1, y: -1.9, z: 0.0}
    orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}
  covariance: [0.25, 0.0, 0.0, 0.0, 0.0, 0.0,
               0.0, 0.25, 0.0, 0.0, 0.0, 0.0,
               0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
               0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
               0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
               0.0, 0.0, 0.0, 0.0, 0.0, 0.06853891909122467]"

# Wait 5 seconds for AMCL particles to converge
echo "Waiting for AMCL localization..."
sleep 5

# Step 2: Run waypoint follower
ros2 launch ex3_3_waypoint_follower.launch.py
```

**Expected** (takes 3-4 minutes):
```
============================================================
Nav2 Waypoint Follower with Drift Tracking
============================================================
Waiting for Nav2 to become active...
Nav2 is active!
Initial pose set: (2.1, -1.9)
...
→ Navigating to waypoint 1/4: Point A - northeast corner
→ Navigating to waypoint 2/4: Point B - northwest corner
→ Navigating to waypoint 3/4: Point C - southwest corner
→ Navigating to waypoint 4/4: Point D - return to start
============================================================
✓ ALL WAYPOINTS COMPLETED SUCCESSFULLY!
============================================================
DRIFT ANALYSIS (Odom vs Map Frame):
============================================================
ODOMETRY FRAME (wheel encoders only - DRIFTS):
  Start position:  ( 2.000, -2.000)
  Final position:  ( X.XXX, -X.XXX)
  Drift vector:    (X.XXX, X.XXX)
  Total drift:     X.XXX meters

MAP FRAME (Nav2 localization - CORRECTED):
  Start position:  ( 2.000, -2.000)
  Final position:  ( 1.99X, -1.99X)
  Error vector:    (-0.00X, 0.00X)
  Total error:     0.0XX meters
============================================================
KEY INSIGHTS:
  • Odometry drift: X.XXX m (accumulated error)
  • Nav2 accuracy:  0.0XX m (corrected by localization)
  • Improvement:    XX.X% reduction in error
============================================================
```

**Verify**:
- ✅ Robot navigates to all 4 waypoints
- ✅ Robot avoids landmarks (no collisions)
- ✅ Drift analysis displayed
- ✅ Odometry drift > 0.1m
- ✅ Map frame error < 0.05m

---

## What to Watch in RViz

During navigation, observe:

1. **Global Plan** (red line): Path from current position to goal
2. **Local Plan** (green line): Immediate trajectory
3. **Global Costmap** (blue/purple): Static obstacles with inflation
4. **Local Costmap** (rolling window): Dynamic obstacle avoidance
5. **Robot Path** (trail): Where robot has been
6. **AMCL Particles** (small arrows): Localization uncertainty

---

## Record Your Results

After waypoint navigation completes:

1. **Odometry drift**: ______ meters
2. **Map frame error**: ______ meters
3. **Error reduction**: ______%
4. **Did robot avoid all landmarks?**: Y / N
5. **Any navigation failures?**: Y / N
6. **Time to complete navigation**: ______ minutes

---

## Shutdown

**Option A: Quick Shutdown (from host)**
```bash
./stop.sh
```

**Option B: Manual Shutdown (Ctrl+C in reverse order)**
1. Terminal 3: Waypoint follower (Ctrl+C)
2. Terminal 2: Nav2 stack (Ctrl+C, wait 5 seconds)
3. Terminal 1: Simulation (Ctrl+C)

---

## Troubleshooting

### Terminal 1 Issues

**Problem**: Gazebo window doesn't appear
```bash
# Check X11 forwarding
echo $DISPLAY
xhost +local:docker
```

**Problem**: "Address already in use"
```bash
killall gzserver gzclient
# Wait 5 seconds, then retry
```

**Problem**: Robot doesn't spawn
- Wait 10 seconds (Gazebo takes time to initialize)
- Check console for "Successfully spawned entity"

---

### Terminal 2 Issues

**Problem**: "Map does not exist"
```bash
# Verify map files exist
ls -l /shared/maps/landmark_map.yaml
ls -l /shared/maps/landmark_map.pgm
```

**Problem**: Nav2 takes too long
- Be patient, it takes 15-20 seconds
- Check console for errors
- Verify robot spawned successfully (Terminal 1)

**Problem**: AMCL fails to activate
```bash
# Check if /scan topic is publishing
ros2 topic hz /scan  # Should show ~5 Hz
```

---

### Terminal 3 Issues

**Problem**: "Nav2 is not active" timeout
- Wait for Terminal 2 to show "Managed nodes are active"
- Check Nav2 status: `ros2 lifecycle get /bt_navigator`

**Problem**: Robot doesn't move
```bash
# Check velocity commands
ros2 topic hz /cmd_vel  # Should show ~10 Hz when moving

# Check navigation feedback
ros2 topic echo /navigate_to_pose/_action/feedback
```

**Problem**: AMCL particles don't converge
- Initial pose might be wrong
- Set initial pose again
- Wait 10 seconds for convergence

---

## Monitoring Commands

Open a 4th terminal for monitoring (optional):

```bash
docker exec -it nav2_workspace bash
source /opt/ros/humble/setup.bash

# List all active topics
ros2 topic list

# Monitor key topics
ros2 topic hz /odom        # ~30-50 Hz
ros2 topic hz /scan        # ~5 Hz
ros2 topic hz /cmd_vel     # ~10 Hz when moving

# Check Nav2 lifecycle states
ros2 lifecycle get /bt_navigator
ros2 lifecycle get /controller_server

# View TF tree
ros2 run tf2_tools view_frames  # Creates frames.pdf
```

---

## Timeline

| Step | Duration | Cumulative |
|------|----------|------------|
| Terminal 1: Simulation | 10s | 0:10 |
| Terminal 2: Nav2 stack | 20s | 0:30 |
| Terminal 3: Set initial pose | 5s | 0:35 |
| Terminal 3: AMCL convergence | 5s | 0:40 |
| Terminal 3: Navigation (4 waypoints) | 3-4 min | 4-5 min |
| Review results | 30s | 5 min |

**Total**: ~5 minutes from start to results

---

## Success Criteria

✅ **PASS** if:
- All 3 terminals launch without errors
- Gazebo and RViz display correctly
- Robot navigates to all 4 waypoints
- Robot avoids landmarks (no collisions)
- Drift analysis shows:
  - Odometry drift > 0.1m
  - Map frame error < 0.05m
  - Error reduction > 90%

❌ **FAIL** if:
- Launch files error out
- Robot doesn't spawn
- Nav2 fails to activate
- Robot doesn't move
- Collisions occur
- Navigation times out

---

## Quick Reference Commands

Copy-paste ready:

```bash
# TERMINAL 1
docker exec -it nav2_workspace bash
source /opt/ros/humble/setup.bash
cd /ros2_ws/src
ros2 launch ex3_1_simulation.launch.py

# TERMINAL 2 (wait 10s after Terminal 1)
docker exec -it nav2_workspace bash
source /opt/ros/humble/setup.bash
cd /ros2_ws/src
ros2 launch ex3_2_nav2.launch.py

# TERMINAL 3 (wait 20s after Terminal 2)
docker exec -it nav2_workspace bash
source /opt/ros/humble/setup.bash
cd /ros2_ws/src
ros2 topic pub --once /initialpose geometry_msgs/msg/PoseWithCovarianceStamped "header: {frame_id: 'map'} pose: {pose: {position: {x: 2.1, y: -1.9, z: 0.0}, orientation: {w: 1.0}}, covariance: [0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.06853891909122467]}"
sleep 5
ros2 launch ex3_3_waypoint_follower.launch.py
```

---

## Files Used

| File | Purpose |
|------|---------|
| `/ros2_ws/src/ex3_1_simulation.launch.py` | Gazebo + Robot + RViz |
| `/ros2_ws/src/ex3_2_nav2.launch.py` | Nav2 navigation stack |
| `/ros2_ws/src/ex3_3_waypoint_follower.launch.py` | Waypoint navigation |
| `/ros2_ws/src/nav2_waypoint_follower.py` | Navigation script |
| `/shared/maps/landmark_map.yaml` | Map configuration |
| `/shared/maps/landmark_map.pgm` | Map image |
| `/shared/worlds/landmark_world.world` | Gazebo world |
| `/shared/configs/nav2_params_ex3.yaml` | Nav2 parameters |

---

## What You're Learning

1. **Odometry Drift**: Wheel encoders accumulate error over time
2. **Nav2 Localization**: AMCL corrects drift using map + sensors
3. **Collision Avoidance**: Costmaps prevent crashes
4. **Path Planning**: Global and local planners work together
5. **Frame Transforms**: Understanding odom vs map frames

---

**Created**: 2025-10-28
**For detailed explanations**: See `E3_LAUNCH_FILES_GUIDE.md`
**For manual terminal commands**: See `E3_MANUAL_TERMINAL_SETUP.md`
