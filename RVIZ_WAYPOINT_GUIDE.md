# RViz Visualization Guide for Exercise 3

## Updated Configuration

The RViz configuration has been optimized for the 0.5m × 0.5m square waypoint path with **fixed map coordinates**.

### Waypoint Path (Fixed Coordinates in Odom Frame)

```
Start:   (0.0, 0.0)  ← Origin marker
Point A: (0.5, 0.0)  → 0.5m forward (east)
Point B: (0.5, 0.5)  → 0.5m right (north)
Point C: (0.0, 0.5)  → 0.5m backward (west)
Point D: (0.0, 0.0)  → Return to origin
```

## Launch RViz

```bash
docker exec -it nav2_workspace bash
rviz2 -d /shared/configs/ex3_odometry_drift.rviz
```

## What You'll See

### Grid Configuration
- **Cell Size**: 0.25m (quarter-meter grid)
- **Total Grid**: 5m × 5m (20 cells)
- **Centered**: Origin (0, 0) at grid center
- **Visual**: Each waypoint is 2 grid cells apart

### Camera View
- **Focal Point**: (0.25, 0.25) - center of the square
- **Distance**: 2.5m - close-up view of the path
- **Pitch**: 1.2 rad (~69°) - bird's eye view
- **Yaw**: 0.785 rad (45°) - diagonal perspective

### Displays

1. **Grid**:
   - Shows 0.25m cells
   - Origin clearly visible at center

2. **Robot Model (TurtleBot3)**:
   - 3D visualization
   - Real-time position

3. **Odometry Trail** (Red Arrows):
   - Keeps last 1000 poses
   - Shows complete navigation path
   - Arrow direction = robot heading
   - **No covariance ellipses** (cleaner view)

4. **LaserScan** (White Points):
   - LiDAR sensor data
   - 360° obstacle detection

5. **TF Frames**:
   - `odom` → `base_footprint` → `base_link`
   - Shows coordinate frame relationships

6. **Start Position Axes**:
   - RGB axes at origin
   - Reference point for drift measurement

## Testing Procedure

### Step 1: Position Robot Near Origin

First, move the robot close to (0, 0):

```bash
# Reset simulation (robot goes to default position)
docker exec nav2_workspace bash -c "source /opt/ros/humble/setup.bash && ros2 service call /reset_simulation std_srvs/srv/Empty"

# Check current position
docker exec nav2_workspace bash -c "source /opt/ros/humble/setup.bash && ros2 topic echo /odom --once | grep -A 3 'position:'"
```

**Option A**: If robot is far from origin, use teleop to drive it:
```bash
docker exec -it nav2_workspace bash
ros2 run turtlebot3_teleop teleop_keyboard
# Drive to approximately (0, 0)
```

**Option B**: The waypoint script will navigate from wherever the robot starts, but it's best to start near the origin for visualization.

### Step 2: Launch RViz (Terminal 1)

```bash
docker exec -it nav2_workspace bash
rviz2 -d /shared/configs/ex3_odometry_drift.rviz
```

### Step 3: Run Waypoint Follower (Terminal 2)

```bash
docker exec -it nav2_workspace bash
cd /ros2_ws/src
python3 simple_odom_waypoint.py
```

## What to Observe in RViz

### During Navigation

1. **Robot Movement**:
   - Should navigate toward (0.5, 0.0) first
   - Then to (0.5, 0.5)
   - Then to (0.0, 0.5)
   - Finally attempt to return to (0.0, 0.0)

2. **Odometry Trail**:
   - Red arrows forming approximate square
   - Trail shows cumulative path
   - Watch for drift accumulation

3. **Grid Reference**:
   - Each waypoint is 2 grid cells (0.5m) apart
   - Easy to visually verify distances

### After Completion

1. **Final Position**:
   - Robot should be NEAR but NOT exactly at origin
   - Visual drift: ~5-15 cm typical (0.5m square)
   - Console shows exact drift measurement

2. **Path Shape**:
   - Should resemble a square
   - Corners may not be perfectly sharp
   - Final leg may not close perfectly (drift!)

## RViz Tips

### Adjust Camera View

- **Rotate**: Middle-click + drag
- **Pan**: Shift + middle-click + drag
- **Zoom**: Scroll wheel

### Reset View

If you lose the robot:
1. Click "Reset" in Views panel
2. Or manually set:
   - Focal Point: (0.25, 0.25, 0)
   - Distance: 2.5

### Toggle Displays

Uncheck displays you don't need:
- LaserScan (if too cluttered)
- TF (if frame names overlap)

### Measure Distance

Use the "Measure" tool (toolbar) to verify waypoint distances:
1. Click Measure tool
2. Click start point
3. Click end point
4. Distance displayed

## Troubleshooting

### Robot Not Visible
- Check "RobotModel" is enabled
- Verify /robot_description topic is publishing
- Try: `ros2 topic echo /robot_description --once`

### No Odometry Trail
- Check "Odometry" display is enabled
- Verify topic is `/odom`
- Check: `ros2 topic hz /odom` (should be ~30 Hz)

### Grid Not Centered on Origin
- Fixed Frame should be `odom`
- Grid Offset should be (0, 0, 0)

### Poor Performance
- Disable LaserScan display
- Reduce Odometry "Keep" from 1000 to 100

## Expected Drift Visualization

For 0.5m × 0.5m square:
- **Typical drift**: 5-15 cm (0.05-0.15m)
- **Visual**: Final position offset from origin
- **Trail**: Square path doesn't perfectly close

### Drift Patterns

Common observations:
- **Rotational drift**: Square rotates slightly
- **Translational drift**: Entire path shifts
- **Scaling drift**: Square slightly larger/smaller
- **Combination**: All of the above!

---

**Pro Tip**: Take a screenshot when the robot completes the path to document the drift pattern for your report!

```bash
# Screenshot tool in RViz (if available)
# Or use system screenshot: PrintScreen key
```

---

**Configuration Version**: 1.1
**Last Updated**: 2025-10-28
**Waypoint Path**: 0.5m × 0.5m fixed map coordinates
**Covariance Display**: Disabled for cleaner visualization
