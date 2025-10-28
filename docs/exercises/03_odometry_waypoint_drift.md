# Exercise 3: Odometry-Only Waypoint Navigation (The Drift Problem)

**Objective**: Demonstrate why SLAM and localization are necessary by navigating waypoints using ONLY wheel odometry.

**Duration**: 2-3 hours
**Difficulty**: Beginner
**Prerequisites**: Completed Exercise 1-2 (Environment setup, can teleoperate robot)

---

## Learning Objectives

By the end of this exercise, you will:
1. Understand what odometry is and how it works
2. Implement simple waypoint navigation using odometry only
3. Observe and measure odometry drift over time
4. **Motivate** the need for SLAM (mapping) and localization
5. Understand the fundamental problem that SLAM solves

---

## Introduction

**The Fundamental Question**: Can a robot navigate accurately using only its wheel encoders?

**Spoiler Alert**: NO! And you're about to prove it experimentally.

### What is Odometry?

Odometry is the process of estimating a robot's position by measuring wheel rotations:
- **Left wheel**: Rotated N times → traveled X meters
- **Right wheel**: Rotated M times → traveled Y meters
- **Difference**: Calculate turning angle
- **Result**: Estimated position (x, y, theta) in `odom` frame

### The Problem: Drift

Odometry **accumulates errors** over time:
- Wheel slip on smooth floors
- Uneven wheel wear
- Measurement noise
- Computational rounding errors

**Result**: After driving a square path, robot thinks it's at (0, 0) but actually at (0.3, -0.2)!

### Why This Exercise Matters

Before learning SLAM (complex), you need to **feel the pain** of odometry drift. This exercise gives you that experience.

---

## Part 1: Understanding the System (30 minutes)

### Step 1: Explore Odometry Topic

Launch the simulation and inspect the odometry data:

```bash
# Terminal 1: Start Gazebo
docker exec -it nav2_workspace bash
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
```

```bash
# Terminal 2: Inspect odometry topic
docker exec -it nav2_workspace bash
ros2 topic info /odom
ros2 topic echo /odom
```

**Observe**:
- Topic type: `nav_msgs/msg/Odometry`
- Contains: position (x, y, z), orientation (quaternion), velocities
- Frame: `odom` (origin at robot's start position)

### Step 2: Visualize Odometry in RViz2

```bash
# Terminal 3: Launch RViz2
docker exec -it nav2_workspace bash
rviz2
```

**Configure RViz2**:
1. Set Fixed Frame: `odom`
2. Add display: `Odometry` → Topic: `/odom`
3. Add display: `RobotModel`
4. Add display: `TF` (to see coordinate frames)

**Teleoperate** the robot (Terminal 4):
```bash
docker exec -it nav2_workspace bash
ros2 run turtlebot3_teleop teleop_keyboard
```

**Observe**: As robot moves, odometry track appears in RViz2.

---

## Part 2: Simple Waypoint Navigation (1 hour)

### Step 3: Copy Waypoint Follower Template

```bash
# Inside container
cd /ros2_ws/src
cp /ros2_ws/src/simple_odom_waypoint.py ./
chmod +x simple_odom_waypoint.py
```

**Review the code** (don't run yet):
```python
#!/usr/bin/env python3
"""
Simple odometry-based waypoint follower
Demonstrates odometry drift problem

Student: Read this code carefully before running!
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import math

class SimpleOdomWaypoint(Node):
    def __init__(self):
        super().__init__('simple_odom_waypoint')

        # Publishers and subscribers
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10)

        # Waypoint sequence (square path)
        self.waypoints = [
            {'x': 1.0, 'y': 0.0, 'name': 'Point A (1m forward)'},
            {'x': 1.0, 'y': 1.0, 'name': 'Point B (1m right)'},
            {'x': 0.0, 'y': 1.0, 'name': 'Point C (1m back)'},
            {'x': 0.0, 'y': 0.0, 'name': 'Point D (return to start)'},
        ]

        self.current_waypoint_idx = 0
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0

        self.goal_tolerance = 0.1  # 10cm tolerance

        self.get_logger().info('Simple Odometry Waypoint Follower Started')
        self.get_logger().info('Waypoint 0: Start position')

    def odom_callback(self, msg):
        """Update current position from odometry"""
        # Extract position
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y

        # Extract yaw from quaternion
        orientation_q = msg.pose.pose.orientation
        siny_cosp = 2 * (orientation_q.w * orientation_q.z + orientation_q.x * orientation_q.y)
        cosy_cosp = 1 - 2 * (orientation_q.y * orientation_q.y + orientation_q.z * orientation_q.z)
        self.current_yaw = math.atan2(siny_cosp, cosy_cosp)

        # Navigate to current waypoint
        self.navigate_to_waypoint()

    def navigate_to_waypoint(self):
        """Simple proportional controller to reach waypoint"""
        if self.current_waypoint_idx >= len(self.waypoints):
            # All waypoints reached
            self.stop_robot()
            self.get_logger().info(
                f'All waypoints complete! Final position: ({self.current_x:.3f}, {self.current_y:.3f})')
            self.get_logger().info('Expected: (0.000, 0.000) - Compare to see drift!')
            rclpy.shutdown()
            return

        # Get current goal
        goal = self.waypoints[self.current_waypoint_idx]
        goal_x = goal['x']
        goal_y = goal['y']

        # Calculate distance and angle to goal
        dx = goal_x - self.current_x
        dy = goal_y - self.current_y
        distance = math.sqrt(dx**2 + dy**2)
        angle_to_goal = math.atan2(dy, dx)
        angle_diff = self.normalize_angle(angle_to_goal - self.current_yaw)

        # Check if waypoint reached
        if distance < self.goal_tolerance:
            self.get_logger().info(
                f'Waypoint {self.current_waypoint_idx + 1} reached: {goal["name"]}')
            self.current_waypoint_idx += 1
            return

        # Proportional control
        cmd = Twist()

        # First rotate to face goal
        if abs(angle_diff) > 0.1:  # 5.7 degrees
            cmd.angular.z = 0.5 * angle_diff  # Proportional turning
        else:
            # Then move forward
            cmd.linear.x = min(0.2, 0.5 * distance)  # Proportional forward speed

        self.cmd_pub.publish(cmd)

    def stop_robot(self):
        """Send zero velocity command"""
        cmd = Twist()
        self.cmd_pub.publish(cmd)

    def normalize_angle(self, angle):
        """Normalize angle to [-pi, pi]"""
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle

def main(args=None):
    rclpy.init(args=args)
    node = SimpleOdomWaypoint()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Interrupted by user')
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
```

**Key Questions** (answer before running):
1. What coordinate frame is the robot using? (`odom`)
2. What sensors provide position data? (Wheel encoders only!)
3. What happens if wheels slip? (Position estimate becomes wrong!)

### Step 4: Run First Trial

```bash
# Terminal 1: Gazebo (already running)
# Terminal 2: Run waypoint follower
docker exec -it nav2_workspace bash
cd /ros2_ws/src
python3 simple_odom_waypoint.py
```

**Observe**:
- Robot navigates: Forward → Right → Back → Left
- Should return to (0, 0)
- **But does it?** Watch the final position message!

**Record Results**:
```
Trial 1 Final Position: (_____, _____)
Expected: (0.000, 0.000)
Error: _____ meters
```

### Step 5: Repeat Trials

Run the waypoint follower **3 more times** (restart Gazebo between trials):

```bash
# Stop Gazebo (Ctrl+C in Terminal 1)
# Restart Gazebo
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py

# Run waypoint follower again
python3 simple_odom_waypoint.py
```

**Record All Results**:
```
Trial 1: (_____, _____)
Trial 2: (_____, _____)
Trial 3: (_____, _____)
Trial 4: (_____, _____)
```

**Calculate**:
- Average error: _____ meters
- Maximum error: _____ meters
- Minimum error: _____ meters

---

## Part 3: Experiment with Larger Paths (30 minutes)

### Step 6: Modify Waypoints (2m Square)

Edit `simple_odom_waypoint.py`, change waypoints to:
```python
self.waypoints = [
    {'x': 2.0, 'y': 0.0, 'name': 'Point A (2m forward)'},
    {'x': 2.0, 'y': 2.0, 'name': 'Point B (2m right)'},
    {'x': 0.0, 'y': 2.0, 'name': 'Point C (2m back)'},
    {'x': 0.0, 'y': 0.0, 'name': 'Point D (return to start)'},
]
```

Run 3 trials and record results.

**Hypothesis**: Will error be larger or smaller? Why?

### Step 7: Modify Waypoints (Complex Path)

Try a more complex path (figure-8, spiral, etc.) and observe drift.

---

## Part 4: Analysis & Report (30 minutes)

### Deliverable: "Why We Need SLAM" Report

Create a markdown file: `/shared/student_work/ex3_odometry_drift/drift_analysis.md`

**Template**:
```markdown
# Exercise 3: Odometry Drift Analysis

**Student Name**: [Your Name]
**Date**: [Date]

---

## Executive Summary

This experiment demonstrated that odometry-only navigation **is insufficient** for accurate long-term positioning. Over a simple 1m x 1m square path, the average position error was _____ meters, representing a ___% error relative to the path length.

---

## Experimental Setup

- **Robot**: TurtleBot3 Waffle in Gazebo simulation
- **Environment**: turtlebot3_world (flat, no obstacles)
- **Path**: 1m x 1m square (4 waypoints)
- **Expected Final Position**: (0.000, 0.000)
- **Trials**: 4 per configuration

---

## Results

### Configuration 1: 1m x 1m Square

| Trial | Final X (m) | Final Y (m) | Error (m) | Error (%) |
|-------|-------------|-------------|-----------|-----------|
| 1     |             |             |           |           |
| 2     |             |             |           |           |
| 3     |             |             |           |           |
| 4     |             |             |           |           |
| **Average** | | | | |

### Configuration 2: 2m x 2m Square

| Trial | Final X (m) | Final Y (m) | Error (m) | Error (%) |
|-------|-------------|-------------|-----------|-----------|
| 1     |             |             |           |           |
| 2     |             |             |           |           |
| 3     |             |             |           |           |
| **Average** | | | | |

### Observation: Error Scaling

**Question**: Does error scale linearly with path length?

**Answer**: [Your analysis]

---

## Analysis

### What Causes Odometry Drift?

1. **Wheel Slippage**: [Your explanation]
2. **Uneven Surfaces**: [Your explanation]
3. **Wheel Radius Uncertainty**: [Your explanation]
4. **Measurement Noise**: [Your explanation]

### Why Can't We Just Calibrate Better?

[Your thoughts - can perfect calibration eliminate drift?]

---

## Implications for Robotics

### Problem Statement

If a warehouse robot needs to navigate 100 meters:
- Odometry error (1m path): ___m
- Odometry error (10m path): ___m (estimated)
- Odometry error (100m path): ___m (estimated)

**Conclusion**: Odometry alone is **not viable** for practical navigation.

---

## Solutions: Preview of Upcoming Exercises

Based on this experiment, we need:

1. **SLAM (Simultaneous Localization and Mapping)** - Exercise 4-5
   - Build a map of the environment
   - Use map features to correct position estimate
   - Reduce drift through loop closure

2. **Sensor Fusion (EKF)** - Exercise 6
   - Combine multiple sensors (wheels + IMU + camera)
   - Each sensor corrects others' weaknesses
   - More robust position estimate

3. **Localization (SLAM Toolbox/AMCL)** - Exercise 7
   - Use pre-built map for continuous position correction
   - Particle filter or scan matching for accuracy

---

## Key Learnings

1. [Learning 1]
2. [Learning 2]
3. [Learning 3]

---

## Questions for Instructor

1. [Question 1]
2. [Question 2]

---

**Estimated Time**: 2.5 hours
**Difficulty**: ★★☆☆☆ (Beginner-Intermediate)
```

---

## Submission

### Files to Submit

```
/shared/student_work/ex3_odometry_drift/
├── drift_analysis.md (main report)
├── trial_data.csv (optional: raw data)
└── screenshots/ (optional: RViz screenshots showing drift)
```

### Grading Rubric (100 points)

| Criterion | Excellent (90-100%) | Good (75-89%) | Satisfactory (60-74%) | Needs Improvement (<60%) | Points |
|-----------|---------------------|---------------|-----------------------|--------------------------|--------|
| **Experimental Data** | 4+ trials per config, complete data | 3 trials, mostly complete | 2 trials, some gaps | <2 trials or major gaps | /30 |
| **Error Analysis** | Quantitative analysis, calculations correct | Good analysis, minor errors | Basic analysis | Incomplete or incorrect | /25 |
| **Understanding** | Deep insights into drift causes | Good understanding | Surface-level understanding | Misunderstandings evident | /25 |
| **Written Quality** | Clear, well-organized, professional | Clear, organized | Readable but disorganized | Unclear or incomplete | /20 |

**Total**: _____ / 100

**Passing Grade**: 60/100

---

## Extension Challenges (Optional)

### Challenge 1: Drift Visualization
Create a Python script to visualize odometry vs. ground truth trajectory.

### Challenge 2: Different Environments
Test drift on different Gazebo worlds (turtlebot3_house, turtlebot3_stage_4).

### Challenge 3: Speed Impact
Does driving faster increase drift? Test with different velocities.

### Challenge 4: Sensor Noise Addition
Add simulated sensor noise in Gazebo, observe impact on drift.

---

## Common Issues & Troubleshooting

### Issue 1: Robot doesn't move
**Symptom**: Waypoint follower runs but robot stays still.

**Solution**:
```bash
# Check if /cmd_vel is being published
ros2 topic echo /cmd_vel

# Check if Gazebo is subscribed
ros2 topic info /cmd_vel
```

### Issue 2: Robot spins in place
**Symptom**: Robot rotates continuously without moving forward.

**Solution**: Check angle normalization logic, ensure quaternion to yaw conversion is correct.

### Issue 3: Waypoint follower exits immediately
**Symptom**: Script exits without navigating.

**Solution**: Ensure odometry callback is receiving data:
```bash
ros2 topic hz /odom  # Should show ~30-50 Hz
```

---

## Next Steps

After completing this exercise, you will:
1. **Appreciate the problem** SLAM solves
2. **Understand why** localization is necessary
3. **Be motivated** to learn SLAM Toolbox (Exercise 4)

In Exercise 4, you'll see how SLAM provides:
- **Loop closure**: Corrects drift when robot returns to known location
- **Map-based localization**: Continuous position correction
- **Global consistency**: Accurate maps over large areas

**The difference**:
- Odometry-only: 0.4m error after 4m path
- With SLAM: <0.05m error after 100m path (8x better!)

---

**Ready?** Let's prove that odometry alone isn't enough! 🤖
