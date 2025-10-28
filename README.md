# MUSI-nav2: ROS2 Navigation Learning Environment

A turnkey Docker-based educational environment for learning ROS2 Navigation (Nav2) with TurtleBot3 simulation.

## Overview

This project provides a complete, containerized ROS2 Humble environment with:
- Nav2 navigation stack
- TurtleBot3 Waffle robot simulation
- Gazebo for realistic physics simulation
- RViz2 for visualization
- SLAM tools (Cartographer, SLAM Toolbox)

**Goal**: Eliminate setup friction and enable students to focus on learning autonomous robot navigation.

---

## Prerequisites

### System Requirements

**Standard Configuration**:
- **Operating System**: Ubuntu 22.04 (native or WSL2)
- **Docker**: Version 20.10 or higher
- **RAM**: 8 GB minimum (16 GB recommended)
- **Disk Space**: 10 GB free space
- **Display**: X11 server for GUI applications

**Low-Spec Configuration** (4-6 GB RAM):
- Same as above, but use resource-saving options (see "Reducing Resource Usage" section)
- Headless Gazebo + lightweight robot model
- Can run on systems with 4 GB RAM (with reduced features)

### Required Software

1. **Docker and Docker Compose**
   ```bash
   # Install Docker
   sudo apt update
   sudo apt install -y docker.io docker-compose

   # Add your user to docker group (logout/login required after this)
   sudo usermod -aG docker $USER

   # Verify installation
   docker --version
   docker-compose --version
   ```

2. **X11 Server** (for GUI applications)
   - **Ubuntu Desktop**: Already installed
   - **WSL2**: Install VcXsrv or use WSLg (Windows 11)

---

## Quick Start (5 Minutes)

### Step 1: Clone and Build

```bash
# Clone the repository
git clone <repository-url>
cd MUSI-nav2

# Build the Docker environment
./build.sh
```

The build process will:
- Create necessary directories (`ros2_ws/`, `shared/`)
- Build the Docker image (~4 minutes, ~4.2 GB)
- Install ROS2 Humble, Nav2, TurtleBot3, and all dependencies

### Step 2: Start the Environment

```bash
# Start the container
./run.sh
```

This will:
- Enable X11 forwarding for GUI applications
- Start the Docker container in the background
- Mount your workspace for persistent data

### Step 3: Access the Container

```bash
# Open a terminal inside the container
docker exec -it nav2_workspace bash
```

You're now inside the ROS2 environment with all packages ready to use!

---

## First Navigation Simulation

Follow these steps to launch your first TurtleBot3 simulation:

### Terminal 1: Launch Gazebo Simulation

```bash
# Access the container
docker exec -it nav2_workspace bash

# Launch TurtleBot3 in Gazebo
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
```

**Wait 30-60 seconds** for Gazebo to fully load. You'll see:
- A Gazebo window with a simulated indoor environment
- A TurtleBot3 Waffle robot with a LiDAR sensor on top

**Note**: If you see "not responding" dialog, click "Wait" - this is normal during first launch while shaders compile.

### Terminal 2: Launch RViz2 Visualization

```bash
# Open another terminal in the container
docker exec -it nav2_workspace bash

# Launch RViz2
rviz2
```

**Configure RViz2**:
1. In the left panel, find "Global Options" → "Fixed Frame"
2. Change from "map" to **"odom"**
3. Click "Add" button (bottom left)
4. Add "LaserScan" display:
   - Select "By topic" tab
   - Choose `/scan` → LaserScan
   - Click OK
5. Add "RobotModel" display:
   - Click "Add" again
   - Select "By display type" tab
   - Choose "RobotModel"
   - Click OK

You should now see the robot model and LiDAR data visualized in RViz2!

### Terminal 3: Control the Robot (Optional)

```bash
# Open another terminal in the container
docker exec -it nav2_workspace bash

# Launch keyboard teleoperation
ros2 run turtlebot3_teleop teleop_keyboard
```

**Controls**:
- `w` - Move forward
- `x` - Move backward
- `a` - Turn left
- `d` - Turn right
- `s` - Stop
- `Ctrl+C` - Exit teleop

Watch the robot move in Gazebo and see the LiDAR scan update in RViz2!

---

## Verifying Your Installation

Run these commands to verify everything is working:

```bash
# Inside the container, check ROS2 installation
ros2 pkg list | grep nav2          # Should show 20+ Nav2 packages
ros2 pkg list | grep turtlebot3    # Should show 38+ TurtleBot3 packages

# Check active ROS2 topics (run after launching simulation)
ros2 topic list                    # Should show /scan, /odom, /cmd_vel, etc.

# Check TF frames
ros2 run tf2_tools view_frames     # Generates frames.pdf with transform tree
```

---

## Understanding the Environment

### Directory Structure

```
MUSI-nav2/
├── Dockerfile              # Docker image definition
├── docker-compose.yml      # Container configuration
├── build.sh                # Build automation script
├── run.sh                  # Startup script
├── stop.sh                 # Shutdown script
│
├── ros2_ws/                # Your ROS2 workspace (persistent)
│   └── src/                # Put your custom packages here
│
└── shared/                 # Shared resources
    ├── maps/               # SLAM-generated maps
    ├── configs/            # Nav2 configurations
    └── student_work/       # Your deliverables
```

### Key Components

**TurtleBot3 Waffle**: The robot platform for this course
- Differential drive mobile robot
- 360° LiDAR sensor (for mapping and obstacle detection)
- RGB camera
- IMU (Inertial Measurement Unit)

**Gazebo**: Physics-based 3D simulator
- Simulates robot dynamics and sensors
- Provides realistic environments for testing

**RViz2**: 3D visualization tool
- Visualize sensor data (LiDAR, camera, transforms)
- Display maps and planned paths
- Set navigation goals

**Nav2**: ROS2 Navigation Stack
- Path planning algorithms
- Obstacle avoidance
- Localization and mapping
- Behavior coordination

---

## Common Workflows

### Creating a New ROS2 Package

```bash
# Inside the container
cd /ros2_ws/src

# Create a Python package
ros2 pkg create --build-type ament_python my_nav_package

# Build the workspace
cd /ros2_ws
colcon build --symlink-install

# Source the workspace
source install/setup.bash

# Run your node
ros2 run my_nav_package my_node
```

### SLAM Mapping (Create a Map)

```bash
# Terminal 1: Launch TurtleBot3 simulation
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py

# Terminal 2: Launch SLAM with Cartographer
ros2 launch turtlebot3_cartographer cartographer.launch.py use_sim_time:=True

# Terminal 3: Teleoperation to drive and map the environment
ros2 run turtlebot3_teleop teleop_keyboard

# Terminal 4: Save the map when done
ros2 run nav2_map_server map_saver_cli -f /shared/maps/my_map
```

Your map will be saved as:
- `/shared/maps/my_map.pgm` (image)
- `/shared/maps/my_map.yaml` (metadata)

### Autonomous Navigation (Use a Map)

```bash
# Terminal 1: Launch TurtleBot3 simulation
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py

# Terminal 2: Launch Nav2 with your saved map
ros2 launch nav2_bringup navigation_launch.py \
    use_sim_time:=True \
    map:=/shared/maps/my_map.yaml

# In RViz2: Use "2D Pose Estimate" to set initial robot position
# Then use "Nav2 Goal" to send navigation goals
```

---

## Reducing Resource Usage (Low-Spec Systems)

If your computer has limited resources (< 8 GB RAM or older CPU), you can significantly reduce resource consumption:

### Quick Comparison

| Configuration | CPU Usage | RAM Usage | Startup Time | Use Case |
|---------------|-----------|-----------|--------------|----------|
| **Full** (Waffle + GUI + World) | ~150% | ~2.5 GB | ~45s | Full-featured learning |
| **Headless** (GUI off) | ~40% | ~2.0 GB | ~20s | Background simulation |
| **Burger Model** | ~100% | ~2.2 GB | ~35s | Lighter robot |
| **Empty World** | ~80% | ~1.8 GB | ~25s | Basic testing |
| **Low-Spec** (Burger + Headless + Empty) | ~30% | ~1.2 GB | ~8s | Minimal systems |

### Option 1: Use Headless Gazebo (No GUI)

Instead of full Gazebo, use the headless server (saves ~60% CPU and ~500 MB RAM):

```bash
# Terminal 1: Launch headless Gazebo (no GUI)
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py gui:=false

# Terminal 2: Use RViz2 for visualization (lighter than Gazebo GUI)
rviz2
```

**Benefits**:
- Gazebo GUI: ~150% CPU → Headless: ~40% CPU
- Visualize everything you need in RViz2
- Faster startup (~10 seconds vs ~45 seconds)

### Option 2: Use TurtleBot3 Burger (Lighter Model)

The Burger model is lighter than Waffle (no camera, simpler):

```bash
# Set environment variable for Burger model
export TURTLEBOT3_MODEL=burger

# Launch with Burger
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
```

**Benefits**:
- ~30% faster simulation
- Burger still has LiDAR (sufficient for navigation learning)

### Option 3: Use Empty World

Skip the detailed world for basic testing:

```bash
# Launch in empty world (minimal environment)
ros2 launch turtlebot3_gazebo turtlebot3_empty_world.launch.py
```

**Benefits**:
- ~50% less CPU usage
- Faster startup
- Good for initial testing before moving to complex environments

### Option 4: Limit Docker Resources

Restrict Docker resource usage in `docker-compose.yml`:

```yaml
services:
  nav2_dev:
    # ... existing configuration ...
    deploy:
      resources:
        limits:
          cpus: '4.0'      # Limit to 4 CPU cores
          memory: 4G        # Limit to 4GB RAM
        reservations:
          memory: 2G        # Reserve at least 2GB
```

Then restart:
```bash
docker-compose down
docker-compose up -d
```

### Option 5: Reduce Gazebo Physics Rate

Create `~/.gazebo/gui.ini` inside the container:

```bash
docker exec -it nav2_workspace bash
mkdir -p ~/.gazebo
echo "[physics]
max_step_size = 0.01
real_time_update_rate = 100" > ~/.gazebo/gui.ini
```

**Benefits**:
- Default: 1000 Hz physics → Reduced: 100 Hz
- Still accurate for navigation, but ~40% less CPU

### Recommended Low-Spec Combination

For systems with 4-6 GB RAM:

```bash
# Use headless Gazebo + Burger + empty world
export TURTLEBOT3_MODEL=burger
ros2 launch turtlebot3_gazebo turtlebot3_empty_world.launch.py gui:=false

# Visualize in RViz2 (separate terminal)
rviz2
```

**Resource savings**:
- CPU usage: ~150% → ~30-40%
- RAM usage: ~2.5 GB → ~1.2 GB
- Startup time: ~45s → ~8s

### Performance Monitoring

Check resource usage:

```bash
# Monitor Docker container resources
docker stats nav2_workspace

# Inside container, check process usage
docker exec -it nav2_workspace bash
top  # Press 'q' to quit
```

---

## Environment Management

### Start the Environment

```bash
./run.sh
```

### Stop the Environment

```bash
./stop.sh
```

This gracefully shuts down the container while preserving all your workspace data.

### Rebuild After Changes

```bash
# If you modify the Dockerfile
docker-compose down
./build.sh
./run.sh
```

### Clean Docker System

```bash
# Remove unused Docker resources (frees disk space)
docker system prune -a
```

**Warning**: This removes all unused Docker images and containers!

---

## Troubleshooting

### Issue: "Permission denied" when running scripts

**Solution**:
```bash
chmod +x build.sh run.sh stop.sh
```

### Issue: GUI applications don't display (Gazebo/RViz2)

**Solution 1** - Enable X11 forwarding:
```bash
xhost +local:docker
export DISPLAY=:0
docker-compose restart nav2_dev
```

**Solution 2** - Check X11 socket permissions:
```bash
ls -la /tmp/.X11-unix/
# Should show X0 socket
```

**Solution 3** - WSL2 specific:
```bash
# Add to ~/.bashrc
export DISPLAY=$(cat /etc/resolv.conf | grep nameserver | awk '{print $2}'):0
source ~/.bashrc
```

### Issue: Gazebo shows "not responding" dialog

**Solution**: This is normal on first launch. Click **"Wait"** and give it 30-60 seconds for shader compilation. Subsequent launches will be faster.

### Issue: RViz2 error "frame [map] does not exist"

**Solution**: Change the Fixed Frame in RViz2:
1. Left panel → Global Options → Fixed Frame
2. Change from "map" to **"odom"**

### Issue: ROS2 nodes can't communicate between terminals

**Solution**: The container uses host networking, which should work automatically. Verify:
```bash
# Check ROS_DOMAIN_ID is consistent
echo $ROS_DOMAIN_ID  # Should be 0

# In different terminals, run:
# Terminal 1:
ros2 topic pub /test_topic std_msgs/String "data: hello"

# Terminal 2:
ros2 topic echo /test_topic
```

### Issue: "docker: command not found"

**Solution**: Install Docker:
```bash
sudo apt update
sudo apt install -y docker.io docker-compose
sudo usermod -aG docker $USER
# Logout and login again
```

### Issue: Container starts but immediately exits

**Solution**: Check logs:
```bash
docker logs nav2_workspace
```

### Issue: Out of disk space during build

**Solution**:
```bash
# Check available space
df -h

# Clean Docker system
docker system prune -a

# Clean apt cache (inside container)
docker exec nav2_workspace bash -c "apt clean"
```

---

## ROS2 Essential Commands

### Topics (Message Streams)

```bash
# List all topics
ros2 topic list

# Show topic info
ros2 topic info /scan

# Echo messages from a topic
ros2 topic echo /scan

# Show message rate
ros2 topic hz /scan

# Publish to a topic
ros2 topic pub /cmd_vel geometry_msgs/Twist "{linear: {x: 0.2}, angular: {z: 0.0}}"
```

### Nodes (Running Programs)

```bash
# List all nodes
ros2 node list

# Show node info
ros2 node info /turtlebot3_node

# Run a node
ros2 run <package_name> <node_name>
```

### Services (Request/Response)

```bash
# List all services
ros2 service list

# Call a service
ros2 service call /reset_simulation std_srvs/Empty
```

### Transforms (TF)

```bash
# View transform tree
ros2 run tf2_tools view_frames

# Echo a transform
ros2 run tf2_ros tf2_echo odom base_link
```

### Launch Files (Multi-Node Startup)

```bash
# Launch a launch file
ros2 launch <package_name> <launch_file>

# Example
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
```

---

## Getting Help

### ROS2 Documentation
- Official Nav2 Docs: https://docs.nav2.org/
- ROS2 Humble Docs: https://docs.ros.org/en/humble/
- TurtleBot3 Manual: https://emanual.robotis.com/docs/en/platform/turtlebot3/

### Useful Commands

```bash
# Get help for any ROS2 command
ros2 <command> --help

# Example
ros2 topic --help
ros2 launch --help
```

### Package Documentation

```bash
# Get package info
ros2 pkg list
ros2 pkg prefix <package_name>

# Find executables in a package
ros2 pkg executables turtlebot3_teleop
```

---

## Next Steps

Now that your environment is set up, you're ready to start learning Nav2! Recommended learning path:

1. **Exercise 1**: Environment verification (you just completed this!)
2. **Exercise 2**: Teleoperation and sensor data exploration
3. **Exercise 3**: SLAM mapping with Cartographer
4. **Exercise 4**: Localization with AMCL
5. **Exercise 5**: Autonomous navigation with Nav2
6. **Exercise 6**: Waypoint following
7. **Exercise 7**: Parameter tuning for better performance
8. **Exercise 8**: Custom behaviors and plugins
9. **Exercise 9**: Dynamic obstacle avoidance
10. **Exercise 10**: Complete navigation challenge

Refer to the `docs/exercises/` directory for detailed exercise instructions.

---

## Tips for Success

1. **Save your work frequently**: Your `ros2_ws/` directory persists across container restarts
2. **Use multiple terminals**: Most ROS2 workflows require 2-3 terminals
3. **Read the error messages**: ROS2 provides detailed error information
4. **Check topic data**: Use `ros2 topic echo` to debug communication issues
5. **Visualize in RViz2**: Always verify sensor data visually
6. **Start simple**: Test individual components before combining them
7. **Use simulation time**: Always set `use_sim_time:=True` in launch files when using Gazebo

---

## Contributing

Found a bug or have a suggestion? Please open an issue or submit a pull request.

---

## License

This educational project is provided for learning purposes. See LICENSE file for details.

---

**Happy Learning!** 🤖🗺️
