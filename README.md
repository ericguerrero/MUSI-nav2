# MUSI-nav2: ROS2 Navigation Learning Environment

## Overview

This project provides a complete, containerized ROS2 Humble environment with:
- Nav2 navigation stack
- TurtleBot3 Waffle robot simulation
- Gazebo for realistic physics simulation
- RViz2 for visualization
- SLAM tools

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

#### Linux (Ubuntu 20.04/22.04)

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

2. **X11 Server**: Already installed on Ubuntu Desktop

#### WSL2 (Windows)

1. **Docker Desktop for Windows** (Recommended)
   - Download and install [Docker Desktop](https://www.docker.com/products/docker-desktop)
   - During installation, ensure **"Use WSL 2 based engine"** is enabled
   - After installation, open Docker Desktop → Settings → Resources → WSL Integration
   - Enable integration with your Ubuntu distribution (e.g., Ubuntu-20.04)
   - Start Docker Desktop (it must be running for Docker commands to work in WSL)

   **Verify installation in WSL terminal**:
   ```bash
   docker --version
   docker-compose --version

   # Check Docker is running
   docker ps
   ```

2. **X11 Server** (for GUI applications)
   - **Windows 11**: WSLg is built-in (no setup needed)
   - **Windows 10**: Install [VcXsrv](https://sourceforge.net/projects/vcxsrv/) or [X410](https://x410.dev/)
     - Launch XLaunch with settings: Multiple windows, Start no client, Disable access control
     - Add to `~/.bashrc`:
       ```bash
       export DISPLAY=$(cat /etc/resolv.conf | grep nameserver | awk '{print $2}'):0
       ```

**Important**: Docker Desktop must be running on Windows before using Docker commands in WSL.

#### macOS

1. **Docker Desktop for Mac**
   - Download and install [Docker Desktop for Mac](https://www.docker.com/products/docker-desktop)
   - Docker Desktop includes Docker Compose
   - Start Docker Desktop from Applications

   **Verify installation**:
   ```bash
   docker --version
   docker-compose --version
   ```

2. **X11 Server** (for GUI applications)
   - Install [XQuartz](https://www.xquartz.org/)
   - Launch XQuartz → Preferences → Security → Enable "Allow connections from network clients"
   - Restart XQuartz
   - In terminal:
     ```bash
     # Allow X11 forwarding
     xhost +localhost

     # Set DISPLAY variable
     export DISPLAY=host.docker.internal:0
     ```

**Note**: On Apple Silicon (M1/M2/M3), Docker will use Rosetta 2 for x86_64 images. Performance may vary.

---

## Quick Start (5 Minutes)

### Step 1: Clone and Build

```bash
# Clone the repository
git clone https://github.com/ericguerrero/MUSI-nav2.git
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

To verify your installation and get started with Nav2, follow the official Nav2 tutorials:

### Quick Verification

```bash
# Access the container
docker exec -it nav2_workspace bash

# Launch TurtleBot3 in Gazebo
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
```

**Wait 30-60 seconds** for Gazebo to fully load. You should see a Gazebo window with a simulated indoor environment and a TurtleBot3 Waffle robot.

**Note**: If you see "not responding" dialog, click "Wait" - this is normal during first launch while shaders compile.

### Learning Path

Once your environment is running, follow the official Nav2 documentation for comprehensive tutorials:

**Official Nav2 Tutorials**: https://docs.nav2.org/tutorials/index.html

Recommended learning sequence:
1. **Getting Started**: https://docs.nav2.org/getting_started/index.html
2. **SLAM and Navigation**: https://docs.nav2.org/tutorials/docs/navigation2_with_slam.html
3. **Using Nav2 with TurtleBot3**: https://docs.nav2.org/tutorials/docs/get_backtrace.html

These tutorials will guide you through:
- Teleoperation and sensor exploration
- SLAM mapping (Cartographer/SLAM Toolbox)
- Localization (AMCL)
- Autonomous navigation
- Waypoint following
- Parameter tuning
- Custom behaviors

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

## Directory Structure

```
MUSI-nav2/
├── Dockerfile              # Docker image definition
├── docker-compose.yml      # Container configuration
├── build.sh                # Build automation script
├── run.sh                  # Startup script
├── stop.sh                 # Shutdown script
├── README.md               # This file
│
├── ros2_ws/                # Your ROS2 workspace (persistent)
│   └── src/                # Put your custom ROS2 packages here
│
└── shared/                 # Shared resources (accessible from host and container)
    ├── bags/               # ROS2 bag recordings (for data logging)
    ├── configs/            # Nav2 parameter configurations
    ├── maps/               # SLAM-generated maps (.pgm and .yaml files)
    └── worlds/             # Custom Gazebo world files
        └── landmark_world.world  # Example world with landmarks
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

### Issue: "Cannot connect to the Docker daemon" or "Is the docker daemon running?"

**WSL2 Solution**:
1. Ensure Docker Desktop is running on Windows
2. Verify WSL integration is enabled:
   - Open Docker Desktop → Settings → Resources → WSL Integration
   - Enable for your Ubuntu distribution
3. Restart Docker Desktop if needed
4. Test in WSL: `docker ps`

**Linux Solution**:
```bash
# Start Docker service
sudo systemctl start docker
sudo systemctl enable docker

# Add user to docker group if not already done
sudo usermod -aG docker $USER
# Logout and login again
```

**macOS Solution**:
1. Ensure Docker Desktop is running (check menu bar icon)
2. Restart Docker Desktop if needed
3. Test: `docker ps`

### Issue: "Permission denied" when running scripts

**Solution**:
```bash
chmod +x build.sh run.sh stop.sh
```

### Issue: GUI applications don't display (Gazebo/RViz2)

**Linux Solution**:
```bash
# Enable X11 forwarding
xhost +local:docker
export DISPLAY=:0
docker-compose restart nav2_dev
```

**WSL2 Solution**:
```bash
# Windows 11 (WSLg) - should work automatically
# If not, add to ~/.bashrc:
export DISPLAY=:0

# Windows 10 (VcXsrv/X410) - add to ~/.bashrc:
export DISPLAY=$(cat /etc/resolv.conf | grep nameserver | awk '{print $2}'):0
source ~/.bashrc

# Ensure X server is running on Windows
```

**macOS Solution**:
```bash
# Ensure XQuartz is running
# Enable X11 forwarding
xhost +localhost
export DISPLAY=host.docker.internal:0

# Restart container
docker-compose restart nav2_dev
```

**Check X11 socket** (Linux/WSL):
```bash
ls -la /tmp/.X11-unix/
# Should show X0 socket
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

For a comprehensive guide to ROS2 command-line tools, refer to the official documentation:

**ROS2 CLI Tutorial**: https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools.html

This covers:
- Topics (pub/sub messaging)
- Nodes (running programs)
- Services (request/response)
- Parameters (runtime configuration)
- Actions (long-running tasks)
- Launch files (multi-node startup)

**Quick reference**:
```bash
# Get help for any ROS2 command
ros2 <command> --help

# Examples
ros2 topic list          # List all topics
ros2 node list           # List all nodes
ros2 pkg list            # List all packages
```

---

## Additional Resources

- **Nav2 Documentation**: https://docs.nav2.org/
- **ROS2 Humble Documentation**: https://docs.ros.org/en/humble/
- **TurtleBot3 Manual**: https://emanual.robotis.com/docs/en/platform/turtlebot3/

---

## License

This educational project is provided for learning purposes. See LICENSE file for details.
