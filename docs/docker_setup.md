# Docker Environment Setup Guide

This guide covers the Docker configuration and setup for the MUSI-nav2 learning environment.

---

## Prerequisites

### System Requirements

- **Operating System**: Ubuntu 22.04 (native or WSL2)
- **Docker**: Version 20.10 or higher
- **RAM**: 8 GB minimum (16 GB recommended)
- **Disk Space**: 10 GB free space
- **Display**: X11 server for GUI applications

### Required Software

#### 1. Install Docker and Docker Compose

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

**Important**: After adding yourself to the docker group, you must logout and login again for changes to take effect.

#### 2. X11 Server Setup

**Ubuntu Desktop**: X11 is already installed

**WSL2 Windows**:
- Install VcXsrv, or
- Use WSLg (Windows 11 has this built-in)

---

## Initial Setup

### Step 1: Clone and Build

```bash
# Clone the repository
git clone https://github.com/ericguerrero/MUSI-nav2.git
cd MUSI-nav2

# Build the Docker environment
./build.sh
```

**Build process** (~4 minutes):
- Creates workspace directories (`ros2_ws/`, `shared/`)
- Builds Docker image (~4.2 GB)
- Installs ROS2 Humble, Nav2, TurtleBot3, and all dependencies

### Step 2: Start the Environment

```bash
# Start the container
./run.sh
```

**Startup actions**:
- Enables X11 forwarding for GUI applications
- Starts Docker container in background
- Mounts workspace for persistent data

### Step 3: Access the Container

```bash
# Open a terminal inside the container
docker exec -it nav2_workspace bash
```

You're now inside the ROS2 environment with all packages ready!

---

## Verification

### Test Docker Installation

```bash
# Inside the container
ros2 pkg list | grep nav2          # Should show 20+ Nav2 packages
ros2 pkg list | grep turtlebot3    # Should show 38+ TurtleBot3 packages
```

### Test GUI Forwarding

```bash
# Inside container
rviz2
```

If RViz2 window appears, GUI forwarding is working correctly.

---

## Environment Management

### Start Environment
```bash
./run.sh
```

### Stop Environment
```bash
./stop.sh
```

Workspace data is preserved across restarts.

### Rebuild After Dockerfile Changes
```bash
docker-compose down
./build.sh
./run.sh
```

---

## Troubleshooting

### Permission Denied on Scripts

**Solution**:
```bash
chmod +x build.sh run.sh stop.sh
```

### GUI Applications Don't Display

**Solution 1** - Enable X11 forwarding:
```bash
xhost +local:docker
export DISPLAY=:0
docker-compose restart nav2_dev
```

**Solution 2** - WSL2 specific:
```bash
# Add to ~/.bashrc
export DISPLAY=$(cat /etc/resolv.conf | grep nameserver | awk '{print $2}'):0
source ~/.bashrc
```

### Docker Command Not Found

**Solution**:
```bash
sudo apt update
sudo apt install -y docker.io docker-compose
sudo usermod -aG docker $USER
# Logout and login again
```

### Out of Disk Space

**Solution**:
```bash
# Check available space
df -h

# Clean Docker system
docker system prune -a
```

---

## Resource Optimization (Optional)

For systems with limited resources (< 8GB RAM), see the main README.md "Reducing Resource Usage" section for:
- Headless Gazebo mode (saves ~500MB RAM)
- Lighter robot models
- Docker resource limits
