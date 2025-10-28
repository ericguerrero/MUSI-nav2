# Phase 1 Infrastructure Testing Plan

**Purpose**: Validate that the MUSI-nav2 Docker infrastructure works correctly before proceeding to Phase 2.

**Target Environment**: Ubuntu 22.04 WSL
**Estimated Time**: 15-20 minutes
**Last Updated**: 2025-10-18

---

## Test Categories

1. **Pre-Build Tests** - Verify prerequisites
2. **Build Tests** - Validate Docker image creation
3. **Runtime Tests** - Verify container functionality
4. **Persistence Tests** - Ensure data survives restarts
5. **Integration Tests** - End-to-end workflow validation

---

## 1. Pre-Build Tests (5 minutes)

### Test 1.1: Docker Availability
**Purpose**: Ensure Docker daemon is running and accessible

```bash
# Test command
docker --version
docker ps

# Expected output
# Docker version 28.5.1 or higher
# Container list (may be empty)

# Success criteria
# - Exit code: 0
# - Docker version displayed
# - No permission errors
```

**Result**: [ ] PASS / [ ] FAIL
**Notes**: ___________________________________________

---

### Test 1.2: Docker Compose Availability
**Purpose**: Verify Docker Compose V2 is installed

```bash
# Test command
docker compose version

# Expected output
# Docker Compose version v2.40.0 or higher

# Success criteria
# - Exit code: 0
# - Version v2.x displayed
```

**Result**: [ ] PASS / [ ] FAIL
**Notes**: ___________________________________________

---

### Test 1.3: Disk Space
**Purpose**: Ensure sufficient space for Docker image (~5-6 GB)

```bash
# Test command
df -h /home/ericguerrero

# Success criteria
# - At least 10 GB free space available
```

**Result**: [ ] PASS / [ ] FAIL
**Free Space**: ___________GB

---

### Test 1.4: X11 Socket
**Purpose**: Verify X11 forwarding prerequisites

```bash
# Test command
ls -la /tmp/.X11-unix/
echo $DISPLAY

# Expected output
# X11 socket files present (X0, X1, etc.)
# DISPLAY=:0 or similar

# Success criteria
# - X11 socket directory exists
# - DISPLAY variable is set
```

**Result**: [ ] PASS / [ ] FAIL
**DISPLAY value**: ___________________________________________

---

### Test 1.5: File Permissions
**Purpose**: Verify all scripts are executable

```bash
# Test command
ls -l *.sh

# Expected output
# All .sh files should have 'x' permission (rwxrwxr-x)

# Success criteria
# - build.sh is executable
# - run.sh is executable
# - stop.sh is executable
```

**Result**: [ ] PASS / [ ] FAIL
**Notes**: ___________________________________________

---

## 2. Build Tests (10-15 minutes)

### Test 2.1: Directory Structure Creation
**Purpose**: Verify build.sh creates required directories

```bash
# Test command
rm -rf ros2_ws shared  # Clean slate
./build.sh

# During build, observe:
# - "📁 Directory structure created" message appears
# - Directories are created before Docker build starts

# After build completes, verify:
ls -la ros2_ws/src
ls -la shared/maps
ls -la shared/configs
ls -la shared/bags
ls -la shared/student_work

# Success criteria
# - All directories exist
# - ros2_ws/src directory is empty (ready for packages)
# - shared/ subdirectories are empty
```

**Result**: [ ] PASS / [ ] FAIL
**Build Time**: ___________ minutes
**Notes**: ___________________________________________

---

### Test 2.2: Docker Image Build
**Purpose**: Verify Dockerfile builds without errors

```bash
# Test command
# (Already executed by ./build.sh in Test 2.1)

# Observe during build:
# - "🐳 Building Docker image..." message
# - Base image pulls successfully (osrf/ros:humble-desktop-full)
# - All apt packages install without errors
# - No DEBIAN_FRONTEND interactive prompts
# - Cache is cleaned (rm -rf /var/lib/apt/lists/*)
# - Build completes with "✅ Build complete!" message

# After build, verify image exists:
docker images nav2_student:humble

# Success criteria
# - Exit code: 0
# - Image tagged as nav2_student:humble
# - Image size approximately 4-6 GB
# - Build completes in < 15 minutes
```

**Result**: [ ] PASS / [ ] FAIL
**Image Size**: ___________GB
**Notes**: ___________________________________________

---

### Test 2.3: Image Layer Inspection
**Purpose**: Verify Dockerfile layers are correct

```bash
# Test command
docker history nav2_student:humble

# Success criteria
# - Base layer: osrf/ros:humble-desktop-full
# - Single RUN layer for all apt packages (efficiency)
# - Environment variables set (TURTLEBOT3_MODEL, GAZEBO_MODEL_PATH, ROS_DOMAIN_ID)
# - WORKDIR /ros2_ws
# - CMD ["/bin/bash"]
```

**Result**: [ ] PASS / [ ] FAIL
**Layer Count**: ___________
**Notes**: ___________________________________________

---

## 3. Runtime Tests (5 minutes)

### Test 3.1: Container Startup
**Purpose**: Verify run.sh starts container successfully

```bash
# Test command
./run.sh

# Observe:
# - "🖥️ Enabling X11 forwarding..." message
# - xhost command executes (may show warning, that's OK)
# - "🐳 Starting Docker container..." message
# - Container starts in detached mode
# - "✅ Nav2 development environment started!" message
# - Usage instructions displayed

# Verify container is running:
docker ps | grep nav2_workspace

# Success criteria
# - Exit code: 0
# - Container named 'nav2_workspace' is running
# - Container status is "Up"
# - No error messages
```

**Result**: [ ] PASS / [ ] FAIL
**Container Status**: ___________________________________________

---

### Test 3.2: Container Shell Access
**Purpose**: Verify interactive access to container

```bash
# Test command
docker exec -it nav2_workspace bash

# Inside container, verify:
whoami           # Should be 'root'
pwd              # Should be '/ros2_ws'
ls -la           # Should see src/ directory
echo $TURTLEBOT3_MODEL  # Should be 'waffle'
echo $ROS_DOMAIN_ID     # Should be '0'

# Exit container
exit

# Success criteria
# - Shell access works
# - Working directory is /ros2_ws
# - Environment variables are set correctly
# - Welcome message displayed on login
```

**Result**: [ ] PASS / [ ] FAIL
**Notes**: ___________________________________________

---

### Test 3.3: ROS2 Environment
**Purpose**: Verify ROS2 Humble is installed and sourced

```bash
# Test command
docker exec -it nav2_workspace bash -c "ros2 --version"

# Expected output
# ros2 cli version: <version>
# ros2 api version: <version>

docker exec -it nav2_workspace bash -c "source /opt/ros/humble/setup.bash && ros2 pkg list | head -5"

# Success criteria
# - ros2 command is available
# - ROS2 packages are listed
# - No errors about missing environment
```

**Result**: [ ] PASS / [ ] FAIL
**ROS2 Version**: ___________________________________________

---

### Test 3.4: Nav2 Packages Installed
**Purpose**: Verify all required Nav2 packages are present

```bash
# Test command
docker exec -it nav2_workspace bash -c "source /opt/ros/humble/setup.bash && ros2 pkg list | grep nav2"

# Expected packages (partial list):
# - nav2_amcl
# - nav2_bt_navigator
# - nav2_bringup
# - nav2_controller
# - nav2_planner
# - nav2_smoother
# - nav2_behaviors
# - nav2_waypoint_follower

# Success criteria
# - All major Nav2 packages are listed
# - No "package not found" errors
```

**Result**: [ ] PASS / [ ] FAIL
**Package Count**: ___________ nav2-* packages found
**Notes**: ___________________________________________

---

### Test 3.5: TurtleBot3 Packages Installed
**Purpose**: Verify TurtleBot3 simulation packages are present

```bash
# Test command
docker exec -it nav2_workspace bash -c "source /opt/ros/humble/setup.bash && ros2 pkg list | grep turtlebot3"

# Expected packages:
# - turtlebot3
# - turtlebot3_gazebo
# - turtlebot3_msgs
# - turtlebot3_teleop
# - turtlebot3_cartographer
# - turtlebot3_navigation2

# Success criteria
# - All TurtleBot3 packages are listed
```

**Result**: [ ] PASS / [ ] FAIL
**Notes**: ___________________________________________

---

### Test 3.6: SLAM Tools Installed
**Purpose**: Verify Cartographer and SLAM Toolbox are available

```bash
# Test command
docker exec -it nav2_workspace bash -c "source /opt/ros/humble/setup.bash && ros2 pkg list | grep -E 'cartographer|slam_toolbox'"

# Expected packages:
# - cartographer
# - cartographer_ros
# - slam_toolbox

# Success criteria
# - Both SLAM systems are available
```

**Result**: [ ] PASS / [ ] FAIL
**Notes**: ___________________________________________

---

### Test 3.7: Python Dependencies
**Purpose**: Verify transforms3d is installed for waypoint examples

```bash
# Test command
docker exec -it nav2_workspace bash -c "python3 -c 'import transforms3d; print(transforms3d.__version__)'"

# Success criteria
# - No ImportError
# - Version number displayed
```

**Result**: [ ] PASS / [ ] FAIL
**transforms3d version**: ___________________________________________

---

### Test 3.8: Development Tools
**Purpose**: Verify colcon, git, vim, tmux are available

```bash
# Test command
docker exec -it nav2_workspace bash -c "which colcon git vim tmux"

# Expected output:
# /usr/bin/colcon
# /usr/bin/git
# /usr/bin/vim
# /usr/bin/tmux

# Success criteria
# - All tools are in PATH and executable
```

**Result**: [ ] PASS / [ ] FAIL
**Notes**: ___________________________________________

---

## 4. Persistence Tests (3 minutes)

### Test 4.1: Volume Mount Verification
**Purpose**: Ensure host directories are mounted in container

```bash
# Test command
docker exec -it nav2_workspace bash -c "df -h | grep -E 'ros2_ws|shared'"

# Alternative check:
docker inspect nav2_workspace | grep -A 10 "Mounts"

# Success criteria
# - /ros2_ws is mounted from host ./ros2_ws
# - /shared is mounted from host ./shared
# - Both are read-write
```

**Result**: [ ] PASS / [ ] FAIL
**Notes**: ___________________________________________

---

### Test 4.2: Workspace Persistence
**Purpose**: Verify files in ros2_ws survive container restart

```bash
# Test commands
# 1. Create a test file in workspace
docker exec -it nav2_workspace bash -c "echo 'test content' > /ros2_ws/test_persistence.txt"

# 2. Verify file exists
docker exec -it nav2_workspace bash -c "cat /ros2_ws/test_persistence.txt"
# Should output: "test content"

# 3. Stop container
./stop.sh

# 4. Verify file exists on host
cat ros2_ws/test_persistence.txt
# Should output: "test content"

# 5. Restart container
./run.sh

# 6. Verify file still exists in container
docker exec -it nav2_workspace bash -c "cat /ros2_ws/test_persistence.txt"
# Should output: "test content"

# 7. Cleanup
docker exec -it nav2_workspace bash -c "rm /ros2_ws/test_persistence.txt"

# Success criteria
# - File created in container appears on host
# - File survives container stop/start cycle
# - Same content is readable after restart
```

**Result**: [ ] PASS / [ ] FAIL
**Notes**: ___________________________________________

---

### Test 4.3: Shared Directory Persistence
**Purpose**: Verify files in shared/ survive container restart

```bash
# Test commands
# 1. Create a test map file
docker exec -it nav2_workspace bash -c "echo 'map_data' > /shared/maps/test_map.txt"

# 2. Stop and restart
./stop.sh
./run.sh

# 3. Verify file persists
docker exec -it nav2_workspace bash -c "cat /shared/maps/test_map.txt"
# Should output: "map_data"

# 4. Cleanup
docker exec -it nav2_workspace bash -c "rm /shared/maps/test_map.txt"

# Success criteria
# - Files in /shared persist across restarts
```

**Result**: [ ] PASS / [ ] FAIL
**Notes**: ___________________________________________

---

### Test 4.4: Container Rebuild Persistence
**Purpose**: Verify data survives Docker image rebuild

```bash
# Test commands
# 1. Create test data
docker exec -it nav2_workspace bash -c "echo 'important data' > /ros2_ws/important.txt"

# 2. Stop container and remove
./stop.sh
docker rm nav2_workspace

# 3. Rebuild image (no code changes, but forces rebuild)
docker build -t nav2_student:humble --no-cache .

# 4. Restart container
./run.sh

# 5. Verify data still exists
docker exec -it nav2_workspace bash -c "cat /ros2_ws/important.txt"
# Should output: "important data"

# 6. Cleanup
docker exec -it nav2_workspace bash -c "rm /ros2_ws/important.txt"

# Success criteria
# - Data in volume mounts survives image rebuild
```

**Result**: [ ] PASS / [ ] FAIL
**Notes**: ___________________________________________

---

## 5. Integration Tests (5-10 minutes)

### Test 5.1: ROS2 Topic Communication
**Purpose**: Verify ROS2 DDS networking works (host network mode)

```bash
# Test commands
# Terminal 1: Start container
./run.sh

# Terminal 2: Publish test topic
docker exec -it nav2_workspace bash -c "source /opt/ros/humble/setup.bash && ros2 topic pub -1 /test_topic std_msgs/msg/String '{data: \"hello\"}'"

# Terminal 3: Echo topic
docker exec -it nav2_workspace bash -c "source /opt/ros/humble/setup.bash && timeout 3 ros2 topic echo /test_topic"

# Expected output in Terminal 3:
# data: hello
# ---

# Success criteria
# - Topic publishes successfully
# - Topic can be echoed in different terminal
# - No network errors or DDS warnings
```

**Result**: [ ] PASS / [ ] FAIL
**Notes**: ___________________________________________

---

### Test 5.2: Colcon Workspace Build
**Purpose**: Verify ROS2 workspace can build packages

```bash
# Test commands
docker exec -it nav2_workspace bash -c "
source /opt/ros/humble/setup.bash &&
cd /ros2_ws &&
ros2 pkg create --build-type ament_python test_pkg --dependencies rclpy &&
colcon build --packages-select test_pkg &&
source install/setup.bash &&
ros2 pkg list | grep test_pkg
"

# Expected output:
# - Package created successfully
# - Build completes without errors
# - test_pkg appears in package list

# Cleanup:
docker exec -it nav2_workspace bash -c "rm -rf /ros2_ws/src/test_pkg /ros2_ws/build /ros2_ws/install /ros2_ws/log"

# Success criteria
# - Package creation works
# - colcon build succeeds
# - Built package is discoverable
```

**Result**: [ ] PASS / [ ] FAIL
**Build Time**: ___________ seconds
**Notes**: ___________________________________________

---

### Test 5.3: Gazebo Launch Test (Headless)
**Purpose**: Verify TurtleBot3 Gazebo simulation can start (no GUI check)

```bash
# Test command (headless mode, no GUI)
docker exec -it nav2_workspace bash -c "
source /opt/ros/humble/setup.bash &&
timeout 30 ros2 launch turtlebot3_gazebo empty_world.launch.py 2>&1 | grep -E 'Process|Error'
"

# Expected behavior:
# - Gazebo processes start
# - No fatal errors
# - Times out after 30 seconds (expected)

# Note: This test does NOT require GUI (X11)
# GUI functionality will be tested manually later

# Success criteria
# - No "package not found" errors
# - No missing model errors
# - Gazebo processes initialize (even if GUI fails)
```

**Result**: [ ] PASS / [ ] FAIL / [ ] SKIP (X11 not available)
**Notes**: ___________________________________________

---

### Test 5.4: Stop Script
**Purpose**: Verify graceful shutdown works

```bash
# Test commands
# 1. Ensure container is running
docker ps | grep nav2_workspace

# 2. Run stop script
./stop.sh

# Observe:
# - "🛑 Stopping..." message
# - docker-compose down executes
# - "✅ Environment stopped" message
# - Data preservation notice displayed

# 3. Verify container stopped
docker ps | grep nav2_workspace
# Should return nothing (container stopped)

# Success criteria
# - Exit code: 0
# - Container is stopped
# - No orphaned processes
# - Clear user feedback
```

**Result**: [ ] PASS / [ ] FAIL
**Notes**: ___________________________________________

---

### Test 5.5: Full Restart Cycle
**Purpose**: Verify complete stop → start cycle works

```bash
# Test commands
# 1. Stop (if running)
./stop.sh

# 2. Start fresh
./run.sh

# 3. Verify container is running
docker ps | grep nav2_workspace

# 4. Access container
docker exec -it nav2_workspace bash -c "echo 'Restart cycle successful'"

# Success criteria
# - Container starts cleanly after stop
# - No stale processes or ports
# - All functionality available
```

**Result**: [ ] PASS / [ ] FAIL
**Notes**: ___________________________________________

---

## 6. Error Handling Tests (2 minutes)

### Test 6.1: Build Script - Missing Image Error
**Purpose**: Verify run.sh fails gracefully if image doesn't exist

```bash
# Test commands
# 1. Stop and remove container and image
./stop.sh
docker rmi nav2_student:humble

# 2. Try to run without building
./run.sh

# Expected behavior:
# - "❌ Docker image 'nav2_student:humble' not found." message
# - "Please run ./build.sh first" guidance
# - Exit code: 1

# 3. Rebuild for next tests
./build.sh

# Success criteria
# - Clear error message
# - Helpful guidance provided
# - Script exits with error code
```

**Result**: [ ] PASS / [ ] FAIL
**Notes**: ___________________________________________

---

### Test 6.2: Docker Daemon Down Error
**Purpose**: Verify scripts handle Docker daemon not running

```bash
# NOTE: This test requires stopping Docker daemon
# ONLY run if you can safely restart Docker

# Test commands (OPTIONAL):
# 1. Stop Docker daemon
# sudo systemctl stop docker (on systemd systems)

# 2. Try to build
# ./build.sh

# Expected:
# - Error about Docker not available
# - Helpful error message

# 3. Restart Docker
# sudo systemctl start docker

# Success criteria
# - Graceful error handling
# - Clear user guidance
```

**Result**: [ ] PASS / [ ] FAIL / [ ] SKIP
**Notes**: ___________________________________________

---

## Test Summary

### Statistics
- **Total Tests**: 27
- **Tests Passed**: _____ / 27
- **Tests Failed**: _____ / 27
- **Tests Skipped**: _____ / 27

### Pass Rate
- **Pre-Build Tests**: _____ / 5 (____%)
- **Build Tests**: _____ / 3 (____%)
- **Runtime Tests**: _____ / 8 (____%)
- **Persistence Tests**: _____ / 4 (____%)
- **Integration Tests**: _____ / 5 (____%)
- **Error Handling Tests**: _____ / 2 (____%)

---

## Critical Issues Found

| Test # | Issue Description | Severity | Resolution |
|--------|------------------|----------|------------|
|        |                  |          |            |
|        |                  |          |            |
|        |                  |          |            |

**Severity Levels**: CRITICAL (blocks usage) / HIGH (major issue) / MEDIUM (workaround exists) / LOW (minor)

---

## Phase 1 Exit Criteria

**Must Pass (MVP Ready)**:
- [ ] All Pre-Build Tests (5/5)
- [ ] Docker image builds successfully (Test 2.2)
- [ ] Container starts and stops cleanly (Tests 3.1, 5.4)
- [ ] ROS2 environment is functional (Test 3.3)
- [ ] Nav2 packages are installed (Test 3.4)
- [ ] Workspace persistence works (Test 4.2)
- [ ] ROS2 communication works (Test 5.1)

**Should Pass (Enhanced Quality)**:
- [ ] All Runtime Tests (8/8)
- [ ] All Persistence Tests (4/4)
- [ ] Colcon build works (Test 5.2)

**Nice to Have (Full Validation)**:
- [ ] Gazebo launches without errors (Test 5.3)
- [ ] Error handling is graceful (Tests 6.1, 6.2)

---

## Recommendations

**Before Phase 2 (Documentation)**:
1. All "Must Pass" criteria should be met
2. Document any known issues in troubleshooting guide
3. Test on fresh Ubuntu 22.04 WSL environment if possible

**Performance Benchmarks**:
- Docker build time: ______ minutes (target: < 15 min)
- Container startup time: ______ seconds (target: < 10 sec)
- Image size: ______ GB (target: < 6 GB)

---

## Tester Information

**Tested By**: ___________________________________________
**Date**: ___________________________________________
**Environment**: Ubuntu 22.04 WSL / Docker _____ / Compose _____
**Notes**: ___________________________________________

---

## Next Steps

After completing this test plan:

1. **If all "Must Pass" tests pass**: ✅ Ready for Phase 2 (Documentation)
2. **If any critical issues found**: 🔧 Fix issues before proceeding
3. **If minor issues found**: 📝 Document in troubleshooting guide, proceed to Phase 2

**Manual GUI Test** (requires VcXsrv/WSLg on Windows):
```bash
# After all tests pass, manually verify:
docker exec -it nav2_workspace bash
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py

# Expected: Gazebo window opens with TurtleBot3 visible
```
