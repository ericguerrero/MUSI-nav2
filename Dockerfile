FROM osrf/ros:humble-desktop-full

# Metadata for documentation
LABEL maintainer="Nav2 Student Tutorial"
LABEL description="Complete ROS2 Nav2 development environment for students"
LABEL version="1.0"

# Avoid interactive prompts during build
ENV DEBIAN_FRONTEND=noninteractive

# Install all necessary packages in single layer for efficiency.
#
# ros2_control and its controllers give students a standard interface for
# swapping kinematics: ackermann_steering_controller, mecanum_drive_controller,
# tricycle_steering_controller and diff_drive_controller are configured from
# YAML rather than hand-written Gazebo plugin blocks. gazebo_ros2_control is the
# bridge that lets those controllers drive Gazebo Classic.
#
# matplotlib backs the trajectory plots in the evaluation harness.
RUN apt-get update && apt-get install -y \
    ros-humble-navigation2 \
    ros-humble-nav2-bringup \
    ros-humble-turtlebot3* \
    ros-humble-gazebo-ros-pkgs \
    ros-humble-cartographer \
    ros-humble-cartographer-ros \
    ros-humble-slam-toolbox \
    ros-humble-ros2-control \
    ros-humble-ros2-controllers \
    ros-humble-gazebo-ros2-control \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-pip \
    git \
    vim \
    nano \
    tmux \
    wget \
    curl \
    htop \
    && rm -rf /var/lib/apt/lists/*

# Install Python dependencies for Nav2 examples and SLAM metrics
RUN pip3 install --no-cache-dir transforms3d numpy pillow pyyaml scipy matplotlib

# Create workspace
WORKDIR /ros2_ws
RUN mkdir -p src

# Environment setup for TurtleBot3
ENV TURTLEBOT3_MODEL=waffle
ENV GAZEBO_MODEL_PATH=/opt/ros/humble/share/turtlebot3_gazebo/models
ENV ROS_DOMAIN_ID=0

# Configure bashrc for automatic sourcing and welcome message
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc && \
    echo "if [ -f /ros2_ws/install/setup.bash ]; then source /ros2_ws/install/setup.bash; fi" >> ~/.bashrc && \
    echo "echo ''" >> ~/.bashrc && \
    echo "echo '🤖 Nav2 Student Environment Ready!'" >> ~/.bashrc && \
    echo "echo 'Workspace: /ros2_ws'" >> ~/.bashrc && \
    echo "echo 'Shared folder: /shared'" >> ~/.bashrc && \
    echo "echo ''" >> ~/.bashrc

CMD ["/bin/bash"]
