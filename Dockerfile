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

# Run as a non-root user whose UID/GID match the host user. Everything written
# into ./ros2_ws and ./shared lands on the host, and while the container ran as
# root those files were root-owned: students could not edit or delete their own
# maps, bags and results without sudo. build.sh passes the host's ids.
#
# Passwordless sudo is kept so students can still apt-get install whatever
# libraries their project needs.
ARG USER_UID=1000
ARG USER_GID=1000
RUN if ! getent group "${USER_GID}" >/dev/null; then groupadd --gid "${USER_GID}" student; fi \
    && useradd --uid "${USER_UID}" --gid "${USER_GID}" --create-home --shell /bin/bash student \
    && echo "student ALL=(ALL) NOPASSWD:ALL" > /etc/sudoers.d/student \
    && chmod 0440 /etc/sudoers.d/student \
    && mkdir -p /ros2_ws/src /shared \
    && chown -R "${USER_UID}:${USER_GID}" /ros2_ws /shared

# Environment setup for TurtleBot3
ENV TURTLEBOT3_MODEL=waffle
ENV GAZEBO_MODEL_PATH=/opt/ros/humble/share/turtlebot3_gazebo/models
ENV ROS_DOMAIN_ID=0

USER student
WORKDIR /ros2_ws

# Configure bashrc for automatic sourcing and welcome message
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc && \
    echo "if [ -f /ros2_ws/install/setup.bash ]; then source /ros2_ws/install/setup.bash; fi" >> ~/.bashrc && \
    echo "echo ''" >> ~/.bashrc && \
    echo "echo '🤖 Nav2 Student Environment Ready!'" >> ~/.bashrc && \
    echo "echo 'Workspace: /ros2_ws'" >> ~/.bashrc && \
    echo "echo 'Shared folder: /shared'" >> ~/.bashrc && \
    echo "echo ''" >> ~/.bashrc

CMD ["/bin/bash"]
