# Pinned by digest: the tag is the only remaining copy of the ROSbot 2R
# simulation packages that match the lab robot's driver (see below).
FROM husarion/rosbot-gazebo:humble-0.14.0-20250319@sha256:909c1b195fcacb21f415f53b7494e2f65e98e212f70dbd8da6128b24f1cda694 AS rosbot_gazebo

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

# Husarion ROSbot 2R stack for Gazebo Fortress, as an underlay outside
# /ros2_ws so the bind-mounted student workspace cannot shadow it.
#
# Taken from Husarion's simulation image of the same build as the driver on the
# lab robot (husarion/rosbot:humble-0.14.0-20250319, rosbot_ros commit
# c860d02), so simulation and robot share topics, frames and controller limits.
# It is copied rather than built because that commit can no longer be built
# from source: its pinned ros_components_description repository was deleted
# upstream and the renamed repository no longer holds the pinned commit.
#
# Runtime dependencies are listed by hand: `rosdep check` over the copied
# package.xml files, with HUSARION_ROS_BUILD_TYPE=simulation, reports only
# these two as missing for simulation. ros_gz and Gazebo Fortress already come
# with humble-desktop-full. The other unmet keys (libgpiod, pyftdi, pyserial,
# usbutils) serve firmware flashing on the real robot. rosdep install cannot
# be pointed at the install tree directly: colcon's COLCON_IGNORE there makes
# it find no packages and report success.
COPY --from=rosbot_gazebo /ros2_ws/install /opt/rosbot_ws/install
COPY patches/rosbot_ros/0001-optional-camera.patch /tmp/0001-optional-camera.patch
RUN patch /opt/rosbot_ws/install/rosbot_description/share/rosbot_description/urdf/rosbot.urdf.xacro \
        < /tmp/0001-optional-camera.patch \
    && rm /tmp/0001-optional-camera.patch \
    && apt-get update && apt-get install -y \
        ros-humble-ign-ros2-control \
        ros-humble-robot-localization \
    && rm -rf /var/lib/apt/lists/*

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
    echo "source /opt/rosbot_ws/install/local_setup.bash" >> ~/.bashrc && \
    echo "if [ -f /ros2_ws/install/setup.bash ]; then source /ros2_ws/install/setup.bash; fi" >> ~/.bashrc && \
    echo "echo ''" >> ~/.bashrc && \
    echo "echo '🤖 Nav2 Student Environment Ready!'" >> ~/.bashrc && \
    echo "echo 'Workspace: /ros2_ws'" >> ~/.bashrc && \
    echo "echo 'Shared folder: /shared'" >> ~/.bashrc && \
    echo "echo ''" >> ~/.bashrc

CMD ["/bin/bash"]
