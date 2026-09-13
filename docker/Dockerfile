FROM ubuntu:22.04
ENV DEBIAN_FRONTEND=noninteractive

# Install basic tools and locales
RUN apt-get update && apt-get install -y --no-install-recommends \
    ca-certificates curl gnupg2 lsb-release wget sudo locales tzdata \
    git build-essential python3-pip \
    && rm -rf /var/lib/apt/lists/*

# Setup locale
RUN locale-gen en_US.UTF-8
ENV LANG en_US.UTF-8

# Install ROS2 Humble
RUN apt-get update && apt-get install -y --no-install-recommends software-properties-common \
 && curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add - \
 && sh -c 'echo "deb http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros2.list' \
 && apt-get update \
 && apt-get install -y ros-humble-desktop python3-rosdep python3-colcon-common-extensions \
 && rm -rf /var/lib/apt/lists/*

# Install Ignition Gazebo (gz-harmonic) and ros_gz
RUN apt-get update && apt-get install -y --no-install-recommends \
    wget gnupg2 lsb-release \
    && wget -qO - https://packages.osrfoundation.org/gazebo.key | apt-key add - \
    && sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" > /etc/apt/sources.list.d/gazebo-stable.list' \
    && apt-get update \
    && apt-get install -y gz-harmonic ros-humble-ros-gz ros-humble-ros-gz-bridge \
    && rm -rf /var/lib/apt/lists/*

# Install Micro XRCE-DDS Agent prerequisites
RUN apt-get update && apt-get install -y --no-install-recommends libasio-dev libtinyxml2-dev libboost-dev \
    && rm -rf /var/lib/apt/lists/*

# Create workspace directories
RUN mkdir -p /workspace/src /px4
WORKDIR /workspace

# Copy repo into workspace (mounted by docker-compose at runtime if preferred)
# Expect the host to mount the project at /workspace

# Environment
ENV ROS_DOMAIN_ID=0
ENV DISPLAY=${DISPLAY}

# Default entrypoint: keep container alive and let docker-compose run the command
CMD ["bash"]
