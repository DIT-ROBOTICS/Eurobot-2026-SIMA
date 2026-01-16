# Dockerfile for Nav2 on RPI5(arm64) / Desktop
FROM osrf/ros:humble-desktop

ARG USERNAME=user
ARG USERID=1000
ARG USERGID=${USERID}
ARG SHELL=bash
ARG DEBIAN_FRONTEND=noninteractive

ENV USERNAME=${USERNAME}
ENV SHELL=/bin/${SHELL}
ENV TERM=xterm-256color
ENV LC_ALL=en_US.UTF-8
ENV LANG=en_US.UTF-8
ENV ROS_DISTRO=humble

# Keep downloaded packages for caching purposes
RUN rm -f /etc/apt/apt.conf.d/docker-clean; echo 'Binary::apt::APT::Keep-Downloaded-Packages "true";' > /etc/apt/apt.conf.d/keep-cache

# Basic Tools
RUN --mount=type=cache,target=/var/cache/apt,sharing=private \
    apt-get update && apt-get install -y \
    sudo \
    vim \
    curl \
    wget \
    tmux \
    htop \
    git \
    git-extras \
    gnupg2 \
    net-tools \
    build-essential \
    cmake \
    locales \
    mesa-utils \
    python3-pip \
    python3-setuptools && \
    rm -rf /var/lib/apt/lists/*

# Install colcon and other Python tools
RUN --mount=type=cache,target=/var/cache/apt,sharing=private \
    apt-get update && \
    apt-get install -y \
    python3-colcon-common-extensions \
    python3-colcon-mixin \
    python3-rosdep \
    python3-vcstool && \
    rm -rf /var/lib/apt/lists/*

# Install Nav2 dependencies, Gazebo, and other useful ROS2 packages
RUN --mount=type=cache,target=/var/cache/apt,sharing=private \
    apt-get update && \
    apt-get install -y \
    libsdl-image1.2-dev \
    libsuitesparse-dev \
    libfreeimage-dev \
    libyaml-cpp-dev \
    ros-$ROS_DISTRO-gazebo-ros-pkgs \
    ros-$ROS_DISTRO-tf2-sensor-msgs \
    ros-$ROS_DISTRO-libg2o \
    ros-$ROS_DISTRO-teleop-twist-keyboard \
    ros-$ROS_DISTRO-navigation2 \
    ros-$ROS_DISTRO-nav2-bringup \
    ros-$ROS_DISTRO-geographic-msgs \
    ros-$ROS_DISTRO-geometry-msgs \
    ros-$ROS_DISTRO-test-msgs \
    ros-$ROS_DISTRO-bondcpp \
    ros-$ROS_DISTRO-foxglove-bridge \
    ros-$ROS_DISTRO-ament-lint-auto \
    ros-$ROS_DISTRO-rmw-cyclonedds-cpp \
    ros-$ROS_DISTRO-robot-localization && \
    rm -rf /var/lib/apt/lists/*

# --- Fix Header File (Must be done as ROOT) ---
# Copy the patch file to the system location
COPY ./settings/tf2_sensor_msgs.hpp /opt/ros/$ROS_DISTRO/include/tf2_sensor_msgs/tf2_sensor_msgs.hpp

# Create a non-root user
RUN groupadd --gid ${USERGID} ${USERNAME} \
    && useradd --uid ${USERID} --gid ${USERGID} -m ${USERNAME} -s /bin/bash \
    && echo "${USERNAME} ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers

# Timezone setup
RUN apt update && DEBIAN_FRONTEND=noninteractive apt install -y --no-install-recommends tzdata
RUN TZ=Asia/Taipei && \
    ln -snf /usr/share/zoneinfo/$TZ /etc/localtime && \
    echo $TZ > /etc/timezone && \
    dpkg-reconfigure -f noninteractive tzdata 

# --- User Setup ---
USER ${USERNAME}
WORKDIR /home/${USERNAME}

# Copy bashrc
COPY ./settings/.bashrc /home/${USERNAME}/.bashrc 

RUN sudo apt autoremove -y && \
    sudo apt clean -y

SHELL ["/bin/bash", "-c"]
CMD ["bash"]