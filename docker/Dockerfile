FROM arm64v8/ros:humble-perception-jammy


ARG USERNAME=user
ARG USERID=1000
ARG USERGID=${USERID}
ARG SHELL=bash


ENV USERNAME=${USERNAME}
ENV SHELL=/bin/${SHELL}
ENV TERM=xterm-256color
ENV LC_ALL=en_US.UTF-8
ENV LANG=en_US.UTF-8
ENV ROS_DISTRO=humble


RUN apt-get update && \
    apt-get install -y \
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
    locales \
    build-essential \
    cmake \
    locales \
    python3-colcon-common-extensions

RUN apt-get update && \
    apt-get install -y \
    libsdl-image1.2-dev \
    libsuitesparse-dev \
    libfreeimage-dev \
    libyaml-cpp-dev \
    ros-humble-tf2-sensor-msgs \
    ros-humble-libg2o \
    ros-humble-teleop-twist-keyboard \
    ros-humble-navigation2 \
    ros-humble-nav2-bringup \
    ros-humble-geographic-msgs \
    ros-humble-geometry-msgs \
    ros-humble-test-msgs \
    ros-humble-bondcpp \
    ros-humble-foxglove-bridge \
    ros-humble-rmw-cyclonedds-cpp && \
    rm -rf /var/lib/apt/lists/*


RUN DEBIAN_FRONTEND=noninteractive apt install -y --no-install-recommends tzdata


RUN apt-get update && apt-get install -y locales && \
    echo "en_US.UTF-8 UTF-8" > /etc/locale.gen && \
    locale-gen en_US.UTF-8 && \
    update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
ENV LANG=en_US.UTF-8
ENV LC_ALL=en_US.UTF-8

RUN groupadd --gid ${USERGID} ${USERNAME} \
    && useradd --uid ${USERID} --gid ${USERGID} -m ${USERNAME} -s /bin/bash \
    && echo "${USERNAME} ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers

USER ${USERNAME}
WORKDIR /home/${USERNAME}
COPY ./settings/.bashrc /home/${USERNAME}/.bashrc 
COPY ./settings/tf2_sensor_msgs.hpp /opt/ros/humble/include/tf2_sensor_msgs/tf2_sensor_msgs.hpp


RUN sudo apt autoremove -y && \
    sudo apt clean -y 


SHELL ["/bin/bash", "-c"]
CMD ["bash"]