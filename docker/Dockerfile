# syntax = docker/dockerfile:experimental
FROM osrf/ros:melodic-desktop-full

############################## SYSTEM PARAMETERS ##############################
## Base arguments
ARG USER=initial
ARG GROUP=initial
ARG UID=1000
ARG GID=${UID}
ARG SHELL=/bin/bash

## NVIDIA GraphicsCard parameter
ENV NVIDIA_VISIBLE_DEVICES all
ENV NVIDIA_DRIVER_CAPABILITIES all
# ENV NVIDIA_DRIVER_CAPABILITIES graphics, utility, compute

## Setup users and groups
RUN groupadd --gid ${GID} ${GROUP} \
    && useradd --gid ${GID} --uid ${UID} -ms ${SHELL} ${USER} \
    && mkdir -p /etc/sudoers.d \
    && echo "${USER}:x:${UID}:${UID}:${USER},,,:$HOME:${shell}" >> /etc/passwd \
    && echo "${USER}:x:${UID}:" >> /etc/group \
    && echo "${USER} ALL=(ALL) NOPASSWD: ALL" > "/etc/sudoers.d/${USER}" \
    && chmod 0440 "/etc/sudoers.d/${USER}"

## Replace apt urls
# Taiwan
# RUN sed -i 's@archive.ubuntu.com@tw.archive.ubuntu.com@g' /etc/apt/sources.list
# TKU
RUN sed -i 's@archive.ubuntu.com@ftp.tku.edu.tw/@g' /etc/apt/sources.list

############################### INSTALL & SETUP ###############################
## Install packages
RUN apt-get update && apt-get install -y --no-install-recommends \
    sudo htop git wget curl \
    # Shell
    byobu zsh \
    terminator \
    dbus-x11 libglvnd0 libgl1 libglx0 libegl1 libxext6 libx11-6 \
    # Editing tools
    nano vim gedit\
    gnome-terminal libcanberra-gtk-module libcanberra-gtk3-module \
    # Work tools
    python3-pip python3-dev python3-setuptools \
    python-pip python-catkin-tools python-setuptools \
    # for using add-apt-repository
    software-properties-common \
    # Install lidar driver(urg_node)
    ros-melodic-urg-node \
    # Install PL-ICP pkg(csm)
    ros-melodic-csm \
    # Install map server (show the trajectory in rviz)
    ros-melodic-map-server \
    ros-melodic-hector-slam \
    # install udev and rosserial for tb3 driver
    udev \
    ros-melodic-rosserial-python \
    # install ORBSLAM2 dependency
    gcc g++ \
    libglew-dev \
    cmake \
    libboost-dev libboost-thread-dev libboost-filesystem-dev \
    libpython2.7-dev \
    libegl1-mesa-dev libwayland-dev libxkbcommon-dev wayland-protocols \
    libeigen3-dev libblas-dev liblapack-dev \
    ros-melodic-pcl-conversions ros-melodic-pcl-ros ros-melodic-perception-pcl ros-melodic-perception \
    # install cuda for ORBSLAM
    nvidia-cuda-toolkit \
    libopencv-dev \
    # add rule file for lidar permission
    && mkdir -p /etc/udev/rules.d \
    apt-get clean \
    && rm -rf /var/lib/apt/lists

## Install realsense SDK
RUN apt-get update \
    && apt-key adv --keyserver keys.gnupg.net --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE || apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE \
    && add-apt-repository -y "deb https://librealsense.intel.com/Debian/apt-repo xenial main" \
    && apt-get update -qq \
    && sudo apt-get install librealsense2-dkms --allow-unauthenticated -y \
    && sudo apt-get install librealsense2-utils --allow-unauthenticated -y \
    && sudo apt-get install librealsense2-dbg --allow-unauthenticated -y \
    && sudo apt-get install librealsense2-dev --allow-unauthenticated -y \ 
    && sudo apt-get install ros-melodic-rgbd-launch \
    && apt-get clean \
    && rm -rf /var/lib/apt/lists
    
RUN apt-get update \
    # && apt-get install -y ros-groovy-openslam-gmapping \
    # install gmapping dependency
    # && apt-get install -y ros-melodic-slam-gmapping \
    && apt-get clean \
    && rm -rf /var/lib/apt/lists

WORKDIR /home/${USER}/.tmp

## setup custom configuration
COPY config .

## ROS Arguments
# Is the computer master or slave in ROS
ARG ROS_TYPE=MASTER

# ARG ROS_MASTER_IP=163.13.164.148
ARG ROS_MASTER_IP=localhost
ARG ROS_SLAVE_IP=localhost

## Favorite shell when using byobu
ARG BYOBU_SHELL=zsh

## Set User configuration
## Set turtlebot model
RUN bash ./run_setup_script.sh \
    && bash ./script/env_setup.sh \
    && bash ./script/pip_setup.sh \
    && rm -rf /home/${USER}/.tmp
    # && bash ./script/ros_setup.sh \

## give permission to the drivers of tb3
# RUN udevadm control --reload-rules \
#     && udevadm trigger

## Copy entrypoint
COPY ./entrypoint.sh /entrypoint.sh
RUN chmod 777 /entrypoint.sh

WORKDIR /home/${USER}

## Switch user to ${USER}
USER ${USER}

RUN sudo mkdir work

## Make SSH available
EXPOSE 22

## Switch to user's HOME folder
WORKDIR /home/${USER}/work

# ENTRYPOINT ["byobu"]
# ENTRYPOINT ["/entrypoint.sh", "byobu"]
# ENTRYPOINT ["/entrypoint.sh", "bash"]
ENTRYPOINT ["/entrypoint.sh", "terminator"]
# ENTRYPOINT ["/entrypoint.sh", "zsh"]
