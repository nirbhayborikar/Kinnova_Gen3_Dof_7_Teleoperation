# ARG BASE_IMAGE=fbe-dockerreg.rwu.de/prj-iki-ros2/orga/ros2-docker-base:jazzy
# FROM $BASE_IMAGE AS base




FROM ros:jazzy AS base
SHELL ["/bin/bash", "-c"]





ENV ROS_WS=/root/ros2_ws
ENV KORTEX=${ROS_WS}/src/prj-iki-ros2/robots/kortex

# Real kortex stuff
RUN apt-get update && apt-get install -y \
    ros-${ROS_DISTRO}-ros2-control \
    ros-${ROS_DISTRO}-moveit \
    ros-${ROS_DISTRO}-moveit-ros-perception \
    ros-${ROS_DISTRO}-image-pipeline \
    ros-${ROS_DISTRO}-depth-image-proc \
    ros-${ROS_DISTRO}-rviz2 \
    ros-${ROS_DISTRO}-rqt \
    ros-jazzy-ros-gz \
    ros-jazzy-ros-gz-sim \
    ros-jazzy-ros-gz-bridge \
    ros-jazzy-gz-tools \
    ros-jazzy-gz-gui \
    ros-jazzy-gz-sim \
    gstreamer1.0-tools \
    gstreamer1.0-libav \
    libgstreamer1.0-dev \
    libgstreamer-plugins-base1.0-dev \
    libgstreamer-plugins-good1.0-dev \
    gstreamer1.0-plugins-good \
    gstreamer1.0-plugins-base \
    python3-vcstool \
    && rm -rf /var/lib/apt/lists/*

# Sim stuff
RUN apt-get update && apt-get install -y \
    ros-${ROS_DISTRO}-ros-gz \
    ros-${ROS_DISTRO}-gz-ros2-control \
    && rm -rf /var/lib/apt/lists/*

ENV GZ_SIM_SYSTEM_PLUGIN_PATH=/opt/ros/jazzy/lib/
ENV GZ_SIM_RESOURCE_PATH="${GZ_SIM_RESOURCE_PATH:+${GZ_SIM_RESOURCE_PATH}:}\
    /opt/ros/jazzy/lib/:\
    /root/ros2_ws/install/kortex_description/share/:\
    /root/ros2_ws/install/iki_kortex_description/share/:\
    /root/ros2_ws/install/robotiq_description/share/"

COPY submodules/iki_object_models/iki_object_models ${KORTEX}/iki_object_models

COPY submodules/ros2_kortex ${KORTEX}/ros2_kortex
COPY /ros2_kortex_vision ${KORTEX}/ros2_kortex_vision
COPY submodules/ros2_robotiq_gripper ${KORTEX}/ros2_robotiq_gripper
COPY submodules/serial ${KORTEX}/serial

COPY iki_kortex_moveit_config ${KORTEX}/iki_kortex_moveit_config
COPY iki_kortex_config ${KORTEX}/iki_kortex_config
COPY iki_kortex_description ${KORTEX}/iki_kortex_description
COPY iki_kortex_bringup ${KORTEX}/iki_kortex_bringup

WORKDIR ${ROS_WS}
RUN source /opt/ros/${ROS_DISTRO}/setup.bash && \
    apt update && rosdep install --from-paths src --ignore-src -r -y && \
    rm -rf /var/lib/apt/lists/*

COPY iki_kortex_moveit_config ${KORTEX}/iki_kortex_moveit_config

RUN source /opt/ros/${ROS_DISTRO}/setup.bash && \
    colcon build --symlink-install \
    --cmake-args -DCMAKE_BUILD_TYPE=Release
