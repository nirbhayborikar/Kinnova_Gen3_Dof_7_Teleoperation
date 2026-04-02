ARG BASE_IMAGE=fbe-dockerreg.rwu.de/prj-iki-ros2/orga/ros2-docker-base:humble
FROM $BASE_IMAGE AS base

ENV REPO=/root/ros2_ws/src/prj-iki-ros2/knowledgebase/iki_object_models

#Install dependencies
RUN apt update && \
    apt install -y \
    ros-$ROS_DISTRO-gazebo-ros-pkgs \
    ros-$ROS_DISTRO-ros-gz && \
    rm -rf /var/lib/apt/lists/*

COPY iki_object_models $REPO/iki_object_models
COPY external $REPO/external

# Build
FROM base AS build 
WORKDIR /root/ros2_ws
RUN source /opt/ros/$ROS_DISTRO/setup.bash && \
    colcon build --symlink-install && \
    echo "source /root/ros2_ws/install/setup.bash" >> /root/.bashrc

