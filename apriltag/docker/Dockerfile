ARG BASE_IMAGE=fbe-dockerreg.rwu.de/prj-iki-ros2/orga/ros2-docker-base:jazzy
FROM $BASE_IMAGE AS base

ENV RMW_IMPLEMENTATION=rmw_zenoh_cpp

RUN apt update \ 
&& apt install -y vim \
    ros-${ROS_DISTRO}-rmw-zenoh-cpp \
   ros-$ROS_DISTRO-image-transport \
   ros-$ROS_DISTRO-image-transport-plugins \
   ros-$ROS_DISTRO-cv-bridge \
&& rm -rf /var/lib/apt/lists/*

ENV WS=/root/ros2_ws/src/prj-iki-ros2/perception

COPY submodules/apriltag ${WS}/apriltag
COPY submodules/apriltag_msgs ${WS}/apriltag_msgs
COPY apriltag_ros ${WS}/apriltag_ros

FROM base AS build
#build packages
WORKDIR /root/ros2_ws/
RUN source /opt/ros/$ROS_DISTRO/setup.bash && colcon build --symlink-install --packages-select apriltag apriltag_msgs apriltag_ros

COPY docker/ros2_entrypoint.sh /ros_entrypoint.sh
ENTRYPOINT ["/ros_entrypoint.sh"]
