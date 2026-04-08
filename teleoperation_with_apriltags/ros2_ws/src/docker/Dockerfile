# =============================================================================
# kinova-follower / Dockerfile
#
# Completely standalone image — zero kortex/moveit/gstreamer bloat.
# Only installs what the follower Python node actually uses:
#   rclpy, tf2_ros, tf2_geometry_msgs, geometry_msgs, numpy, rmw_zenoh_cpp
#
# Connects to the kortex stack purely via ROS2 network
# (same ROS_DOMAIN_ID + network_mode: host).
# No kortex packages, MoveIt, apriltag_ros, or GStreamer needed here.
# =============================================================================

ARG BASE_IMAGE=ros:jazzy-ros-base
FROM $BASE_IMAGE


# Copy all custom interfaces from the submodules
# The idea is that each container that runs on the robot has all interfaces
# available. To avoid duplicated packages, we copy them to a seperate workspace.
ENV INTERFACE_WS=/root/ros2_ws

######


# COPY [path_to_interface_package] ${INTERFACE_WS}/src/[path_to_interface_package]
#COPY src/ /${INTERFACE_WS}/src/




# # ── Only the packages the follower node imports ───────────────────────────────
# RUN apt-get update && apt-get install -y --no-install-recommends \
#     # ROS2 Python client library
#     ros-jazzy-rclpy \
#     # TF2 — for lookup_transform (reads /tf and /tf_static)
#     ros-jazzy-tf2-ros \
#     ros-jazzy-tf2-geometry-msgs \
#     # Message types used by the node
#     ros-jazzy-geometry-msgs \
#     ros-jazzy-std-msgs \
#     ros-jazzy-cv-bridge \
#     # RMW — must match the kortex stack
#     ros-jazzy-rmw-zenoh-cpp \
#     # Build tools (needed to install the Python package with colcon)
#     python3-colcon-common-extensions \
#     python3-numpy \
#     python3-opencv \



# # apriltag dependency
#     ros-jazzy-image-transport \
#     ros-jazzy-image-geometry \
#     ros-jazzy-camera-info-manager


#     # Useful for debugging inside the container
#     ros-jazzy-ros2cli \
#     ros-jazzy-ros2topic \
#     ros-jazzy-ros2param \
    
#     && rm -rf /var/lib/apt/lists/*


RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-jazzy-rclpy \
    ros-jazzy-tf2-ros \
    ros-jazzy-tf2-geometry-msgs \
    ros-jazzy-geometry-msgs \
    ros-jazzy-std-msgs \
    ros-jazzy-sensor-msgs \
    ros-jazzy-cv-bridge \
    ros-jazzy-image-transport \
    ros-jazzy-image-geometry \
    ros-jazzy-camera-info-manager \
    ros-jazzy-rmw-zenoh-cpp \
    ros-jazzy-apriltag \
    ros-jazzy-apriltag-ros \
    ros-jazzy-ros2cli \
    ros-jazzy-ros2topic \
    ros-jazzy-ros2param \
    python3-colcon-common-extensions \
    python3-numpy \
    python3-opencv \
    && rm -rf /var/lib/apt/lists/*
 

WORKDIR ${INTERFACE_WS}



# RUN source /opt/ros/jazzy/setup.bash && \ 
#     colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release \
#     --packages-select apriltag_msgs apriltags apriltag_ros


#RUN source ~/ros2_ws/install/setup.bash



SHELL ["/bin/bash", "-c"]

RUN echo "source /opt/ros/jazzy/setup.bash" >> /root/.bashrc