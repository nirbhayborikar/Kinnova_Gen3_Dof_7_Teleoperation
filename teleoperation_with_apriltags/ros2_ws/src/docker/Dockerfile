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




# ── Only the packages the follower node imports ───────────────────────────────
RUN apt-get update && apt-get install -y --no-install-recommends \
    # ROS2 Python client library
    ros-jazzy-rclpy \
    # TF2 — for lookup_transform (reads /tf and /tf_static)
    ros-jazzy-tf2-ros \
    ros-jazzy-tf2-geometry-msgs \
    # Message types used by the node
    ros-jazzy-geometry-msgs \
    ros-jazzy-std-msgs \
    # RMW — must match the kortex stack
    ros-jazzy-rmw-zenoh-cpp \
    # Build tools (needed to install the Python package with colcon)
    python3-colcon-common-extensions \
    python3-numpy \

    # OpenCv
    ros-jazzy-cv-bridge python3-opencv -y \
    # Useful for debugging inside the container
    ros-jazzy-ros2cli \
    ros-jazzy-ros2topic \
    ros-jazzy-ros2param \
    && rm -rf /var/lib/apt/lists/*

WORKDIR ${INTERFACE_WS}

SHELL ["/bin/bash", "-c"]

RUN echo "source /opt/ros/jazzy/setup.bash" >> /root/.bashrc