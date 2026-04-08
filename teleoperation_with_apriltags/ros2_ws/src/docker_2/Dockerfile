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

FROM ros:jazzy-ros-base

ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=jazzy
ENV FOLLOWER_WS=/root/follower_ws

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
    # Useful for debugging inside the container
    ros-jazzy-ros2cli \
    ros-jazzy-ros2topic \
    ros-jazzy-ros2param \
    && rm -rf /var/lib/apt/lists/*

# ── Global RMW — must match what kortex stack uses ────────────────────────────
ENV RMW_IMPLEMENTATION=rmw_zenoh_cpp
# Nodes start even if zenoh router hasn't replied yet
ENV ZENOH_ROUTER_CHECK_ATTEMPTS=0

# ── Create workspace (src is volume-mounted at runtime — see compose) ─────────
RUN mkdir -p ${FOLLOWER_WS}/src

# ── Entrypoint: build then run ────────────────────────────────────────────────
COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

WORKDIR ${FOLLOWER_WS}
ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]