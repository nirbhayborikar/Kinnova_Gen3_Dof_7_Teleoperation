# =============================================================================
# Kinova Gen3 7-DOF Teleoperation - ROS2 Jazzy
# =============================================================================
FROM ros:jazzy-ros-base

ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=jazzy
SHELL ["/bin/bash", "-c"]

# ---- System Dependencies ----
RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential cmake git wget python3-pip \
    python3-colcon-common-extensions python3-rosdep python3-vcstool \
    ros-${ROS_DISTRO}-rviz2 \
    ros-${ROS_DISTRO}-tf2-tools \
    ros-${ROS_DISTRO}-tf-transformations \
    ros-${ROS_DISTRO}-generate-parameter-library \
    ros-jazzy-usb-cam \
    ros-jazzy-rmw-zenoh-cpp \
    ros-jazzy-rmw-cyclonedds-cpp \
    # Camera & GUI
    libv4l-dev \
    v4l-utils \
    mesa-utils \
    libgl1 \
    libgl1-mesa-dri \
    && rm -rf /var/lib/apt/lists/*

# ---- Python Dependencies ----
RUN pip3 install --no-cache-dir --break-system-packages \
    "numpy<2" \
    matplotlib==3.9.0 \
    mediapipe==0.10.14 \
    opencv-python-headless \
    scipy \
    transforms3d


# ---- Create Workspace ----
ENV WS=/root/ros2_ws
RUN mkdir -p ${WS}/src
WORKDIR ${WS}/src

# ---- Clone ros2_kortex (Jazzy = main branch) ----


# ---- Install ROS dependencies ----
WORKDIR ${WS}
RUN source /opt/ros/${ROS_DISTRO}/setup.bash && \
    apt-get update && \
    rosdep update && \
    rosdep install --from-paths src --ignore-src -r -y && \
    rm -rf /var/lib/apt/lists/*

# ---- Build workspace ----
RUN source /opt/ros/${ROS_DISTRO}/setup.bash && \
    colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release

# ---- Entrypoint + auto-source ----
COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh
RUN echo 'source /opt/ros/${ROS_DISTRO}/setup.bash' >> /root/.bashrc && \
    echo 'if [ -f "/root/ros2_ws/install/setup.bash" ]; then source /root/ros2_ws/install/setup.bash; fi' >> /root/.bashrc

WORKDIR ${WS}
ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
