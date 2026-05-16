# FROM fbe-dockerreg.rwu.de/prj-iki-ros2/orga/ros2-docker-base:jazzy
# RUN apt update \
#     && apt upgrade -y \
#     && apt install -y --no-install-recommends \
#     libgflags-dev \
#     libgoogle-glog-dev \
#     libusb-1.0-0-dev \
#     libeigen3-dev \
#     nlohmann-json3-dev \
#     ros-$ROS_DISTRO-image-geometry \
#     ros-$ROS_DISTRO-camera-info-manager \
#     ros-$ROS_DISTRO-image-transport \
#     ros-$ROS_DISTRO-image-publisher \
#     ros-jazzy-rmw-zenoh-cpp \
#     ros-jazzy-rmw-cyclonedds-cpp \
#     && rm -rf /var/lib/apt/lists*

# COPY submodules/libuvc /root/libuvc
# WORKDIR /root/libuvc
# #RUN mkdir build && cd build && cmake .. && make -j4 && make install && ldconfig

# ENV REPO=/root/ros2_ws/src/prj-iki-ros2/sensors/astra
# COPY astra_camera ${REPO}/astra_camera
# COPY astra_camera_msgs ${REPO}/astra_camera_msgs
# COPY astra_description ${REPO}/astra_description

# WORKDIR /root/ros2_ws
# RUN source /opt/ros/$ROS_DISTRO/setup.bash && \
#     colcon build --event-handlers console_direct+ --cmake-args -DCMAKE_BUILD_TYPE=Release


FROM fbe-dockerreg.rwu.de/prj-iki-ros2/orga/ros2-docker-base:jazzy

RUN apt update \
    && apt upgrade -y \
    && apt install -y --no-install-recommends \
    libgflags-dev \
    libgoogle-glog-dev \
    libusb-1.0-0-dev \
    libeigen3-dev \
    nlohmann-json3-dev \
    ros-$ROS_DISTRO-image-geometry \
    ros-$ROS_DISTRO-camera-info-manager \
    ros-$ROS_DISTRO-image-transport \
    ros-$ROS_DISTRO-image-publisher \
    ros-jazzy-rmw-zenoh-cpp \
    ros-jazzy-rmw-cyclonedds-cpp \
    && rm -rf /var/lib/apt/lists/*

# Build libuvc from submodule and install to /usr so pkg-config finds it
COPY submodules/libuvc /root/libuvc
WORKDIR /root/libuvc
RUN test -f CMakeLists.txt || \
    (echo "ERROR: libuvc submodule is empty. Run 'git submodule update --init --recursive' on the host." && exit 1)
RUN mkdir -p build && cd build && \
    cmake .. -DCMAKE_INSTALL_PREFIX=/usr \
             -DCMAKE_BUILD_TYPE=Release \
             -DBUILD_UVC_SHARED=ON && \
    make -j"$(nproc)" && \
    make install && \
    ldconfig

ENV REPO=/root/ros2_ws/src/prj-iki-ros2/sensors/astra
COPY astra_camera     ${REPO}/astra_camera
COPY astra_camera_msgs ${REPO}/astra_camera_msgs
COPY astra_description ${REPO}/astra_description

WORKDIR /root/ros2_ws
RUN . /opt/ros/$ROS_DISTRO/setup.sh && \
    colcon build --event-handlers console_direct+ --cmake-args -DCMAKE_BUILD_TYPE=Release