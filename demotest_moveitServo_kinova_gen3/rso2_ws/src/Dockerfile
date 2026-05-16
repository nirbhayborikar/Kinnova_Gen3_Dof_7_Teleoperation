# FROM moveit/moveit2:main-jazzy-tutorial-source

# RUN apt update && apt install -y \
#       ros-jazzy-moveit-servo \
#       && rm -rf /var/lib/apt/lists/*



# ENV INTERFACE_WS=/root/ros2_ws    
# WORKDIR ${INTERFACE_WS}
# SHELL ["/bin/bash", "-c"]
# RUN echo "source /opt/ros/jazzy/setup.bash" >> /root/.bashrc
# # 

FROM moveit/moveit2:main-jazzy-tutorial-source

RUN apt-get update && apt-get install -y python3-venv

RUN python3 -m venv /opt/venv
RUN /opt/venv/bin/pip install catkin_pkg


ENV PATH="/opt/venv/bin:$PATH"

# ---- Python Dependencies ----
RUN pip3 install --no-cache-dir --break-system-packages --ignore-installed \
    "numpy<2" \
    matplotlib==3.9.0 \
    mediapipe==0.10.14 \
    opencv-python-headless \
    transforms3d
# pip install

RUN apt-get update && apt-get install -y \
    ros-jazzy-moveit-servo \
    ros-jazzy-moveit-msgs \
    nlohmann-json3-dev \
    ros-jazzy-rmw-zenoh-cpp \
    ros-jazzy-rmw-cyclonedds-cpp \
    ros-jazzy-rmw-fastrtps-cpp \


    # Camera & GUI
    libv4l-dev \
    v4l-utils \
    mesa-utils \
    libgl1 \
    libgl1-mesa-dri \

    && rm -rf /var/lib/apt/lists/*


ENV INTERFACE_WS=/root/ws_moveit
SHELL ["/bin/bash", "-c"]
WORKDIR ${INTERFACE_WS}

RUN source /opt/ros/jazzy/setup.bash && \
    colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release



COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh
RUN echo 'source /opt/ros/jazzy/setup.bash' >> /root/.bashrc && \
    echo 'if [ -f "/ws_moveit/install/setup.bash" ]; then source /ws_moveit/install/setup.bash; fi' >> /root/.bashrc

WORKDIR ${INTERFACE_WS}
ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]