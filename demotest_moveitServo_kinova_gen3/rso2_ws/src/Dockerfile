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

RUN apt-get update && apt-get install -y \
    ros-jazzy-moveit-servo \
    ros-jazzy-moveit-msgs \
    nlohmann-json3-dev \
    ros-jazzy-rmw-zenoh-cpp \
    ros-jazzy-rmw-cyclonedds-cpp \
    && rm -rf /var/lib/apt/lists/*

ENV INTERFACE_WS=/root/ros2_ws
WORKDIR ${INTERFACE_WS}

SHELL ["/bin/bash", "-c"]

RUN echo "source /opt/ros/jazzy/setup.bash" >> /root/.bashrc