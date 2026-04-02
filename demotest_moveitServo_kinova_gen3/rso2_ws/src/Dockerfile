FROM moveit/moveit2:main-jazzy-tutorial-source

RUN apt update && apt install -y \
      ros-jazzy-moveit-servo \
      && rm -rf /var/lib/apt/lists/*

ENV INTERFACE_WS=/root/ros2_ws    
WORKDIR ${INTERFACE_WS}
SHELL ["/bin/bash", "-c"]
RUN echo "source /opt/ros/jazzy/setup.bash" >> /root/.bashrc
