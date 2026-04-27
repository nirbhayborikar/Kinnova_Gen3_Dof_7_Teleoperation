#!/bin/bash
set -e

source /opt/ros/${ROS_DISTRO}/setup.bash

if [ -f "/root/ros2_ws/install/setup.bash" ]; then
  source /root/ros2_ws/install/setup.bash
fi

# Zenoh client session config for rmw_zenoh_cpp
export ZENOH_SESSION_CONFIG_URI=/zenoh_config.json5

exec "$@"