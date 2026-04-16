#!/bin/bash
set -e

source /opt/ros/${ROS_DISTRO}/setup.bash

if [ -f "/ros2_ws/install/setup.bash" ]; then
  source /ros2_ws/install/setup.bash
fi

# Zenoh router config for rmw_zenoh_cpp
export ZENOH_RUNTIME_CONFIG=/zenoh_config.json5

exec "$@"