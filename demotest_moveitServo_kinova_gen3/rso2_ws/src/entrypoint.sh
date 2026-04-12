#!/bin/bash
set -e
source /opt/ros/jazzy/setup.bash
if [ -f "/root/ws_moveit/install/setup.bash " ]; then
    source /root/ws_moveit/install/setup.bash 
fi
exec "$@"
