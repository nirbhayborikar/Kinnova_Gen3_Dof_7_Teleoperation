#!/usr/bin/env bash
# start_follower.sh
# 1. Waits for the arm driver to be ready
# 2. Switches from joint_trajectory_controller → twist_controller
# 3. Launches the AprilTag follower node
set -e

source /opt/ros/${ROS_DISTRO:-jazzy}/setup.bash
source /opt/ros2_ws/install/setup.bash

export RMW_IMPLEMENTATION=rmw_zenoh_cpp

echo "=== Follower: waiting for kortex_driver to publish joint_states ==="
# until ros2 topic list 2>/dev/null | grep -q joint_states; do
#     echo "  ... not ready yet, sleeping 3s"
#     sleep 3
# done
echo "=== kortex_driver is up ==="

echo "=== Switching to twist_controller ==="
# ros2 service call /controller_manager/switch_controller \
#     controller_manager_msgs/srv/SwitchController \
#     "{
#         activate_controllers:   [twist_controller],
#         deactivate_controllers: [joint_trajectory_controller],
#         strictness: 1,
#         activate_asap: true
#     }"

# echo "=== twist_controller active — launching follower node ==="
# ros2 run apriltag_follower follower_node