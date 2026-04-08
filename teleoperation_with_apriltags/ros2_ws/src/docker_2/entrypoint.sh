#!/usr/bin/env bash
# =============================================================================
# entrypoint.sh
# Builds the follower package (symlink-install = instant code changes)
# then hands off to CMD.
# =============================================================================
set -e

source /opt/ros/${ROS_DISTRO}/setup.bash

echo ""
echo "╔══════════════════════════════════════════════╗"
echo "║   Kinova Gen3 — AprilTag Follower            ║"
echo "║   RMW : $RMW_IMPLEMENTATION"
echo "║   Domain: $ROS_DOMAIN_ID"
echo "╚══════════════════════════════════════════════╝"
echo ""

# Build the package — symlink-install means edits to .py files take effect
# immediately on next run without rebuilding
#cd /root/follower_ws
# colcon build \
#     --symlink-install \
#     --cmake-args -DCMAKE_BUILD_TYPE=Release \
#     --packages-select apriltag_follower \
#     --event-handlers console_direct+ \
#     2>&1 | tail -5   # suppress verbose output, show last 5 lines

#source /follower_ws/install/setup.bash

exec "$@"