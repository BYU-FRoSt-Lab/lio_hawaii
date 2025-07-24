#!/bin/bash
set -e

# setup ros2 environment
source "/opt/ros/$ROS_DISTRO/setup.bash" --

# Source the custom workspace setup file
# This is crucial for finding your custom packages like liorf
if [ -f "/root/ros2_ws/install/setup.bash" ]; then
  source "/root/ros2_ws/install/setup.bash" --
fi

exec "$@"