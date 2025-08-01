#!/bin/bash
set -e

# # Source ROS 2 environment
# if [ -f "/opt/ros/$ROS_DISTRO/setup.bash" ]; then
#     source "/opt/ros/$ROS_DISTRO/setup.bash"
# else
#     echo "Could not find ROS 2 setup script at /opt/ros/$ROS_DISTRO/setup.bash"
#     exit 1
# fi

source "/opt/ros/$ROS_DISTRO/setup.bash"
# Source the workspace if it exists
if [ -f "/root/ros2_ws/install/setup.bash" ]; then
    source "/root/ros2_ws/install/setup.bash"
fi

# Execute the passed command
exec "$@"