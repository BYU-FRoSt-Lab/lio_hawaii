#!/bin/bash
set -e

# # Source ROS 2 environment
# if [ -f "/opt/ros/$ROS_DISTRO/setup.bash" ]; then
#     source "/opt/ros/$ROS_DISTRO/setup.bash"
# else
#     echo "Could not find ROS 2 setup script at /opt/ros/$ROS_DISTRO/setup.bash"
#     exit 1
# fi

# # Source the workspace if it exists
# if [ -f "/root/ros_ws/install/setup.bash" ]; then
#     source "/root/ros_ws/install/setup.bash"
# fi
source "/opt/ros/$ROS_DISTRO/setup.bash"

# Execute the passed command
exec "$@"