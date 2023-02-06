#!/bin/bash
set -e

# setup gazebo environment
source "/opt/ros/humble/setup.bash" && source "/opt/ros2_ws/install/setup.bash"
exec "$@"