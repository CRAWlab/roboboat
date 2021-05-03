#!/bin/bash
set -e

# setup ROS environment
source "/root/catkin_ws/devel/setup.bash"
exec "$@"