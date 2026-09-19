#!/bin/bash
set -e
source "/opt/ros/${ROS_DISTRO}/setup.bash"
source /opt/andino/setup.bash
exec "$@"
