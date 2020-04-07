#!/bin/bash
set -e

# setup ros environment
source /opt/ros/$ROS_DISTRO/setup.bash
source /root/ambf/build/devel/setup.bash

exec "$@"