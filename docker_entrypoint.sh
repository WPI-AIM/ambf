#!/bin/bash

# setup ros environment
echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> ~/.bashrc
echo "source /root/ambf/build/devel/setup.bash" >> ~/.bashrc

source ~/.bashrc