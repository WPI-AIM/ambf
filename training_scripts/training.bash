#!/bin/bash
. /opt/ros/melodic/setup.bash
. $AMBF_WS/build/devel/setup.bash

python3 $AMBF_WS/ambf_ros_modules/ambf_comm/scripts/RL/stable_baseline_her.py