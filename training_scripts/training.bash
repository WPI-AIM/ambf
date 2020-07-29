#!/bin/bash
source /opt/ros/melodic/setup.bash
source ../build/devel/setup.bash

python3 ../ambf_ros_modules/ambf_comm/scripts/RL/stable_baseline_her.py
