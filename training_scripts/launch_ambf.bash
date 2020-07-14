#!/bin/bash
. /opt/ros/melodic/setup.bash
. $AMBF_WS/build/devel/setup.bash
cd $AMBF_WS/bin/lin-x86_64/
./ambf_simulator -g false