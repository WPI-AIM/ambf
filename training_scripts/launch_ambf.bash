#!/bin/bash

. /opt/ros/melodic/setup.bash
. $AMBF_WS/build/devel/setup.bash

cd $AMBF_WS/bin-debug/lin-x86_64/
./ambf_simulator -l 5,22 -p 2000 -g false -t 1 -s 2 --ns /test1/