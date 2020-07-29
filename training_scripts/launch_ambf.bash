#!/bin/bash

source /opt/ros/melodic/setup.bash
source ../build/devel/setup.bash

cd ../bin-debug/lin-x86_64/
./ambf_simulator -l 5,22 -p 2000 -g false -t 1 -s 2 --ns /test1/
