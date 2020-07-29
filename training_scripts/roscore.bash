#!/bin/bash

export LD_LIBRARY_PATH=/usr/local/lib:$LD_LIBRARY_PATH

source /opt/ros/melodic/setup.bash
roscore #-p 12345
