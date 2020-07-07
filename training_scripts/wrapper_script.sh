#!/bin/bash

# turn on bash's job control
set -m

# Start the first process
./roscore.sh &

# Start the second process
./launch_ambf.sh &

# Start the third process
./training.sh

# now we bring the primary process back into the foreground
# and leave it there
fg %1