#!/bin/bash

# define signal handler and its variable
allowAbort=true;
trainingInterruptHandler()
{
    if $allowAbort; then
        exit 1;
    fi;
}

# register signal handler
trap trainingInterruptHandler SIGINT;

# disable the abortability of the script
allowAbort=false;

# turn on bash's job control
set -m

# Start the first process
./roscore.bash &

# Start the second process
./psmIK_service.bash &

# Start the third process
./training.bash &

# Start the fifth process
./tensorboard_launch.bash &

# Start the fourth process
./launch_ambf.bash

# now we bring the primary process back into the foreground
# and leave it there
fg %5

# and now make the script abortable again
allowAbort=true;
