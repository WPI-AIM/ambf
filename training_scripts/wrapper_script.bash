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

sleep 3

# Start the second process
./launch_ambf.bash &

# Start the third process
./training.bash

# now we bring the primary process back into the foreground
# and leave it there
fg %2

# and now make the script abortable again
allowAbort=true;