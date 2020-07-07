#!/bin/bash

# turn on bash's job control
set -m

# Start the first process
./roscore.sh &
status=$?
if [ $status -ne 0 ]; then
  echo "Failed to start roscore: $status"
  exit $status
fi

# Start the second process
./launch_ambf.sh &
status=$?
if [ $status -ne 0 ]; then
  echo "Failed to launch ambf: $status"
  exit $status
fi

# Start the third process
./training.sh
status=$?
if [ $status -ne 0 ]; then
  echo "Failed to start training: $status"
  exit $status
fi

# now we bring the primary process back into the foreground
# and leave it there
fg %1
fg %2

# Naive check runs checks once a minute to see if either of the processes exited.
# This illustrates part of the heavy lifting you need to do if you want to run
# more than one service in a container. The container exits with an error
# if it detects that either of the processes has exited.
# Otherwise it loops forever, waking up every 60 seconds

while sleep 60; do
  ps aux |grep ./roscore.sh |grep -q -v grep
  PROCESS_1_STATUS=$?
  ps aux |grep ./launch_ambf.sh |grep -q -v grep
  PROCESS_2_STATUS=$?
  ps aux |grep ./training.sh |grep -q -v grep
  PROCESS_3_STATUS=$?
  # If the greps above find anything, they exit with 0 status
  # If they are not both 0, then something is wrong
  if [ $PROCESS_1_STATUS -ne 0 -o $PROCESS_2_STATUS -ne 0 -o $PROCESS_3_STATUS -ne 0 ]; then
    echo "One of the processes has already exited."
    exit 1
  fi
done