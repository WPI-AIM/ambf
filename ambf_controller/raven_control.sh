# The following script should automate the compilation and execution of your code
# If the .sh file is not executable, do the following:
#
# sudo chmod 700 raven_control.sh
#
# Enjoy!

printf "\n To run the code, make sure an external roscore is running.\n"
printf " To run the AMBF Controller, change the world settings in the \n"
printf " launch.yaml file to enable raven_world.yaml.\n"
printf " Press any key to continue (ctrl-C to quit)"
read -n 1 -s -r -p "..."



printf "\n Run the AMBF Simulator.  (On a new terminal)"
cd ../bin/lin-x86_64/
gnome-terminal -e "./ambf_simulator --launch_file ../../ambf_controller/raven2/launch.yaml -l0"



printf "\n Run the AMBF Controller. (Here!)\n"
./raven_controller
