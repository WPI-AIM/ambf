# The following script should automate the compilation and execution of your code
# If the .sh file is not executable, do the following:
#
# sudo chmod 700 CompileAndRunScript.sh
#
# Enjoy!

printf "\n Start Compiling the AMBF Code...\n"


cd ./build
cmake ..
make
source ./devel/setup.bash
printf "\n AMBF Code Finished Building!\n"



printf "\n To run the code, make sure an external roscore is running.\n"
printf " To run the AMBF Controller, change the world settings in the \n"
printf " launch.yaml file to enable raven_world.yaml.\n"
printf " Press any key to continue (ctrl-C to quit)"
read -n 1 -s -r -p "..."



printf "\n Run the AMBF Simulator.  (On a new terminal)"
cd ../bin/lin-x86_64/
gnome-terminal -e "./ambf_simulator -l 3"



printf "\n Run the AMBF Controller. (Here!)\n"
./ambf_controller


# Git notes:
# 
# 1. Clone your fork:
# 
# git clone git@github.com:YOUR-USERNAME/YOUR-FORKED-REPO.git
# 
# 2. Add remote from original repository in your forked repository:
# 
# cd into/cloned/fork-repo
# git remote add upstream git://github.com/ORIGINAL-DEV-USERNAME/REPO-YOU-FORKED-FROM.git
# git fetch upstream
# 
# 3. Updating your fork from original repo to keep up with their changes:
# 
# git pull upstream master
# git push
#



# First Time Build notes:
#
# 1. fatal error: alsa/asoundlib.h: No such file or directory
#
# Do the following and remake.
# sudo apt-get install libasound2-dev