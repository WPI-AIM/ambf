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



printf "\n To run the AMBF Controller, follow the setups below:\n"
printf " (1) Run an external roscore on a seperate terminal.\n"
printf " (2) Change the world settings in the ambf/ambf_models/descriptions/launch.yaml file to enable the raven_world.yaml.\n"
printf " Press any key to continue (ctrl-C to quit)"
read -n 1 -s -r -p "..."




cd ../bin/lin-x86_64/
clear
printf "\n Get ready for the AMBF Simulator. \n"
printf "\n Current command:\n"
varname="./ambf_simulator -l 3 -t 1 -p 60 -a ../../ambf_models/descriptions/multi-bodies/sb/heart.yaml,../../ambf_models/descriptions/multi-bodies/environments/Table/table.yaml"
echo $varname
while true; do
	read -p " Continue? (Y/N or ctrl-C to quit): " yn
    case $yn in
        [Yy]* ) break;;
        [Nn]* ) printf "\n All setting options:\n"
				./ambf_simulator -h
				printf "\n Type your own command: "
				read -p "" varname
				break;;
            * ) printf "\n Invalid input.\n";;
    esac
done
clear
printf "\n Run the AMBF Simulator.  (On a new terminal)\n"
gnome-terminal -e "$varname" 


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