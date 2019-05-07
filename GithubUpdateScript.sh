# The following script should automate the github update process
# If the .sh file is not executable, do the following:
#
# sudo chmod 700 GithubUpdateScript.sh
#
# Enjoy!

printf "\n Start Github Update...\n"

git fetch upstream
git pull upstream master
git push
git status

printf "\n See what to add and do it on a new terminal!\n"
gnome-terminal
printf " Press any key to continue (ctrl-C to quit)"
read -n 1 -s -r -p "..."

git status
printf "\n Now do they all look good? Press any key to continue (ctrl-C to quit)"
read -n 1 -s -r -p "..."

git commit
git push

printf "\n All done. Great work!\n"