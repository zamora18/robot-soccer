# Add this file to the bottom of your ~/.bashrc to include all of this goodness:
# i.e.,     `source /path/to/repo/scripts/odroid_setup.bash`

PATH_TO_REPO=~/dev/lusk/repo

# Aliases
alias ll='ls -lh --color'
alias l='ls -alh --color'
alias ..='cd ..'
alias ...='cd ../..'

# ROS
source /opt/ros/indigo/setup.bash
export ROS_MASTER_URI=http://ronald:11311
source $PATH_TO_REPO/ros/devel/setup.bash

# Killbot
alias killbot='$PATH_TO_REPO/scripts/killbot.py'
alias battery='$PATH_TO_REPO/scripts/battery.py'
alias kick='$PATH_TO_REPO/scripts/kick.py'

# Setup GPIO (kicker == gpio200)
#echo 200 > /sys/class/gpio/export
#echo out > /sys/class/gpio/gpio200/direction
#echo 0 > /sys/class/gpio/gpio200/value
