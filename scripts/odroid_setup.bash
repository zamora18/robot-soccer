# Add this file to the bottom of your ~/.bashrc to include all of this goodness:
# i.e.,     `source /path/to/repo/scripts/odroid_setup.bash`

# Which robot am I?
# (Make sure to put `export ROBOT=<robot-name>` in your ~/.bashrc)
# That way you can access the ROBOT env var

# Where is your ROS Core?
ROS_MASTER=ronald

# What is the package name of all your robot code?
ROBOT_PKG='playground'

# =============================================================================

# For DIR, see http://stackoverflow.com/a/246128
DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
PATH_TO_REPO="$DIR/.."

# My IP Address
# See: http://stackoverflow.com/a/13322549
MY_IP=`ifconfig | sed -En 's/127.0.0.1//;s/.*inet (addr:)?(([0-9]*\.){3}[0-9]*).*/\2/p'`

# include parse_yaml function
source parse_yaml.sh

# read Robot YAML file to get configuration settings (whether or not to use rcv3 or rcv5)
eval $(parse_yaml "$PATH_TO_REPO/ros/src/$ROBOT_PKG/param/$ROBOT.yaml" "robot_config_")

# Tell the world about using rcv3 or rcv5
export USE_RCV3=$robot_config_use_rcv3

# Aliases
alias ll='ls -lh --color'
alias l='ls -alh --color'
alias ..='cd ..'
alias ...='cd ../..'
alias teleop='cd "$PATH_TO_REPO"/tests/motors/ && ipython teleop.py'
alias gpio='cd /sys/class/gpio/ && cd gpio200'

# ROS
source /opt/ros/indigo/setup.bash
export ROS_MASTER_URI="http://${ROS_MASTER}:11311"
export ROS_IP="$MY_IP"
source $PATH_TO_REPO/ros/devel/setup.bash

# Killbot
# alias killbot='$PATH_TO_REPO/scripts/killbot.py'
alias battery='$PATH_TO_REPO/scripts/battery.py'
alias kick='$PATH_TO_REPO/scripts/kick.py'

function killbot() {
    killall roslaunch
    sleep 2
    $PATH_TO_REPO/scripts/killbot.py

    # Let's just make sure...
    sleep 6
    $PATH_TO_REPO/scripts/killbot.py
}

# Setup GPIO (kicker == gpio200)
#echo 200 > /sys/class/gpio/export
#echo out > /sys/class/gpio/gpio200/direction
#echo 0 > /sys/class/gpio/gpio200/value
