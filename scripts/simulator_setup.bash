# Add this file to the bottom of your ~/.bashrc to include all of this goodness:
# i.e.,     `source /path/to/repo/scripts/simulator.bash`

# What is the package name of all your robot code?
ROBOT_PKG='playground'

# For DIR, see http://stackoverflow.com/a/246128
DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
PATH_TO_REPO="$DIR/.."

# =============================================================================

# My IP Address
# See: http://stackoverflow.com/a/13322549
MY_IP=`ifconfig | sed -En 's/127.0.0.1//;s/.*inet (addr:)?(([0-9]*\.){3}[0-9]*).*/\2/p'`

# Aliases
alias ll='ls -lh --color'
alias l='ls -alh --color'
alias ..='cd ..'
alias ...='cd ../..'

# ROS
source /opt/ros/indigo/setup.bash
export ROS_MASTER_URI=http://localhost:11311
export ROS_IP="$MY_IP"
source $PATH_TO_REPO/ros/devel/setup.bash

function killsim() {
    # must be called in the same terminal you started
    killall roslaunch

    # Kill all jobs
    # kill $(jobs -p)

    # # kill gazebo
    # kill `ps aux | grep gazebo | grep ros | awk '{ print $2; }'`

    # # kill al nodes
    # kill `ps aux | grep python | grep ros | awk '{ print $2;}'`
}

# Simulate the "space bar" being pressed on the vision code
function vision_spacebar() {
    rostopic pub /game_state playground/GameState -- "{'play': true, 'two_v_two': false}";
}

# Simulation Scripts (The user can put in bg with &)
function simulator_1v1() {
    # Set up localhost as my master
    master_sim;

    # To launch the simulation environment in the background, with ally1
    # ready to go (delete home2 and away2 robots)
    roslaunch "$ROBOT_PKG" simulator.launch &
    sleep 6 # Otherwise there is a race condition

    roslaunch "$ROBOT_PKG" ally1.launch &
    export SIM_ROBOTS=1
}

function simulator_2v2() {
    # Set up localhost as my master
    master_sim;

    # To launch the simulation environment in the background,
    # with ally1 and ally2 ready to go.
    roslaunch "$ROBOT_PKG" simulator.launch &
    sleep 6 # Otherwise there is a race condition
    roslaunch "$ROBOT_PKG" ally1.launch &
    sleep 2
    roslaunch "$ROBOT_PKG" ally2.launch &
    export SIM_ROBOTS=2
}

function sim_go() {
    if [[ $SIM_ROBOTS -eq 1 ]]; then
        # Delete unneccesary models
        rosservice call /gazebo/delete_model home2
        rosservice call /gazebo/delete_model away2
        roslaunch "$ROBOT_PKG" ai_ally1.launch &
        export SIM_PID_1=$!
    elif [[ $SIM_ROBOTS -eq 2 ]]; then
        roslaunch "$ROBOT_PKG" ai_ally1.launch &
        export SIM_PID_1=$!
        roslaunch "$ROBOT_PKG" ai_ally2.launch &
        export SIM_PID_2=$!
    else
        echo
        echo "ERROR!"
        echo "You must start the simulator first using:"
        echo "    For 1v1: simulator_1v1"
        echo "    For 2v2: simulator_2v2"
        echo
        echo "... noob."
        echo
    fi

    # Update for sim_stop
    export LAST_SIM_ROBOTS=${SIM_ROBOTS}

    # Remove env var for next run
    unset SIM_ROBOTS

    vision_spacebar;
}

function sim_stop() {
    if [[ $LAST_SIM_ROBOTS -eq 1 ]]; then
        kill "$SIM_PID_1"
        unset SIM_PID_1
    elif [[ $SIM_ROBOTS -eq 2 ]]; then
        kill "$SIM_PID_1"
        kill "$SIM_PID_2"
        unset SIM_PID_1
        unset SIM_PID_2
    else
        echo
        echo "ERROR!"
        echo "You must start the simulator AI first using:"
        echo "    sim_go"
        echo
        echo "... noob."
        echo
    fi

    unset LAST_SIM_ROBOTS
}

function master_ronald() {
    export ROS_MASTER_URI=http://ronald:11311
}

function master_sim() {
    export ROS_MASTER_URI=http://localhost:11311
}

function command_center() {
    master_ronald;
    rosrun "$ROBOT_PKG" gui.py
}

function sim_command_center() {
    master_sim;
    rosrun "$ROBOT_PKG" gui.py
}