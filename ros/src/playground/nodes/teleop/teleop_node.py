#!/usr/bin/env python

import sys

import roslib; roslib.load_manifest('playground')
import rospy
from geometry_msgs.msg import Twist, Pose2D
from std_srvs.srv import Trigger

import numpy as np

from Getch import _Getch

_ctrl_period = 1.0/100

_vx = 0.5
_vy = 0.5
_w = 180

_xhat = 0
_yhat = 0
_thetahat = 0

_vel_pub = None
_pos_cmd_pub = None

def _handle_estimated_bot_position(msg):
    global _xhat, _yhat, _thetahat
    _xhat = msg.x
    _yhat = msg.y
    _thetahat = msg.theta

def _toggle_controller():
    rospy.wait_for_service('/controller/toggle')
    try:
        toggle = rospy.ServiceProxy('/controller/toggle', Trigger)
        return toggle()
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

def _shutdown_hook():
    _set_velocity(0,0,0)

def _set_velocity(vx,vy,w):
    msg = Twist()
    msg.linear.x = vx
    msg.linear.y = vy
    msg.angular.z = w
    _vel_pub.publish(msg)

def _set_desired_position(x,y,theta):
    msg = Pose2D()
    msg.x = x
    msg.y = y
    msg.theta = theta
    _pos_cmd_pub.publish(msg)

def _ask_for_point():
    """Ask for point
    Asks user for a point to go to. User enters three floats, separated
    by spaces, with no other characters; x, y, theta respectively
    """
    usr = raw_input("Input point, (syntax: \"x y theta\"): ")

    usr_list = usr.split()

    if len(usr_list) != 3:
        print("\n\rEnter the point correctly, dummy.\r\n")
        return False

    try:
        x = float(usr_list[0])
        y = float(usr_list[1])
        theta = float(usr_list[2])
    except:
        print("\n\rThere was a problem making your input a float.\r\n")
        return False

    print("Going to: ({}, {}, {})\r".format(x,y,theta))
    return (x, y, theta)

def _get_action():
    getch = _Getch()
    k = getch()
    if k == 'w':
        return 'UP'
    elif k == 's':
        return 'DOWN'
    elif k == 'd':
        return 'RIGHT'
    elif k == 'a':
        return 'LEFT'
    elif k == 'x':
        return 'SPIN_CW'
    elif k == 'z':
        return 'SPIN_CCW'
    elif k == 'h':
        return 'SET_HOME'
    elif k == 'u':
        return 'TOGGLE_SMOOTH'
    elif k == 'o':
        return 'TOGGLE_ODOM'
    elif k == 'b':
        return 'BREAKPOINT'
    elif k == 'B':
        return 'BATTERY'
    elif k == 'H':
        return 'GO_HOME'
    elif k == 'G':
        return 'GO_TO_POINT'
    elif k == 'c':
        return 'TOGGLE_CNTRL'
    elif k == 'C':
        return 'CALIBRATION_MODE'
    elif k == ' ':
        return 'DIE'


def main():
    rospy.init_node('teleop', anonymous=False)

    rospy.Subscriber('estimated_robot_position', Pose2D, _handle_estimated_bot_position)
    
    global _vel_pub, _pos_cmd_pub
    _vel_pub = rospy.Publisher('vel_cmds', Twist, queue_size=10)
    _pos_cmd_pub = rospy.Publisher('desired_position', Pose2D, queue_size=10)

    # Setup shutdown hook
    rospy.on_shutdown(_shutdown_hook)

    print 'Starting...'


    while not rospy.is_shutdown():
        action = _get_action()
        if action == 'UP':
            print 'hi'
            _set_velocity(0, _vy, 0)

        elif action == 'DOWN':
            _set_velocity(0, -_vy, 0)

        elif action == 'RIGHT':
            _set_velocity(_vx, 0, 0)

        elif action == 'LEFT':
            _set_velocity(-_vx, 0, 0)

        elif action == 'SPIN_CW':
            _set_velocity(0, 0, _w)

        elif action == 'SPIN_CCW':
            _set_velocity(0, 0, _w)

        elif action == 'GO_HOME':
            _set_desired_position(0, 0, 0)

        elif action == 'GO_TO_POINT':
            (x, y, theta) = _ask_for_point()
            _set_desired_position(x, y, theta)
            
        elif action == 'TOGGLE_CNTRL':
            r = _toggle_controller()
            print("Controller: {}".format(r.message))

        # elif action == 'TOGGLE_SMOOTH':
        #     _smooth = not _smooth
        #     print("Smooth: {}".format(_smooth))

        # elif action == 'TOGGLE_ODOM':
        #     _odom_on = not _odom_on
        #     print("Odom: {}".format(_odom_on))

        # elif action == 'BATTERY':
        #     print("\n\r*** Battery: {} ***\n\r".format(get_battery()))

        # elif action == 'BREAKPOINT':
        #     _toggle_timers(False)
        #     import ipdb; ipdb.set_trace()
        #     _toggle_timers(True)

        # elif action == 'CALIBRATION_MODE':
        #     _toggle_timers(False)

        #     _deal_with_calibration()

        #     time.sleep(1)
        #     _toggle_timers(True)

        elif action == 'DIE':
            sys.exit(0)

        else:
            _set_velocity(0, 0, 0)


if __name__ == '__main__':
    main()
