#!/usr/bin/env python

import roslib; roslib.load_manifest('playground')
import rospy
from geometry_msgs.msg import Twist, Pose2D

import numpy as np

from Getch import _Getch

_ctrl_period = 1.0/100

_vx = 0.5
_vy = 0.5
_w = np.pi

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
    rospy.init_node('controller', anonymous=False)

    rospy.Subscriber('estimated_robot_position', Pose2D, _handle_estimated_bot_position)
    
    global _vel_pub, _pos_cmd_pub
    _vel_pub = rospy.Publisher('vel_cmds', Twist, queue_size=10)
    _pos_cmd_pub = rospy.Publisher('desired_position', Pose2D, queue_size=10)

    # Setup shutdown hook
    rospy.on_shutdown(_shutdown_hook)

    while(1):
        action = _get_action()
        if action == 'UP':
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

        # elif action == 'GO_TO_POINT':
        #     _toggle_timers(False)
        #     motion.stop()
        #     time.sleep(1)

        #     _ask_for_point()

        #     time.sleep(1)
        #     _ctrl_on = True
        #     _odom_on = True
        #     _toggle_timers(True)

        # elif action == 'TOGGLE_CNTRL':
        #     _ctrl_on = not _ctrl_on
        #     print("Controller: {}".format(_ctrl_on))

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

        # elif action == 'DIE':
        #     pass

        # else:
        #     pass


if __name__ == '__main__':
    main()
