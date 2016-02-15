#!/usr/bin/env python

import roslib; roslib.load_manifest('playground')
import rospy
from geometry_msgs.msg import Twist, Pose2D

import numpy as np

from Getch import _Getch

_ctrl_period = 1.0/100

_xhat = 0
_yhat = 0
_thetahat = 0

def _handle_estimated_position(msg):
    # rospy.loginfo(rospy.get_caller_id() + "I heard (%s,%s,%s)", data.linear.x,data.linear.y,data.angular.z)
    global _xhat, _yhat, _thetahat
    _xhat = msg.x
    _yhat = msg.y
    _thetahat = msg.theta

def _handle_desired_position(msg):
    # rospy.loginfo(rospy.get_caller_id() + "I heard (%s,%s,%s)", data.linear.x,data.linear.y,data.angular.z)
    print("Set Point: ({}, {}, {})".format(msg.x, msg.y, msg.theta))
    Controller.set_commanded_position(msg.x, msg.y, msg.theta)


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

    rospy.Subscriber('estimated_position', Pose2D, _handle_estimated_position)
    rospy.Subscriber('desired_position', Pose2D, _handle_desired_position)
    vel_pub = rospy.Publisher('vel_cmds', Twist, queue_size=10)
    

    while(1):
        action = _get_action()
        if action == 'UP':
            msg = Twist()
            msg.linear.x = 0
            msg.linear.y = 0.5
            msg.angular.z = 0
            vel_pub.publish(msg)

        elif action == 'DOWN':
            pass

        elif action == 'RIGHT':
            pass

        elif action == 'LEFT':
            pass

        elif action == 'SPIN_CW':
            pass

        elif action == 'SPIN_CCW':
            pass

        # elif action == 'SET_HOME':
        #     Odometry.init()

        # elif action == 'GO_HOME':
        #     _motion_timer.stop()
        #     motion.stop()
        #     time.sleep(1)

        #     _go_home()

        #     time.sleep(1)
        #     _set_speed = True
        #     _velocities = (0, 0, 0)
        #     _motion_timer.start()

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
