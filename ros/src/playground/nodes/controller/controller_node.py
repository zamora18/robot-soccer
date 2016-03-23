#!/usr/bin/env python

import roslib; roslib.load_manifest('playground')
import rospy
from geometry_msgs.msg import Twist, Pose2D
from std_srvs.srv import Trigger, TriggerResponse

from playground.msg import PIDInfo, RobotState

import numpy as np

import Controller

_ctrl_period = 1.0/100

_xhat = 0
_yhat = 0
_thetahat = 0

_ctrl_on = True
_initializing = True

def _handle_robot_state(msg):
    global _xhat, _yhat, _thetahat, _initializing
    _xhat = msg.xhat
    _yhat = msg.yhat
    _thetahat = msg.thetahat

    if _initializing:
        _initializing = False

        x = rospy.get_param('x_init')
        y = rospy.get_param('y_init')
        theta = rospy.get_param('theta_init')

        Controller.set_commanded_position(x, y, theta)

def _handle_desired_position(msg):
    global _ctrl_on
    Controller.set_commanded_position(msg.x, msg.y, msg.theta)
    _ctrl_on = True

def _toggle(req):
    global _ctrl_on
    _ctrl_on = not _ctrl_on

    srv = Trigger()
    srv.success = True
    srv.message = "Controller is {}".format(_ctrl_on)
    return TriggerResponse(True, "Controller is {}".format(_ctrl_on))

def main():
    rospy.init_node('controller', anonymous=False)

    # Sub/Pub
    rospy.Subscriber('robot_state', RobotState, _handle_robot_state)
    rospy.Subscriber('desired_position', Pose2D, _handle_desired_position)
    pub = rospy.Publisher('vel_cmds', Twist, queue_size=10)
    pub_PIDInfo = rospy.Publisher('pidinfo', PIDInfo, queue_size=10)

    # Services
    rospy.Service('/controller/toggle', Trigger, _toggle)

    rate = rospy.Rate(int(1/_ctrl_period))
    while not rospy.is_shutdown():

        global _ctrl_on

        if _ctrl_on:
            (vx, vy, w) = Controller.update(_ctrl_period, _xhat, _yhat, _thetahat)

            # Publish Velocity Commands
            msg = Twist()
            msg.linear.x = vx
            msg.linear.y = vy
            msg.angular.z = w
            pub.publish(msg)

            # Publish PID Info
            msg = PIDInfo()
            msg.error.x = Controller.PID_x.error_d1
            msg.error.y = Controller.PID_y.error_d1
            msg.error.theta = Controller.PID_theta.error_d1
            set_point = Controller.get_commanded_position()
            msg.desired.x = set_point[0]
            msg.desired.y = set_point[1]
            msg.desired.theta = set_point[2]
            msg.actual.x = _xhat
            msg.actual.y = _yhat
            msg.actual.theta = _thetahat
            pub_PIDInfo.publish(msg)


        rate.sleep()

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    main()
