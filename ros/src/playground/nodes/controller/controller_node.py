#!/usr/bin/env python

import roslib; roslib.load_manifest('playground')
import rospy
from geometry_msgs.msg import Twist, Pose2D
from std_srvs.srv import Trigger, TriggerResponse

import numpy as np

import Controller

_ctrl_period = 1.0/100

_xhat = 0
_yhat = 0
_thetahat = 0

_ctrl_on = True
_initializing = True

def _handle_estimated_position(msg):
    global _xhat, _yhat, _thetahat, _initializing
    _xhat = msg.x
    _yhat = msg.y
    _thetahat = msg.theta

    if _initializing:
        _initializing = False
        Controller.set_commanded_position(msg.x, msg.y, msg.theta)

def _handle_desired_position(msg):
    # rospy.loginfo(rospy.get_caller_id() + "I heard (%s,%s,%s)", data.linear.x,data.linear.y,data.angular.z)
    print("Set Point: ({}, {}, {})".format(msg.x, msg.y, msg.theta))
    Controller.set_commanded_position(msg.x, msg.y, msg.theta)

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
    rospy.Subscriber('estimated_robot_position', Pose2D, _handle_estimated_position)
    rospy.Subscriber('desired_position', Pose2D, _handle_desired_position)
    pub = rospy.Publisher('vel_cmds', Twist, queue_size=10)
    pub_err = rospy.Publisher('error', Pose2D, queue_size=10)

    # Services
    rospy.Service('/controller/toggle', Trigger, _toggle)

    rate = rospy.Rate(int(1/_ctrl_period))
    while not rospy.is_shutdown():

        if _ctrl_on:
            (vx, vy, w) = Controller.update(_ctrl_period, _xhat, _yhat, _thetahat)

            msg = Twist()
            msg.linear.x = vx
            msg.linear.y = vy
            msg.angular.z = w

            pub.publish(msg)

            msg = Pose2D()
            msg.x = Controller.PID_x.error_d1
            msg.y = Controller.PID_y.error_d1
            msg.theta = Controller.PID_theta.error_d1
            pub_err.publish(msg)

        rate.sleep()

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    main()
