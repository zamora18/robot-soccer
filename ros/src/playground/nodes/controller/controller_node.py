#!/usr/bin/env python

import roslib; roslib.load_manifest('playground')
import rospy
from geometry_msgs.msg import Twist, Pose2D

import numpy as np

import Controller

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


def main():
    rospy.init_node('controller', anonymous=False)

    rospy.Subscriber('estimated_position', Pose2D, _handle_estimated_position)
    rospy.Subscriber('desired_position', Pose2D, _handle_desired_position)
    pub = rospy.Publisher('vel_cmds', Twist, queue_size=10)

    rate = rospy.Rate(int(1/_ctrl_period))
    while not rospy.is_shutdown():

        (vx, vy, w) = Controller.update(_ctrl_period, _xhat, _yhat, _thetahat)

        msg = Twist()
        msg.linear.x = vx
        msg.linear.y = vy
        msg.angular.z = w

        pub.publish(msg)

        rate.sleep()

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    main()