#!/usr/bin/env python

import roslib; roslib.load_manifest('playground')
import rospy
from geometry_msgs.msg import Twist, Pose2D

import numpy as np

import Strategy

_robot = { 'xhat': 0, 'yhat': 0, 'thetahat': 0 }
_ball  = { 'xhat': 0, 'yhat': 0 }


def _handle_estimated_robot_position(msg):
    _robot['xhat'] = msg.x
    _robot['yhat'] = msg.y
    _robot['thetahat'] = msg.theta

def _handle_estimated_ball_position(msg):
    _ball['xhat'] = msg.x
    _ball['yhat'] = msg.y

def main():
    rospy.init_node('ai', anonymous=False)

    rospy.Subscriber('estimated_robot_position', Pose2D, _handle_estimated_robot_position)
    rospy.Subscriber('estimated_ball_position', Pose2D, _handle_estimated_ball_position)
    pub = rospy.Publisher('desired_position', Pose2D, queue_size=10)

    rate = rospy.Rate(100) #100 Hz
    while not rospy.is_shutdown():

        (x_c, y_c, theta_c) = Strategy.choose_strategy(_robot, _ball)
        msg = Pose2D()
        msg.x = x_c
        msg.y = y_c
        msg.theta = theta_c
        pub.publish(msg)

        rate.sleep()

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    main()