#!/usr/bin/env python

import roslib; roslib.load_manifest('playground')
import rospy
from geometry_msgs.msg import Pose2D
from playground.msg import BallState

import numpy as np

import Strategy

_robot = { 'xhat': 0, 'yhat': 0, 'thetahat': 0 }
_ball  = { 'xhat': 0, 'yhat': 0, 'xhat_future': 0, 'yhat_future': 0 }


def _handle_estimated_robot_position(msg):
    _robot['xhat'] = msg.x
    _robot['yhat'] = msg.y
    _robot['thetahat'] = msg.theta

def _handle_ball_state(msg):
    _ball['xhat'] = msg.xhat
    _ball['yhat'] = msg.yhat
    _ball['xhat_future'] = msg.xhat_future
    _ball['yhat_future'] = msg.yhat_future

def main():
    rospy.init_node('ai', anonymous=False)

    rospy.Subscriber('estimated_robot_position', Pose2D, _handle_estimated_robot_position)
    rospy.Subscriber('ball_state', BallState, _handle_ball_state)
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
