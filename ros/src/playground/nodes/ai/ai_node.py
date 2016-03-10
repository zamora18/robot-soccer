#!/usr/bin/env python

import roslib; roslib.load_manifest('playground')
import rospy
from geometry_msgs.msg import Pose2D
from std_msgs.msg import Bool
from playground.msg import BallState

import numpy as np

import Strategy

_robot = { 'xhat': 0, 'yhat': 0, 'thetahat': 0 }
_ball  = { 'xhat': 0, 'yhat': 0, 'xhat_future': 0, 'yhat_future': 0 }
_opponent = { 'xhat': 0, 'yhat': 0, 'thetahat': 0 }

_was_goal = False


def _handle_estimated_robot_position(msg):
    _robot['xhat'] = msg.x
    _robot['yhat'] = msg.y
    _robot['thetahat'] = msg.theta

def _handle_opponent_position(msg):
    _opponent['xhat'] = msg.x
    _opponent['yhat'] = msg.y
    _opponent['thetahat'] = msg.theta

def _handle_ball_state(msg):
    _ball['xhat'] = msg.xhat
    _ball['yhat'] = msg.yhat
    _ball['xhat_future'] = msg.xhat_future
    _ball['yhat_future'] = msg.yhat_future

def _handle_goal(msg):
    global _was_goal
    _was_goal = msg.data

def main():
    rospy.init_node('ai', anonymous=False)

    rospy.Subscriber('estimated_robot_position', Pose2D, _handle_estimated_robot_position)
    rospy.Subscriber('vision_opponent_position', Pose2D, _handle_opponent_position)
    rospy.Subscriber('ball_state', BallState, _handle_ball_state)
    rospy.Subscriber('goal', Bool, _handle_goal)
    pub = rospy.Publisher('desired_position', Pose2D, queue_size=10)

    global _was_goal

    rate = rospy.Rate(100) #100 Hz
    while not rospy.is_shutdown():

        (x_c, y_c, theta_c) = Strategy.choose_strategy(_robot, _opponent, _ball, _was_goal)
        msg = Pose2D()
        msg.x = x_c
        msg.y = y_c
        msg.theta = theta_c
        pub.publish(msg)

        _was_goal = False

        rate.sleep()

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    main()
