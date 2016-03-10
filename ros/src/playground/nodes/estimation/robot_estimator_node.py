#!/usr/bin/env python

import time

import roslib; roslib.load_manifest('playground')
import rospy
from geometry_msgs.msg import Pose2D
from playground.msg import RobotState

import numpy as np

from estimators import RobotEstimator

_robot = RobotEstimator()

_estimator_rate = 100

# (x, y, theta) measured from camera
_measured = (0, 0, 0)

_last_time = time.time()

_estimator_on = True
_predictor_on = True

_predict_forward_seconds = 1

def _handle_vision_ball_position(msg):
    global _xhat, _yhat, _last_time
    _xhat = msg.x
    _yhat = msg.y

    # Timestamp this msg so the ball updater knows
    # how much time elapsed since last measurement
    _last_time = time.time()

def main():
    rospy.init_node('robot_estimator', anonymous=False)

    # Sub/Pub
    rospy.Subscriber('vision_robot_position', Pose2D, _handle_vision_ball_position)
    # Use remap in roslaunch file to create separate channels per robot
    pub = rospy.Publisher('robot_state', RobotState, queue_size=10)

    rate = rospy.Rate(_estimator_rate)
    while not rospy.is_shutdown():

        if _estimator_on:
            Ts = (time.time() - _last_time)
            (xhat, yhat) = _robot.update(Ts, _xhat, _yhat)

            msg = Pose2D()
            msg.x = xhat
            msg.y = yhat
            msg.theta = 0
            estimate_pub.publish(msg)

        if _predictor_on:
            Ts = (time.time() - _last_time)
            (xhat, yhat) = _robot.predict(_predict_forward_seconds)

            msg = Pose2D()
            msg.x = xhat
            msg.y = yhat
            msg.theta = 0
            predict_pub.publish(msg)


        rate.sleep()

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    main()