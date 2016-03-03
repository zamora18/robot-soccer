#!/usr/bin/env python

import time

import roslib; roslib.load_manifest('playground')
import rospy
from geometry_msgs.msg import Pose2D
from playground.msg import BallState

import numpy as np

from estimators import BallEstimator

_ball = BallEstimator()

_estimator_rate = 100

# (x, y) measured from camera
_measured = (0, 0)

_last_time = time.time()

_estimator_on = True
_predictor_on = True

_predict_forward_seconds = 0.5

def _handle_vision_ball_position(msg):
    global _measured, _last_time
    _measured = (msg.x, msg.y)

    # Timestamp this msg so the ball updater knows
    # how much time elapsed since last measurement
    _last_time = time.time()

def main():
    rospy.init_node('ball_estimator', anonymous=False)

    # Sub/Pub
    rospy.Subscriber('vision_ball_position', Pose2D, _handle_vision_ball_position)
    pub = rospy.Publisher('ball_state', BallState, queue_size=10)

    rate = rospy.Rate(_estimator_rate)
    while not rospy.is_shutdown():

        xhat = yhat = xhat_future = yhat_future = 0

        if _estimator_on:
            Ts = (time.time() - _last_time)
            (xhat, yhat) = _ball.update(Ts, _measured[0], _measured[1])

        if _predictor_on:
            (xhat_future, yhat_future) = _ball.predict(_predict_forward_seconds)

        # Construct ball_state message, BallState
        msg = BallState()
        msg.xhat = xhat
        msg.yhat = yhat
        msg.xhat_future = xhat_future
        msg.yhat_future = yhat_future
        msg.predict_forward_seconds = _predict_forward_seconds if _predictor_on else 0
        pub.publish(msg)


        rate.sleep()

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    main()
