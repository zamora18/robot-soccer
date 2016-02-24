#!/usr/bin/env python

import time

import roslib; roslib.load_manifest('playground')
import rospy
from geometry_msgs.msg import Twist, Pose2D
from std_srvs.srv import Trigger, TriggerResponse

import numpy as np

from filters import LowpassFilter

ball_lpf = LowpassFilter(0.7, 0.02)

_estimator_period = 1.0/100

_xhat = 0
_yhat = 0
_thetahat = 0

_last_time = time.time()
_camera_flag = 0

_estimator_on = True

def _handle_vision_ball_position(msg):
    global _xhat, _yhat, _thetahat, _last_time, _camera_flag
    _xhat = msg.x
    _yhat = msg.y
    _thetahat = msg.theta

    _last_time = time.time()
    _camera_flag = 1

def main():
    rospy.init_node('estimator', anonymous=False)

    # Sub/Pub
    rospy.Subscriber('vision_ball_position', Pose2D, _handle_vision_ball_position)
    pub = rospy.Publisher('estimated_ball_position', Pose2D, queue_size=10)

    global _estimator_on, _camera_flag

    rate = rospy.Rate(int(1/_estimator_period))
    while not rospy.is_shutdown():

        if _estimator_on:
            Ts = (time.time() - _last_time)
            (xhat, yhat) = ball_lpf.filter(_xhat, _yhat, Ts, _camera_flag)
            _camera_flag = 0

            msg = Pose2D()
            msg.x = xhat
            msg.y = yhat
            msg.theta = 0
            pub.publish(msg)

        rate.sleep()

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    main()
