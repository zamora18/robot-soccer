#!/usr/bin/env python

import time

import roslib; roslib.load_manifest('playground')
import rospy
from geometry_msgs.msg import Twist, Pose2D
from playground.msg import RobotState

import numpy as np

from estimators import RobotEstimator

_robot = RobotEstimator()

_estimator_rate = 100

# (x, y, theta) measured from camera
_measured = (None, None, None)

# (vx, vy, w) from motion node
_velocities = (0, 0, 0)

# _last_time = time.time()

_estimator_on = True
_predictor_on = True

_predict_forward_seconds = (5/30.0)

def _handle_vision_position(msg):
    global _measured#, _last_time
    _measured = (msg.x, msg.y, msg.theta)

    # Timestamp this msg so the bot updater knows
    # how much time elapsed since last measurement
    # _last_time = time.time()

def _handle_vel_cmds(msg):
    global _velocities
    _velocities = (msg.linear.x, msg.linear.y, msg.angular.z)

def main():
    rospy.init_node('robot_estimator', anonymous=False)

    # Sub/Pub
    rospy.Subscriber('vision_position', Pose2D, _handle_vision_position)
    rospy.Subscriber('vel_cmds', Twist, _handle_vel_cmds)
    # Use remap in roslaunch file to create separate channels per robot
    pub = rospy.Publisher('robot_state', RobotState, queue_size=10)

    global _measured

    rate = rospy.Rate(_estimator_rate)
    while not rospy.is_shutdown():

        xhat = yhat = thetahat = xhat_future = yhat_future = thetahat_future = 0

        if rospy.get_param('robot_estimator_on', 'true'):
            # Ts = (time.time() - _last_time)
            (xhat, yhat, thetahat) = _robot.update(_measured, _velocities)
        else:
            (xhat, yhat, thetahat) = _measured

        # if _predictor_on:
        #     (xhat_future, yhat_future) = _robot.predict(_predict_forward_seconds)

        # Grab the estimated velocities of the ball
        # (vx, vy) = _ball.get_velocities()
        (vx, vy, w) = (0, 0, 0)

        # Construct ball_state message, BallState
        msg = RobotState()
        msg.vision_x = _measured[0] if _measured[0] is not None else 0
        msg.vision_y = _measured[1] if _measured[1] is not None else 0
        msg.vision_theta = _measured[2] if _measured[2] is not None else 0
        msg.xhat = xhat
        msg.yhat = yhat
        msg.thetahat = thetahat
        msg.vx = vx
        msg.vy = vy
        msg.w = w
        msg.xhat_future = xhat_future
        msg.yhat_future = yhat_future
        msg.thetahat_future = thetahat_future
        msg.predict_forward_seconds = _predict_forward_seconds if _predictor_on else 0
        msg.correction = _measured[0] is not None and _measured[1] is not None and _measured[2] is not None
        pub.publish(msg)

        # Set measured to None so the robot updater 
        # knows to predict instead of correcting
        if rospy.get_param('robot_estimator_on', 'true'):
            _measured = (None, None, None)


        rate.sleep()

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    main()
