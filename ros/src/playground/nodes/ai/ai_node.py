#!/usr/bin/env python

import roslib; roslib.load_manifest('playground')
import rospy
from geometry_msgs.msg import Pose2D
from std_msgs.msg import Bool

from playground.msg import BallState, RobotState
from playground.srv import SetBool, SetBoolResponse

import numpy as np

import Strategy
from Robot import Robot

_me = None
_ally = None
_opp1 = None
_opp2 = None

_ball = {'xhat': 0}

_ai_enabled = False
_was_goal = False


def _handle_robot_state(msg, which_robot):
    # Update the given robot's current and future positions
    if which_robot == 'me':
        _me.update_state(msg)
    elif which_robot == 'ally':
        _ally.update_state(msg)
    elif which_robot == 'opp1':
        _opp1.update_state(msg)
    elif which_robot == 'opp2':
        _opp2.update_state(msg)

def _handle_ball_state(msg):
    _ball['xhat'] = msg.xhat
    _ball['yhat'] = msg.yhat
    _ball['xhat_future'] = msg.xhat_future
    _ball['yhat_future'] = msg.yhat_future

def _handle_goal(msg):
    global _was_goal
    _was_goal = msg.data

def _set_ai(req):
    global _ai_enabled
    _ai_enabled = req.data
    return SetBoolResponse(_ai_enabled, "")


def _create_robots():
    """Create Robots
    This function uses the namespace of the node to know whether
    this code is running on ally1 (Nugget) or ally2 (Fry).
    Since `am_i_ally1` and `am_i_ally2` are mutually exclusive,
    after creating the `_me` robot object, `_ally` has the
    opposite designation than `_me`.

    For example, if `am_i_ally1` is True, then `am_i_ally2` must
    be False. Thus, `_me` will be ally1 and `_ally` will be ally2.
    """
    global  _me, _ally, _opp1, _opp2
    am_i_ally1 = rospy.get_namespace() == 'ally1'
    am_i_ally2 = rospy.get_namespace() == 'ally2'

    _me = Robot(ally1=am_i_ally1, ally2=am_i_ally2)
    _ally = Robot(ally1=(not am_i_ally1), ally2=(not am_i_ally2))
    _opp1 = Robot()
    _opp2 = Robot()

def main():
    rospy.init_node('ai', anonymous=False)

    # Create robot objects that store that current robot's state
    _create_robots()

    # Subscribe to Robot States
    rospy.Subscriber('my_state', RobotState, lambda msg: _handle_robot_state(msg, 'me'))
    rospy.Subscriber('ally_state', RobotState, lambda msg: _handle_robot_state(msg, 'ally'))
    rospy.Subscriber('opponent1_state', RobotState, lambda msg: _handle_robot_state(msg, 'opp1'))
    rospy.Subscriber('opponent2_state', RobotState, lambda msg: _handle_robot_state(msg, 'opp2'))

    rospy.Subscriber('ball_state', BallState, _handle_ball_state)
    rospy.Subscriber('goal', Bool, _handle_goal)
    pub = rospy.Publisher('desired_position', Pose2D, queue_size=10)

    rospy.Service('set_ai', SetBool, _set_ai)

    global _was_goal

    rate = rospy.Rate(100) #100 Hz
    while not rospy.is_shutdown():

        #(x_c, y_c, theta_c) = Strategy.choose_strategy(_robot, _opponent, _ball, _was_goal)
        if _ai_enabled:
            pass
            # msg = Pose2D()
            # msg.x = x_c
            # msg.y = y_c
            # msg.theta = theta_c
            # pub.publish(msg)

        _was_goal = False

        rate.sleep()

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    main()