#!/usr/bin/env python
import sys

import roslib; roslib.load_manifest('playground')
import rospy
from geometry_msgs.msg import Pose2D
from std_msgs.msg import Bool

from playground.msg import BallState, RobotState, GameState
from playground.srv import SetBool, SetBoolResponse

import numpy as np

import Strategy, Path
from GameObjects import Ball, Robot

_me = None
_ally = None
_opp1 = None
_opp2 = None

_ball = None

_ai_enabled = True
_pathplanning_enabled = False
_game_state = None
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
    # _ball['xhat'] = msg.xhat
    # _ball['yhat'] = msg.yhat
    # _ball['xhat_future'] = msg.xhat_future
    # _ball['yhat_future'] = msg.yhat_future

    _ball.update_state(msg)

def _handle_goal(msg):
    global _was_goal
    _was_goal = msg.data

def _handle_game_state(msg):
    global _game_state
    _game_state = msg

def _set_ai(req):
    global _ai_enabled
    _ai_enabled = req.data
    return SetBoolResponse(_ai_enabled, "")

def _set_path_planning(req):
    global _pathplanning_enabled
    _pathplanning_enabled = req.data
    return SetBoolResponse(_pathplanning_enabled, "")

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
    am_i_ally1 = 'ally1' in rospy.get_namespace()
    am_i_ally2 = 'ally2' in rospy.get_namespace()

    _me = Robot(ally1=am_i_ally1, ally2=am_i_ally2)
    _ally = Robot(ally1=(not am_i_ally1), ally2=(not am_i_ally2))
    _opp1 = Robot()
    _opp2 = Robot()

def main():
    rospy.init_node('ai', anonymous=False)

    # Create robot objects that store that current robot's state
    _create_robots()
 
    global _ball
    _ball = Ball()

    # Initialize our path planning world
    Path.initialize_world()

    # Subscribe to Robot States
    rospy.Subscriber('my_state', RobotState, lambda msg: _handle_robot_state(msg, 'me'))
    rospy.Subscriber('ally_state', RobotState, lambda msg: _handle_robot_state(msg, 'ally'))
    rospy.Subscriber('opponent1_state', RobotState, lambda msg: _handle_robot_state(msg, 'opp1'))
    rospy.Subscriber('opponent2_state', RobotState, lambda msg: _handle_robot_state(msg, 'opp2'))

    rospy.Subscriber('ball_state', BallState, _handle_ball_state)

    rospy.Subscriber('/game_state', GameState, _handle_game_state)
    rospy.Subscriber('goal', Bool, _handle_goal)
    pub = rospy.Publisher('desired_position', Pose2D, queue_size=10)

    rospy.Service('set_ai', SetBool, _set_ai)
    rospy.Service('set_path_planning', SetBool, _set_path_planning)

    global _was_goal

    rate = rospy.Rate(100) #100 Hz
    while not rospy.is_shutdown():

        # Figure out game state stuff
        if _game_state == None:
            one_v_one = False # Default to two_v_two
        else:
            one_v_one = not _game_state.two_v_two

        (x_c, y_c, theta_c) = Strategy.choose_strategy(_me, _ally, _opp1, _opp2, _ball, _was_goal, one_v_one=one_v_one)

        if _pathplanning_enabled:
            (x_c_safe, y_c_safe) = Path.plan(x_c, y_c, _me, _ally, _opp1, _opp2)
            theta_c_safe = theta_c
        else:
            (x_c_safe, y_c_safe, theta_c_safe) = (x_c, y_c, theta_c)

        if _ai_enabled:
            msg = Pose2D()
            msg.x = x_c_safe
            msg.y = y_c_safe
            msg.theta = theta_c_safe
            pub.publish(msg)

        _was_goal = False

        rate.sleep()

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    main()
