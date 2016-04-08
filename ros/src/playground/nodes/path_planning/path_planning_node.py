#!/usr/bin/env python
import sys

import roslib; roslib.load_manifest('playground')
import rospy
from geometry_msgs.msg import Pose2D
from std_msgs.msg import Bool

from playground.msg import BallState, RobotState
from playground.srv import SetBool, SetBoolResponse

import numpy as np


sys.path.append('../ai/')
import Path
from GameObjects import Ball, Robot

_me = None
_ally = None
_opp1 = None
_opp2 = None

_x_c = 0
_y_c = 0
_theta_c = 0

_pathplanning_enabled = True

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

def _handle_desired_position(msg):
    global _x_c, _y_c, _theta_c
    _x_c = msg.x
    _y_c = msg.y
    _theta_c = msg.theta


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
    rospy.init_node('path_planner', anonymous=False)

    # Create robot objects that store that current robot's state
    _create_robots()

    # Initialize our path planning world
    Path.initialize_world()

    # Subscribe to Robot States
    rospy.Subscriber('my_state', RobotState, lambda msg: _handle_robot_state(msg, 'me'))
    rospy.Subscriber('ally_state', RobotState, lambda msg: _handle_robot_state(msg, 'ally'))
    rospy.Subscriber('opponent1_state', RobotState, lambda msg: _handle_robot_state(msg, 'opp1'))
    rospy.Subscriber('opponent2_state', RobotState, lambda msg: _handle_robot_state(msg, 'opp2'))

    rospy.Subscriber('desired_position', Pose2D, _handle_desired_position)
    pub = rospy.Publisher('desired_position_safe', Pose2D, queue_size=10)

    global _pathplanning_enabled
    _pathplanning_enabled = rospy.get_param('plan_paths', False)

    rate = rospy.Rate(100) #100 Hz
    while not rospy.is_shutdown():
        if _pathplanning_enabled:
            (x_c_safe, y_c_safe) = Path.plan(_x_c, _y_c, _me, _ally, _opp1, _opp2)
            theta_c_safe = _theta_c
        else:
            (x_c_safe, y_c_safe, theta_c_safe) = (_x_c, y_c, _theta_c)

        msg = Pose2D()
        msg.x = x_c_safe
        msg.y = y_c_safe
        msg.theta = theta_c_safe
        pub.publish(msg)

        rate.sleep()

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    main()