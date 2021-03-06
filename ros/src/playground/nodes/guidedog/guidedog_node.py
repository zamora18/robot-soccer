#!/usr/bin/env python
"""
This node is a guidedog to the robot (i.e., the robot is blind).
"""
import sys, os
from collections import Iterable

import roslib; roslib.load_manifest('playground')
import rospy
from geometry_msgs.msg import Pose2D
from playground.msg import RobotState, GameState

import numpy as np

sys.path.append(os.path.join(os.path.dirname(__file__), '..', 'ai/'))
import Utilities
import Constants
from GameObjects import Robot

import avoidance

_me = None
_ally = None
_opp1 = None
_opp2 = None

_go_rogue = False
_avoid_opponents = True
_avoid_walls = True

_pub = None

_edge_padding = None
_goal_y_tolerance = None

_one_v_one = True

def _handle_raw_desired_position(msg):
    _me.update_desired(msg)

    if _go_rogue:
        # Send the unchanged desired position as they come through
        unsafe_msg = Pose2D()
        unsafe_msg.x = msg.x
        unsafe_msg.y = msg.y
        unsafe_msg.theta = msg.theta
        _pub.publish(unsafe_msg)

def _handle_game_state(msg):
    global _one_v_one

    # Our naming is terrible! Oh well.
    _one_v_one = not msg.two_v_two

def _handle_robot_state(msg, which_robot):
    # Update the given robot's current and future positions
    if which_robot == 'me' and _me is not None:
        _me.update_state(msg)
    elif which_robot == 'ally' and _ally is not None:
        _ally.update_state(msg)
    elif which_robot == 'opp1' and _opp1 is not None:
        _opp1.update_state(msg)
    elif which_robot == 'opp2' and _opp2 is not None:
        _opp2.update_state(msg)

    if _go_rogue:
        return # Don't even think about being safe

    # Have the guidedog process the field state and make everything safe!
    # Do this any time that the state of the robots on the field change
    safe_c = lead_me_guide_me()

    if safe_c is not None:
        # Send the safe desired position off
        safe_msg = Pose2D()
        safe_msg.x = safe_c[0]
        safe_msg.y = safe_c[1]
        safe_msg.theta = safe_c[2]
        _pub.publish(safe_msg)

def lead_me_guide_me():
    """Lead Me Guide Me

    This is the real meat of this node. These are the things that it does
    for you:

        1) Avoids running into walls
        2) Avoids running into the goal
        3) Avoids hitting opponents

    One thing to note is that without this node, or with _go_rogue = True,
    once the robot gets to its desired position, it will stop trying to go
    there. With this node on, however, it doesn't know that it is where
    it is supposed to be (that's the controller's job), and therefore
    it keeps sending the safe_desired_position, even if there are no desired
    positions being sent. This really isn't a problem, the robot will
    just try and 'hold its ground.'
    """

    # Define field edges
    min_x = -((Constants.field_length/2.0) - _edge_padding[0])
    max_x =  ((Constants.field_length/2.0) - _edge_padding[0])
    min_y = -((Constants.field_width/2.0)  - _edge_padding[1])
    max_y =  ((Constants.field_width/2.0)  - _edge_padding[1])

    # Allow x to have less padding in the goal region
    max_x_in_goal_region = max_x + Constants.goal_box_length
    min_x_in_goal_region = min_x - Constants.goal_box_length

    # If I haven't been given a place to go yet, there is no point
    # in making sure that it is safe...
    if not _me.has_a_desired_position():
        return None

    # initialize as my current, unchanged desired position
    safe_c = [_me.x_c, _me.y_c, _me.theta_c]

    if _avoid_opponents:

        # find the closest robot
        if _one_v_one:
            # Well, there may actually be two opponents, but we only 
            # will avoid opp1 (see main.cpp for vision)
            robot_obstacle = _opp1

        else:
            # Choose between the two opponents as to which is closest        
            robot_obstacle = Utilities._get_closest_robot_to_point( \
                                            _opp1, _opp2, *_me.get_2d_location())

            # And now compare the closest opponent to the closest ally
            robot_obstacle = Utilities._get_closest_robot_to_point( \
                                            _ally, robot_obstacle, *_me.get_2d_location())

        # Pass points to avoid
        (x_c, y_c) = avoidance.avoid(_me.get_2d_location(), _me.get_2d_desired(), \
                                            robot_obstacle.get_2d_location())

        safe_c = [x_c, y_c, _me.theta_c]

    if _avoid_walls:
        # Are we about to hit the edge of the field?

        # Left and right of field (by goals)
        if Utilities.close(_me.xhat, min_x, tolerance=_edge_padding[0]) and _me.x_c < min_x:
            # If I'm in the goal region and I'm close to the wall...
            if in_goal_region():
                if _me.x_c < min_x_in_goal_region:
                    safe_c[0] = min_x_in_goal_region
                else:
                    pass # i.e., let safe_c[0] be unchanged
            else:
                safe_c[0] = min_x
        if Utilities.close(_me.xhat, max_x, tolerance=_edge_padding[0]) and _me.x_c > max_x:
            # If I'm in the goal region and I'm close to the wall...
            if in_goal_region():
                if _me.x_c > max_x_in_goal_region:
                    safe_c[0] = max_x_in_goal_region
                else:
                    pass # i.e., let safe_c[0] be unchanged
            else:
                safe_c[0] = max_x

        # Top and bottom of field
        if Utilities.close(_me.yhat, min_y, tolerance=_edge_padding[1]) and _me.y_c < min_y:
            safe_c[1] = min_y
        if Utilities.close(_me.yhat, max_y, tolerance=_edge_padding[1]) and _me.y_c > max_y:
            safe_c[1] = max_y

    return safe_c

def in_goal_region():
    # Figure out goal top and bottom points (create goal region)
    goal_top_y =  (Constants.goal_box_width/2.0) + _goal_y_tolerance
    goal_bot_y = -((Constants.goal_box_width/2.0) + _goal_y_tolerance)

    return (_me.yhat >= goal_bot_y) and (_me.yhat <= goal_top_y)

def main():
    rospy.init_node('guidedog', anonymous=False)

    # Create robot objects that store that current robot's state
    global  _me, _ally, _opp1, _opp2
    (_me, _ally, _opp1, _opp2) = (Robot(), Robot(), Robot(), Robot())

    global _pub
    _pub = rospy.Publisher('desired_position_safe', Pose2D, queue_size=10)

    # How many meters away from the edges should we stay?
    global _edge_padding
    epx = rospy.get_param('edge_padding/x', 0.05)
    epy = rospy.get_param('edge_padding/y', 0.05)
    _edge_padding = (epx, epy)

    # And the goal tolerance?
    global _goal_y_tolerance
    _goal_y_tolerance = rospy.get_param('goal_tolerance/y', 0.200)

    # How much should the guidedog do?
    global _go_rogue, _avoid_opponents, _avoid_walls
    _go_rogue = rospy.get_param('go_rogue', False)
    _avoid_opponents = rospy.get_param('avoid_opponents', True)
    _avoid_walls = rospy.get_param('avoid_walls', True)
    
    # Subscribe to Robot States
    rospy.Subscriber('my_state', RobotState, lambda msg: _handle_robot_state(msg, 'me'))
    rospy.Subscriber('ally_state', RobotState, lambda msg: _handle_robot_state(msg, 'ally'))
    rospy.Subscriber('opponent1_state', RobotState, lambda msg: _handle_robot_state(msg, 'opp1'))
    rospy.Subscriber('opponent2_state', RobotState, lambda msg: _handle_robot_state(msg, 'opp2'))
    rospy.Subscriber('desired_position', Pose2D, _handle_raw_desired_position)

    rospy.Subscriber('/game_state', GameState, _handle_game_state)

    print("Woof! Woof!")

    avoid_str = ""
    if _go_rogue:
        avoid_str = "Nothing!"
    else:
        if _avoid_opponents and _avoid_walls:
            avoid_str = "Opponents and walls"
        elif _avoid_opponents and not _avoid_walls:
            avoid_str = "Opponents"
        elif not _avoid_opponents and _avoid_walls:
            avoid_str = "Walls"
        else:
            avoid_str = "Nothing, but the robot will hold its ground"

    print("Guidedog for {} will avoid: {}".format( \
            rospy.get_namespace(), avoid_str ))

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    main()