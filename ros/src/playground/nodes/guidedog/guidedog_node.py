#!/usr/bin/env python
"""
This node is a guidedog to the robot (i.e., the robot is blind).

It 
"""
from collections import Iterable

import roslib; roslib.load_manifest('playground')
import rospy
from geometry_msgs.msg import Pose2D
from playground.msg import coords

import numpy as np

_robot_width        = 0.1841 # (7.25 in)
_robot_half_width   = _robot_width/2
_field_length       = 3.68 # (12ft)
_field_width        = 2.62 # (8.58 ft)

_opponent = None
_ally = None
_robot_desired = None

_pub = None

def _handle_raw_desired_position(msg):
    global _robot_desired

    _robot_desired = (msg.x, msg.y, msg.theta)

def _handle_opponent_position(msg):
    global _opponent

    # Grab and save the opponent's current location
    _opponent = (msg.x, msg.y, msg.theta)

def _handle_ally_position(msg):
    global _ally

    # Grab and save the ally's current location
    _ally = (msg.x, msg.y, msg.theta)

    # Create desired message
    desired_msg = Pose2D()
    desired_msg.x = _robot_desired[0] if _robot_desired is not None else msg.x
    desired_msg.y = _robot_desired[1] if _robot_desired is not None else msg.y
    desired_msg.theta = _robot_desired[2] if _robot_desired is not None else msg.theta

    if _ally is not None:
        robot = (_ally[0], _ally[1])
    else:
        robot = (0, 0)

    if _opponent is not None:
        opponent = (_opponent[0], _opponent[1])
    else:
        # don't ever be close to this
        opponent = (100, 100)

    _edge_padding = 0.25

    # Define field edges
    min_x = -((_field_length/2) - _edge_padding)
    max_x = ((_field_length/2) - _edge_padding)
    min_y = -((_field_width/2) - _edge_padding)
    max_y = ((_field_width/2) - _edge_padding)

    # Are we within 10% of the perimeter of the opponent?
    if _close(robot, opponent, tolerance=1.60*_robot_width):
        print "You're close to a robot!"

    # Are we about to hit the edge of the field?
    if _close(robot[0], min_x, tolerance=_edge_padding) and desired_msg.x < min_x:
        print("{} is close to {}".format(robot[0],min_x))
        desired_msg.x = min_x

    if _close(robot[0], max_x, tolerance=_edge_padding) and desired_msg.x > max_x:
        print("{} is close to {}".format(robot[0],max_x))
        desired_msg.x = max_x

    if _close(robot[1], min_y, tolerance=_edge_padding) and desired_msg.y < min_y:
        print("{} is close to {}".format(robot[1],min_y))
        desired_msg.y = min_y

    if _close(robot[1], max_y, tolerance=_edge_padding) and desired_msg.y > max_y:
        print("{} is close to {}".format(robot[1],max_y))
        desired_msg.y = max_y

    _pub.publish(desired_msg)




def _close(a, b, tolerance=0.010):

    # Demand vals to be lists
    a = _demand_list(a)
    b = _demand_list(b)

    return all(abs(np.subtract(a, b)) <= tolerance)

def _demand_list(a):
    """
    Make a non-iterable or a tuple into a list
    """
    if not isinstance(a, Iterable):
        a = [a]

    elif type(a) is tuple:
        a = list(a)

    return a

def main():
    rospy.init_node('guidedog', anonymous=False)

    print("Woof! Woof!")

    global _pub
    _pub = rospy.Publisher('desired_position_safe', Pose2D, queue_size=10)

    rospy.Subscriber('desired_position', Pose2D, _handle_raw_desired_position)
    rospy.Subscriber('vision_opponent_position', Pose2D, _handle_opponent_position)
    rospy.Subscriber('vision_ally_position', Pose2D, _handle_ally_position)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    main()
