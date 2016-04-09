#!/usr/bin/env python
"""
This node is a guidedog to the robot (i.e., the robot is blind).

It 
"""
import sys, os
from collections import Iterable

import roslib; roslib.load_manifest('playground')
import rospy
from geometry_msgs.msg import Pose2D
from playground.msg import RobotState

import numpy as np

sys.path.append(os.path.join(os.path.dirname(__file__), '..', 'ai/'))
import Utilities

import avoidance

_robot_width        = 0.1841 # (7.25 in)
_robot_half_width   = _robot_width/2
_field_length       = 3.68 # (12ft)
_field_width        = 2.62 # (8.58 ft)

_opponent = None
_ally = None
_robot_desired = None

_go_rogue = False

_pub = None

def _handle_raw_desired_position(msg):
    global _robot_desired

    _robot_desired = (msg.x, msg.y, msg.theta)

def _handle_opponent_position(opp, msg):
    global _opponent1, _opponent2

    # Grab and save the opponent's current location
    if opp == 1:
        _opponent1 = (msg.xhat, msg.yhat, msg.thetahat)
    else:
        _opponent2 = (msg.xhat, msg.yhat, msg.thetahat)

def _handle_my_position(msg):
    global _ally

    if _robot_desired is None:
        return

    # Grab and save the ally's current location
    _ally = (msg.xhat, msg.yhat, msg.thetahat)

    if _ally is not None:
        robot = (_ally[0], _ally[1])
    else:
        robot = (0, 0)

    if _opponent1 is not None:
        opponent1 = (_opponent1[0], _opponent1[1])
    else:
        # don't ever be close to this
        opponent1 = (100, 100)

    if _opponent2 is not None:
        opponent2 = (_opponent2[0], _opponent2[1])
    else:
        # don't ever be close to this
        opponent2 = (100, 100)

    if _robot_desired is not None:
        desired = (_robot_desired[0], _robot_desired[1], _robot_desired[2])
    else:
        desired = (msg.xhat, msg.yhat, msg.thetahat)

    _edge_padding = (0.20, 0.15)

    # Define field edges
    min_x = -((_field_length/2) - _edge_padding[0])
    max_x = ((_field_length/2) - _edge_padding[0])
    min_y = -((_field_width/2) - _edge_padding[1])
    max_y = ((_field_width/2) - _edge_padding[1])

    # Create desired message
    desired_msg = Pose2D()
    desired_msg.x = desired[0]
    desired_msg.y = desired[1]
    desired_msg.theta = desired[2]

    # Hack go rogue. Fix this later, it's messy
    if _go_rogue:

        # find the closest robot
        thing = Utilities._get_closest_robot_to_point(opponent1, opponent2, robot[0], robot[1])

        # Pass points to avoid
        (x_c, y_c) = avoidance.avoid(robot, desired[0:2], thing)

        c = Pose2D()
        c.x = x_c
        c.y = y_c
        c.theta = desired[2]
        _pub.publish(c)
        return

    if _go_rogue:
        # Publish through un-guidedog'ed and bail
        _pub.publish(desired_msg)
        return

    # Are we within 50% of the perimeter of the opponent?
    if _close(robot, opponent, tolerance=1.50*_robot_width):
        # Go to the closest defensive side.
        print "You're close to a robot!"
        #desired_msg.x = _ally[0]
        #desired_msg.y = _ally[1]
        #desired_msg.theta = _ally[2]

    
    # if _is_opponent_in_path(robot, opponent, desired, buffer=1.60*_robot_width):
    #     desired = _choose_offset_point(robot, opponent, desired)

    # Are we about to hit the edge of the field?
    if _close(robot[0], min_x, tolerance=_edge_padding[0]) and desired[0] < min_x:
        # print("{} is close to {}".format(robot[0],min_x))
        desired_msg.x = min_x

    if _close(robot[0], max_x, tolerance=_edge_padding[0]) and desired[0] > max_x:
        # print("{} is close to {}".format(robot[0],max_x))
        desired_msg.x = max_x

    if _close(robot[1], min_y, tolerance=_edge_padding[1]) and desired[1] < min_y:
        # print("{} is close to {}".format(robot[1],min_y))
        desired_msg.y = min_y

    if _close(robot[1], max_y, tolerance=_edge_padding[1]) and desired[1] > max_y:
        # print("{} is close to {}".format(robot[1],max_y))
        desired_msg.y = max_y

    _pub.publish(desired_msg)


def _is_opponent_in_path(robot, opponent, desired, buffer=None):
    
    p1 = robot[0:2]
    p2 = desired[0:2]
    p3 = opponent[0:2]

    if buffer is None:
        buffer = 1.0

    return _is_between(p1, p2, p3, epsilon=buffer)


def _choose_offset_point(robot, opponent, desired):
    pass

def _is_between(p1, p2, p3, epsilon=1.0):
    """Is Between

    p1, p2, p3 are in the form (x, y).
    """
    crossproduct = (c[1] - a[1]) * (b[0] - a[0]) - (c[0] - a[0]) * (b[1] - a[1])
    if abs(crossproduct) > epsilon : return False   # (or != 0 if using integers)

    dotproduct = (c[0] - a[0]) * (b[0] - a[0]) + (c[1] - a[1])*(b[1] - a[1])
    if dotproduct < 0 : return False

    squaredlengthba = (b[0] - a[0])*(b[0] - a[0]) + (b[1] - a[1])*(b[1] - a[1])
    if dotproduct > squaredlengthba: return False

    return True
    # return (True, crossproduct, dotproduct, squaredlengthba)


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

    global _pub, _go_rogue
    _pub = rospy.Publisher('desired_position_safe', Pose2D, queue_size=10)

    rospy.Subscriber('desired_position', Pose2D, _handle_raw_desired_position)
    rospy.Subscriber('my_state', RobotState, _handle_my_position)
    # rospy.Subscriber('vision_ally_position', Pose2D, _handle_ally_position)

    # rospy.Subscriber('my_state', RobotState, lambda msg: _handle_robot_state(msg, 'me'))
    # rospy.Subscriber('ally_state', RobotState, lambda msg: _handle_robot_state(msg, 'ally'))
    rospy.Subscriber('opponent1_state', RobotState, lambda x: _handle_opponent_position(1, x))
    rospy.Subscriber('opponent2_state', RobotState, lambda x: _handle_opponent_position(2, x))

    _go_rogue = rospy.get_param('go_rogue', 'false')

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    main()
