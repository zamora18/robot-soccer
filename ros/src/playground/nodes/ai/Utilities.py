from collections import Iterable
import numpy as np 

import Constants



def get_closest_opponent_to_ball(opponent1, opponent2, ball):
    if opponent2 is None:
        return opponent1
    else:
        return _get_closest_robot_to_point(opponent1, opponent2, ball.xhat, ball.yhat)

def am_i_closest_teammate_to_ball(me, my_teammate, ball):
    if my_teammate is None: return True
    closest = _get_closest_robot_to_point(me, my_teammate, ball.xhat, ball.yhat)
    if (closest == me): return True
    else: return False

def am_i_closer_to_ball_than_opponents(me, opponent1, opponent2, ball):
    me_or_opp1 = _get_closest_robot_to_point(me, opponent1, ball.xhat, ball.yhat)
    me_or_opp2 = _get_closest_robot_to_point(me, opponent2, ball.xhat, ball.yhat)

    if (me_or_opp1 == me and me_or_opp2 == me):
        return True
    else:
        return False

def _get_closest_robot_to_point(rob1, rob2, point_x, point_y):
    if rob2 is None: return rob1
    rob_1_dist = get_distance_between_points(rob1.xhat, rob1.yhat, point_x, point_y)
    rob_2_dist = get_distance_between_points(rob2.xhat, rob2.yhat, point_x, point_y)
    if (rob_1_dist < rob_2_dist):
        return rob1
    else:
        return rob2 

def are_both_opponents_attacking_goal(opponent1, opponent2, ball):
    pass

def has_possession():
    pass

def is_ball_behind_robot(robot, ball):
    if (robot.xhat > ball.xhat):
        return True 
    else: 
        return False

def is_ball_between_home_and_robot(robot, ball):
    epsilon = Constants.robot_width/4

    crossproduct = (ball.yhat - robot.yhat) * (Constants.goal_position_home[0] - robot.xhat) - (ball.xhat - robot.xhat) * (Constants.goal_position_home[1] - robot.yhat)
    if abs(crossproduct) > epsilon : return False   # (or != 0 if using integers)

    dotproduct = (ball.xhat - robot.xhat) * (Constants.goal_position_home[0] - robot.xhat) + (ball.yhat - robot.yhat)*(Constants.goal_position_home[1] - robot.yhat)
    if dotproduct < 0 : return False

    squaredlengthba = (Constants.goal_position_home[0] - robot.xhat)*(Constants.goal_position_home[0] - robot.xhat) + (Constants.goal_position_home[1] - robot.yhat)*(Constants.goal_position_home[1] - robot.yhat)
    if dotproduct > squaredlengthba: return False

    return True

def get_sides_of_triangle(x1,x2,y1,y2):
    (a,b,c,theta) = find_triangle(x1,x2,y1,y2)
    return (a,b)

def get_distance_between_points(x1,y1,x2,y2):
    (a,b,c,theta) = find_triangle(x1,y1,x2,y2)
    return c

def get_angle_between_points(x1,y1,x2,y2):
    (a,b,c,theta) = find_triangle(x1,y1,x2,y2)
    return theta

def find_triangle(x1,y1,x2,y2):
    """Find Triangle

    returns the triangle's sides (a, b, c) and theta (in radians) between p1 and p2
    input coordinates should be "FROM (x1,y1) TO (x2,y2)
    """
    # a, b and c are lengths of the side of a right triangle betweem p1 and p2
    #import ipdb; ipdb.set_trace() #FOR DEBUGGING!
    a = x2 - x1
    b = y2 - y1
    c = np.sqrt(a**2 + b**2)

    # theta is the angle between y=0 line at our goal to the ball
    theta = np.arctan2(b,a)

    return (a,b,c,theta)

def get_front_of_robot(robot):
    ### thetahat is in degrees, so we should change from degree to radians?? ----------
    theta = deg_to_rad(robot.thetahat)
    x_pos = robot.xhat+Constants.robot_half_width*np.cos(theta) 
    y_pos = robot.yhat+Constants.robot_half_width*np.sin(theta)

    return (x_pos, y_pos)


def rad_to_deg(rad):
    deg = rad*180/np.pi
    if deg < 0:
        return deg+360
    else:
        return deg

def deg_to_rad(deg):
    return deg*np.pi/180

def robot_close_to_point(robot, point_x, point_y, theta):
    return close(point_x, robot.xhat, tolerance = .10) and close(point_y, robot.yhat, tolerance=.10) \
                and close(theta, robot.thetahat, tolerance = 10) # within 10cm of x and y, and 10 degree tolerance for theta

def close(a, b, tolerance=0.010):
    """
    Usage: bool = _close([1, 2], [1.1, 2.3], tolerance=0.4) # true
    """

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

def get_field_section(x_pos):
        #Field is divided into 4 sections. 2 back half, 2 front half.
        home_back_fourth     = -Constants.fourth_field_length 
        home_front_fourth    = Constants.half_field
        away_back_fourth     = Constants.fourth_field_length
        away_front_fourth    = 2*Constants.fourth_field_length

        if x_pos < Constants.half_field:
            if x_pos < home_back_fourth:
                return 1
            else:
                return 2
        else: 
            if x_pos < away_back_fourth:
                return 3
            else:
                return 4