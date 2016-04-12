from collections import Iterable
import numpy as np 

import Constants
from GameObjects import Robot

_ally1_stuck_counter    = 0
_ALLY1_STUCK_MAX        = 500
_ally2_stuck_counter    = 0
_ALLY2_STUCK_MAX        = 500
_ally1_prev_pos         = (Constants.ally1_start_pos[0], Constants.ally1_start_pos[1])
_ally2_prev_pos         = (Constants.ally2_start_pos[0], Constants.ally2_start_pos[1])

_stuck                  = False


def get_front_of_robot(robot):
    theta = deg_to_rad(robot.thetahat)
    x_pos = robot.xhat+Constants.robot_half_width*np.cos(theta) 
    y_pos = robot.yhat+Constants.robot_half_width*np.sin(theta)

    return (x_pos, y_pos)

#################################################################
#################################################################
####            FUNCTIONS COMPUTING CLOSEST OBJECTS          #### 
#################################################################
#################################################################
def get_closest_opponent_to_ball(opponent1, opponent2, ball):
    if opponent2 is None:
        return opponent1
    else:
        return _get_closest_robot_to_point(opponent1, opponent2, ball.xhat, ball.yhat)

def am_i_closest_teammate_to_ball(me, my_teammate, ball):
    if my_teammate is None: return True
    closest = _get_closest_robot_to_point(me, my_teammate, ball.xhat, ball.yhat)
    if (closest == me): 
        return True
    else: 
        return False

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

def am_i_too_close_to_teammate(me, my_teammate):
    dist_from_each_other = get_distance_between_points(me.xhat, me.yhat, my_teammate.xhat_future, my_teammate.yhat_future)
    if dist_from_each_other < Constants.teammate_gap:
        return True
    else:
        return False

def is_opp_too_close_to_kicker(me, opp1, opp2, ball):
    closest_opp = get_closest_opponent_to_ball(opp1, opp2, ball)
    (x_kicker, y_kicker) = get_front_of_robot(me)
    dist_kicker_to_opp = get_distance_between_points(x_kicker, y_kicker, closest_opp.xhat, closest_opp.yhat)

    if dist_kicker_to_opp <= Constants.robot_half_width+0.05:
        return True
    else:
        return False    


#################################################################
#################################################################
####       FUNCTIONS FOR DETECTING OPPONENTS' STRATEGY       #### 
#################################################################
#################################################################
def are_both_opponents_attacking_goal(opponent1, opponent2, ball):
    pass

def is_in_our_half(obj):
    if obj.xhat < 0:
        return True
    else:
        return False

def has_possession():
    pass

#################################################################
#################################################################
####            FUNCTIONS DEALING WITH THE BALL              #### 
#################################################################
#################################################################

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

def is_ball_close_to_edges(ball):
    if abs(ball.xhat) > Constants.field_x_lim or abs(ball.yhat) > Constants.field_y_lim:
        return True
    else:
        return False


#################################################################
#################################################################
####       FUNCTIONS FOR AVOIDING SCORING ON OURSELVES       #### 
#################################################################
#################################################################
def get_perpendicular_point_from_ball(me, ball):
    x_c = ball.xhat - 0.15
    direction_toggle = 1 # This will switch the side the robot will approach if the ball is too close to a wall.
    if (abs(ball.yhat) > Constants.field_width/2 - Constants.robot_width*0.60): # A little more than half width
        direction_toggle = -1

    if me.yhat > ball.yhat:
        y_c = ball.yhat + Constants.own_goal_y_dist*direction_toggle # CHANGED THE SIGN OF SUBTRACTIN FROM ADDING, TO SEE IF IT WOULD FIX IT!
    else:
        y_c = ball.yhat - Constants.own_goal_y_dist*direction_toggle
    theta_c = 0
    return (x_c, y_c, theta_c)

def get_own_goal_dist_behind_ball(me, ball):
    x_c = ball.xhat - Constants.own_goal_x_dist
    y_c = ball.yhat
    theta_c = 0
    return (x_c, y_c, theta_c)


#################################################################
#################################################################
####       FUNCTIONS FOR FINDING DISTANCES AND ANGLES        #### 
#################################################################
#################################################################
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

def rad_to_deg(rad):
    deg = rad*180/np.pi
    if deg < 0:
        return deg+360
    else:
        return deg

def deg_to_rad(deg):
    return deg*np.pi/180



#################################################################
#################################################################
####       FUNCTIONS TO DETECT IF OBJECTS OR POS ARE CLOSE   #### 
#################################################################
#################################################################

def robot_close_to_point(robot, point_x, point_y, theta):
    return close(point_x, robot.xhat, tolerance=.07) and close(point_y, robot.yhat, tolerance=.07) \
                and close(theta, robot.thetahat, tolerance=10) # within 10cm of x and y, and 10 degree tolerance for theta

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

#################################################################
#################################################################
####                MISCELLANEOUS FUNCTIONS                  #### 
#################################################################
#################################################################

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



def limit_xy_too_close_to_walls(x,y):
    if abs(x) > Constants.field_x_lim:
        if x > 0:
            x = Constants.field_x_lim
        else:
            x = -Constants.field_x_lim
    if abs(y) > Constants.field_y_lim:
        if y > 0:
            y = Constants.field_y_lim
        else:
            y = -Constants.field_y_lim
    return (x,y)


def limit_xy_passing(x,y):
    if abs(x) >= Constants.field_x_lim - 0.15:
        if x > 0:
            x = Constants.field_x_lim - 0.15
        else:
            x = -Constants.field_x_lim - 0.15
    if abs(y) > Constants.field_y_lim - 0.15:
        if y > 0:
            y = Constants.field_y_lim - 0.15
        else:
            y = -Constants.field_y_lim - 0.15
    return (x,y)


# Update this. It won't work because it will only return true for one second....
def i_am_stuck(me):
    global _ally1_stuck_counter, _ALLY1_STUCK_MAX
    global _ally2_stuck_counter, _ALLY2_STUCK_MAX 
    global _ally1_prev_pos, _ally2_prev_pos 
    global _stuck
    
    if me.ally1:
        if _ally1_stuck_counter >= _ALLY1_STUCK_MAX: #Counter expired, so reset counter and return True
            _ally1_stuck_counter = 0
            _ally1_prev_pos = (me.xhat, me.yhat)
            return True
        else: # Keep checking to see if the robot is stuck
            dist_from_prev_pos = get_distance_between_points(me.xhat, me.yhat, _ally1_prev_pos[0], _ally1_prev_pos[1])
            if dist_from_prev_pos <= Constants.not_stuck_dist:
                _ally1_stuck_counter = _ally1_stuck_counter + 1
                _ally1_prev_pos = (me.xhat, me.yhat)
            else:
                _ally1_stuck_counter = 0
            return False
    else: # I am ally2
        if _ally2_stuck_counter >= _ALLY2_STUCK_MAX: #Counter expired, so reset counter and return True
            _ally2_stuck_counter = 0
            _ally2_prev_pos = (me.xhat, me.yhat)
            return True
        else: # Keep checking to see if the robot is stuck
            dist_from_prev_pos = get_distance_between_points(me.xhat, me.yhat, _ally2_prev_pos[0], _ally2_prev_pos[1])
            if dist_from_prev_pos <= Constants.not_stuck_dist:
                _ally2_stuck_counter = _ally2_stuck_counter + 1
                _ally2_prev_pos = (me.xhat, me.yhat)
            else:
                _ally2_stuck_counter = 0
            return False