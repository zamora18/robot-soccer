import numpy as np 

import Constants



def our_robot_closer_to_ball(robot, opponent, ball):
    closest = _get_closest_robot_to_point(robot['xhat'], robot['yhat'], opponent['xhat'], opponent['yhat'], ball['xhat'], ball['yhat'])
    if (closest == 1):
        return True 
    else:
        return False

def _get_closest_robot_to_point(rob1_x, rob1_y, rob2_x, rob2_y, point_x, point_y):
    rob_1_dist = get_distance_between_points(rob1_x, rob1_y, point_x, point_y)
    rob_2_dist = get_distance_between_points(rob2_x, rob2_y, point_x, point_y)
    if (rob_1_dist < rob_2_dist):
        return 1
    else:
        return 2

def ball_is_behind_robot(robot, ball):
    if (robot['xhat']-Constants.robot_half_width > ball['xhat']):
        return True 
    else: 
        return False



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
    """
    # a, b and c are lengths of the side of a right triangle betweem p1 and p2
    a = x2 - x1
    b = y2 - y1
    c = np.sqrt(a**2 + b**2)

    # theta is the angle between y=0 line at our goal to the ball
    theta = np.arctan2(b,a)

    return (a,b,c,theta)

def get_front_of_robot(robot):
    x_pos = robot['xhat']+Constants.robot_half_width*cos(robot['thetahat']) ### thetahat is in degrees, so we should change from degree to radians?? ----------
    y_pos = robot['yhat']+Constants.robot_half_width*sin(robot['thetahat'])

    return (x_pos, y_pos)


def rad_to_deg(rad):
    return rad*180/np.pi

def deg_to_rad(deg):
    return deg*np.pi/180

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

        if x_pos < _half_field:
            if x_pos < home_back_fourth:
                return 1
            else:
                return 2
        else: 
            if x_pos < away_back_fourth:
                return 3
            else:
                return 4