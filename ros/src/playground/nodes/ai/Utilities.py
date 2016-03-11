import Numpy as np 

import Constants






# make function that detects the closest "robot" to the ball. Call it our_rob_closer and return T/F
# or call it closest robot and return 1 or 2


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
    if (robot['xhat'] < 0):
        x_pos = robot['xhat']-Constants.robot_half_width*cos(robot['thetahat'])
        if (robot['yhat'])
        y_pos = robot['xhat']-Constants.robot_half_width*cos(robot['thetahat'])
    else:
        x_pos = robot['xhat']-Constants.robot_half_width*cos(robot['thetahat'])


def rad_to_deg(rad):
    return rad*180/np.pi

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
        home_back_fourth     = -_fourth_field_length 
        home_front_fourth    = _half_field
        away_back_fourth     = _fourth_field_length
        away_front_fourth    = _field_length/2

        if x_pos < _half_field:
            if x_pos < home_back_fourth:
                return 1
            else: # home_front_fourth
                return 2
        else: # ball is in away half
            if x_pos < away_back_fourth:
                return 3
            else:
                return 4