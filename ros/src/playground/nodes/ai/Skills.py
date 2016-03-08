import os
import numpy as np

_field_length       = 3.68 # (12ft)
_field_width        = 2.62 # (8.58 ft)
_robot_width        = 0.1841 # (7.25 in)
_goal_box_width     = 0.660 # (26 in)
_goal_position_home = -_field_length/2
_goal_position_opp  = -_goal_position_home

_distance_behind_ball_for_kick = _robot_width/2 + .05 # this is for the jersey being off center

# actuates solenoid
def kick():
    os.system("echo 1 > /sys/class/gpio/gpio200/value; sleep .1; echo 0 > /sys/class/gpio/gpio200/value")

def _find_triangle(x1,y1,x2,y2):
    # a, b and c are lengths of the side of a right triangle betweem p1 and p2
    a = x2 - x1
    b = y2 - y1
    c = np.sqrt(a**2 + b**2)

    # theta is the angle between y=0 line at our goal to the ball
    theta = np.arctan2(b,a)

    return (a,b,c,theta)

# distance should be between 0 and 1, scaled from the first point
# example follow ball 2/3 distance between goal and ball
def stay_between_points_at_distance(x1, y1, x2, y2, distance):

    a,b,c,theta = _find_triangle(x1,y1,x2,y2)

    cprime = c*(1-distance)

    # aprime is the length of the simalar triangle with hypotenous cprime
    aprime = cprime*np.cos(theta)
    # bprime is the height of the simalar triangle with hypotenous cprime
    bprime = cprime*np.sin(theta)

    x_desired = x2 - aprime
    y_desired = y2 - bprime

    theta = theta*180/np.pi

    return (x_desired, y_desired, theta)

# distance from center is for shooting in corners, 1 will put it in the "top" corner
# while -1 will put it in the bottom of the goal and 0 is exact center
def set_up_kick(robot, ball, distance_from_center_of_goal):



    y2 = distance_from_center_of_goal * _goal_box_width/2
    x2 = _goal_position_opp

    a,b,c,theta = _find_triangle(ball['xhat'], ball['yhat'], x2,y2)

    cprime = _distance_behind_ball_for_kick

    # aprime is the length of the simalar triangle with hypotenous cprime
    aprime = cprime*np.cos(theta)
    # bprime is the height of the simalar triangle with hypotenous cprime
    bprime = cprime*np.sin(theta)

    x_c = ball['xhat'] - aprime
    y_c = ball['yhat'] + bprime

    theta_c = theta

    return (x_c, y_c, theta_c)
