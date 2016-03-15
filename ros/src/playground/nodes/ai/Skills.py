import os
import numpy as np

import Utilities
import Constants


_distance_behind_ball_for_kick      = Constants.robot_width + .03 # this is for the jersey being off center
_distance_behind_ball_for_dribbling = Constants.robot_width/2 + .05
_distance_from_goal_for_arc_defense = Constants.goal_box_width + Constants.robot_width *2
_distance_behind_ball_approach      = .3

_kicker_count = 0

# actuates solenoid
def kick():
    global _kicker_count
    _kicker_count = _kicker_count + 1
    print("Actuate: {}".format(_kicker_count))
    os.system("echo 1 > /sys/class/gpio/gpio200/value; sleep .1; echo 0 > /sys/class/gpio/gpio200/value")

def dribble_forward(robot, ball):
    x_c = robot['xhat']+Constants.dribble_distance*cos(robot['thetahat'])
    y_c = robot['yhat']+Constants.dribble_distance*sin(robot['thetahat'])
    theta_c = robot['thetahat']
    return (x_c, y_c, theta_c)


def dribble_to_point(robot, ball):
    pass


def dribble_along_line(robot, ball):
    pass




def stay_between_points_at_distance(x1, y1, x2, y2, distance):
    """
    Distance should be between 0 and 1, scaled from the first point
    example follow ball 2/3 distance between goal and ball
    """
    theta = Utilities.get_angle_between_points(x1,y1,x2,y2)
    c = Utilities.get_distance_between_points(x1,y1,x2,y2)
    cprime = c*(1-distance)

    # aprime is the length of the simalar triangle with hypotenuse cprime
    aprime = cprime*np.cos(theta)
    # bprime is the height of the simalar triangle with hypotenuse cprime
    bprime = cprime*np.sin(theta)

    x_desired = x2 - aprime
    y_desired = y2 - bprime
    theta = Utilities.rad_to_deg(theta)

    return (x_desired, y_desired, theta)


def set_up_kick_facing_goal(ball, distance_from_center_of_goal):
    """Set Up kick

    puts the robot just behind the ball ready to shoot at the goal
    distance from center is for shooting in corners, 1 will put it in the "top" corner
    while -1 will put it in the bottom of the goal and 0 is exact center"""

    y2 = distance_from_center_of_goal * Constants.goal_box_width/2
    x2 = Constants.goal_position_opp[0]

    theta = Utilities.get_angle_between_points(ball['xhat'], ball['yhat'], x2,y2)
    cprime = _distance_behind_ball_for_kick

    # aprime is the length of the simalar triangle with hypotenuse cprime
    aprime = cprime*np.cos(theta)
    # bprime is the height of the simalar triangle with hypotenuse cprime
    bprime = cprime*np.sin(theta)

    x_c = ball['xhat'] - aprime
    y_c = ball['yhat'] - bprime
    theta_c = Utilities.rad_to_deg(theta)

    return (x_c, y_c, theta_c)

def attack_ball(robot, ball):
    a,b,c,theta = Utilities.find_triangle(robot['xhat'], robot['yhat'], ball['xhat'], ball['yhat'])
    x_c = _distance_behind_ball_approach * np.cos(theta*np.pi/180) + a + ball['xhat']
    y_c = _distance_behind_ball_approach * np.sin(theta*np.pi/180) + b + ball['yhat']
    theta = Utilities.rad_to_deg(theta)
    
    return (x_c, y_c, theta)


def defend_goal_in_arc(ball):
    c = Utilities.get_distance_between_points(Constants.goal_position_home[0], Constants.goal_position_home[1], ball['xhat_future'], ball['yhat_future'])
    distance = _distance_from_goal_for_arc_defense/c

    return stay_between_points_at_distance(Constants.goal_position_home[0], Constants.goal_position_home[1], ball['xhat_future'], ball['yhat_future'], distance)

# def dribble_ball_towards_point(robot, opponent, ball, point_x, point_y):
    # if _close([robot['xhat'], robot['yhat']], [ball['xhat'], ball['yhat']], 0.1) 


def go_behind_ball_facing_target(robot, opponent, ball, des_dist, target_x, target_y):
    theta = Utilities.get_angle_between_points(ball['xhat'], ball['yhat'], target_x, target_y)
    
    hypotenuse = Constants.robot_half_width + des_dist
    x_c = ball['xhat'] - hypotenuse*np.cos(theta)
    y_c = ball['yhat'] - hypotenuse*np.sin(theta)
    theta = Utilities.rad_to_deg(theta)
    return (x_c, y_c, theta)

def push_ball_des_distance (robot, ball, distance):
    hypotenuse = distance
    theta_c = robot['thetahat']
    x_c = ball['xhat'] + hypotenuse*np.cos(theta_c)
    y_c = ball['yhat'] + hypotenuse*np.sin(theta_c)

    return (x_c, y_c, theta_c)



