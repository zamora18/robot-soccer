import os
import numpy as np

_field_length       = 3.68 # (12ft)
_field_width        = 2.62 # (8.58 ft)
_robot_width        = 0.1841 # (7.25 in)
_goal_box_width     = 0.660 # (26 in)
_goal_position_home = -_field_length/2
_goal_position_opp  = -_goal_position_home


_distance_behind_ball_for_kick 		= _robot_width + .03 # this is for the jersey being off center
_distance_behind_ball_for_dribbling = _robot_width/2 + .05
_distance_from_goal_for_arc_defense = _goal_box_width + _robot_width *2
_distance_behind_ball_approach = .3

_kicker_count = 0

# actuates solenoid
def kick():
    global _kicker_count
    _kicker_count = _kicker_count + 1
    print("Actuate: {}".format(_kicker_count))
    os.system("echo 1 > /sys/class/gpio/gpio200/value; sleep .1; echo 0 > /sys/class/gpio/gpio200/value")


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

# distance should be between 0 and 1, scaled from the first point
# example follow ball 2/3 distance between goal and ball
def stay_between_points_at_distance(x1, y1, x2, y2, distance):

    a,b,c,theta = find_triangle(x1,y1,x2,y2)

    cprime = c*(1-distance)

    # aprime is the length of the simalar triangle with hypotenuse cprime
    aprime = cprime*np.cos(theta)
    # bprime is the height of the simalar triangle with hypotenuse cprime
    bprime = cprime*np.sin(theta)

    x_desired = x2 - aprime
    y_desired = y2 - bprime

    theta = theta*180/np.pi

    return (x_desired, y_desired, theta)


def set_up_kick_facing_goal(ball, distance_from_center_of_goal):
    """Set Up kick

    puts the robot just behind the ball ready to shoot at the goal
    distance from center is for shooting in corners, 1 will put it in the "top" corner
    while -1 will put it in the bottom of the goal and 0 is exact center"""



    y2 = distance_from_center_of_goal * _goal_box_width/2
    x2 = _goal_position_opp

    a,b,c,theta = find_triangle(ball['xhat'], ball['yhat'], x2,y2)

    cprime = _distance_behind_ball_for_kick

    # aprime is the length of the simalar triangle with hypotenuse cprime
    aprime = cprime*np.cos(theta)
    # bprime is the height of the simalar triangle with hypotenuse cprime
    bprime = cprime*np.sin(theta)

    x_c = ball['xhat'] - aprime
    y_c = ball['yhat'] - bprime

    theta_c = theta *180/np.pi

    return (x_c, y_c, theta_c)

def attack_ball(robot, ball):
    a,b,c,theta = find_triangle(robot['xhat'], robot['yhat'], ball['xhat'], ball['yhat'])
    x_c = _distance_behind_ball_approach * np.cos(theta*np.pi/180) + a + ball['xhat']
    y_c = _distance_behind_ball_approach * np.sin(theta*np.pi/180) + b + ball['yhat']

    return (x_c, y_c, theta)


def defend_goal_in_arc(ball):
    a,b,c,theta = find_triangle(_goal_position_home, 0, ball['xhat_future'], ball['yhat_future'])
    distance = _distance_from_goal_for_arc_defense/c

    return stay_between_points_at_distance(_goal_position_home, 0, ball['xhat_future'], ball['yhat_future'], distance)

# def dribble_ball_towards_point(robot, opponent, ball, point_x, point_y):
	# if _close([robot['xhat'], robot['yhat']], [ball['xhat'], ball['yhat']], 0.1) 




def go_behind_ball_facing_target(robot, opponent, ball, des_dist, target_x, target_y):
	[a,b,c,theta] = find_triangle(ball['xhat'], ball['yhat'], target_x, target_y)
	
	hypotenuse = _robot_width/2 + des_dist
	x_c = ball['xhat'] - hypotenuse*np.cos(theta)
	y_c = ball['yhat'] - hypotenuse*np.sin(theta)

	return (x_c, y_c, theta)

def push_ball_des_distance (robot, ball, distance):
	hypotenuse = distance
	theta_c = robot['thetahat']
	x_c = ball['xhat'] + hypotenuse*np.cos(theta_c)
	y_c = ball['yhat'] + hypotenuse*np.sin(theta_c)

	return (x_c, y_c, theta_c)


def _close(a, b, tolerance=0.010):
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
