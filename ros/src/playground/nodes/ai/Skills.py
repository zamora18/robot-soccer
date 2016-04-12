import time
import numpy as np

import rospy
from std_srvs.srv import Trigger

import Utilities
import Constants

class ClearBallState:
    setup   = 0
    clear  = 1
    kick   = 2

class OwnGoalState:
    perp_setup      = 0
    behind_setup    = 1
    attack          = 2
    kick            = 3

_clear_ball_st  = ClearBallState.setup
_own_goal_st    = OwnGoalState.perp_setup
_went_to_perp_first = False # This avoids the state machine starting in the 'behind_setup' and ruining the whole avoid own goal function

_kick_num = 0

#################################################
# General Skills potentially used by everyone:  #
#################################################
def get_unstuck(me):
    dist_to_retreat = 0.20
    theta = Utilities.get_angle_between_points(me.xhat, me.yhat, 0, 0) #move a little towards the center of the field.
    x_c = dist_to_retreat*np.cos(theta)
    y_c = dist_to_retreat*np.sin(theta)
    theta = Utilities.rad_to_deg(theta)
    return (x_c, y_c, theta)



##########################################
# Skills mainly for "attacker" position: #
##########################################
def kick():
    """Kick

    Send a service call to kick the ball.
    """
    global _kick_num
    try:
        kick_srv = rospy.ServiceProxy('kick', Trigger)
        kick_srv()
        _kick_num = _kick_num + 1
        print ("Kicking. Kick number: {}" .format(_kick_num))
    except rospy.ServiceException, e:
        print "Kick service call failed: %s"%e


def set_up_kick_facing_goal(ball, distance_from_center_of_goal):
    """Set Up kick

    puts the robot just behind the ball ready to shoot at the goal
    distance from center is for shooting in corners, 1 will put it in the "top" corner
    while -1 will put it in the bottom of the goal and 0 is exact center"""

    target_y = distance_from_center_of_goal * Constants.goal_box_width/2
    target_x = Constants.goal_position_opp[0]
    return go_behind_ball_facing_target(ball, Constants.distance_behind_ball_for_kick, target_x, target_y)


def go_behind_ball_facing_target(ball, des_distance_from_ball, target_x, target_y):
    theta = Utilities.get_angle_between_points(ball.xhat, ball.yhat, target_x, target_y)
    hypotenuse = Constants.robot_half_width + des_distance_from_ball
    x_c = ball.xhat - hypotenuse*np.cos(theta)
    y_c = ball.yhat - hypotenuse*np.sin(theta)
    theta = Utilities.rad_to_deg(theta)
    return (x_c, y_c, theta)



def attack_ball_towards_goal(me, ball, goal_y):
    target_y = goal_y*Constants.goal_box_width/2
    return attack_ball_towards_point(me, ball, Constants.goal_position_opp[0], target_y)


def attack_ball(me, ball):
    """
    Simply pushes the ball along the "vector" from robot to ball
    """
    theta = Utilities.get_angle_between_points(me.xhat, me.yhat, ball.xhat, ball.yhat)
    x_c = ball.xhat + Constants.kick_dist*np.cos(theta)
    y_c = ball.yhat + Constants.kick_dist*np.sin(theta)
    theta_c = Utilities.rad_to_deg(theta)

    return(x_c, y_c, theta_c)


def attack_ball_towards_point(me, ball, point_x, point_y):
    theta = Utilities.get_angle_between_points(ball.xhat, ball.yhat, point_x, point_y)
    x_c = ball.xhat + Constants.kick_dist*np.cos(theta)
    y_c = ball.yhat + Constants.kick_dist*np.sin(theta)
    theta_c = Utilities.rad_to_deg(theta)
    return(x_c, y_c, theta_c) 



##########################################
# Skills mainly for "defender" position: #
##########################################

def give_my_teammate_some_space(me, my_teammate):
    theta = Utilities.get_angle_between_points(me.xhat, me.yhat, my_teammate.xhat, my_teammate.yhat)
    x_c = my_teammate.xhat - (Constants.teammate_gap+0.05)*np.cos(theta)
    y_c = my_teammate.yhat - (Constants.teammate_gap+0.05)*np.cos(theta)
    theta_c = Utilities.rad_to_deg(theta)
    return (x_c, y_c, theta_c)


def clear_ball_from_half(me, ball):
    global _clear_ball_st
    center_of_goal = 0
    desired_setup = set_up_kick_facing_goal(ball, center_of_goal)

    (x_pos, y_pos) = Utilities.get_front_of_robot(me)
    distance_from_kicker_to_ball = Utilities.get_distance_between_points(x_pos, y_pos, ball.xhat, ball.yhat)



    #########################
    ### transition states ###
    #########################
    if _clear_ball_st == ClearBallState.setup:
        if Utilities.robot_close_to_point(me, *desired_setup):
            _clear_ball_st = ClearBallState.clear
    elif _clear_ball_st == ClearBallState.clear:
        if distance_from_kicker_to_ball <= Constants.kickable_distance:
            _clear_ball_st = ClearBallState.kick
        elif distance_from_kicker_to_ball > Constants.distance_behind_ball_for_kick:
            _clear_ball_st = ClearBallState.setup
    elif _clear_ball_st == ClearBallState.kick:
        if distance_from_kicker_to_ball > Constants.kickable_distance:
            _clear_ball_st = ClearBallState.setup
    else:
        _clear_ball_st = ClearBallState.setup


    ###############################
    ### Moore Outputs in states ###
    ###############################
    if _clear_ball_st == ClearBallState.setup:
        return desired_setup
    elif _clear_ball_st == ClearBallState.clear:
        return attack_ball(me, ball)
    elif _clear_ball_st == ClearBallState.kick:
        kick()
        return attack_ball(me, ball)
    else:
        return (desired_setup)



##########################################
# Skills mainly for "goalie" position:   #
##########################################
def avoid_own_goal(me, ball):
    """
    Robot will go to a point perpendicular away from the ball, so that it doesn't hit it into our goal.
    """

    desired_perp_setup = Utilities.get_perpendicular_point_from_ball(me, ball)

    return desired_perp_setup # THIS IS FREAKIN RIDICULOUS

