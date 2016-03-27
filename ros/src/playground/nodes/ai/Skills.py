import time
import numpy as np

import rospy
from std_srvs.srv import Trigger

import Utilities
import Constants

##########################################
# Skills mainly for "attacker" position: #
##########################################
def kick():
    """Kick

    Send a service call to kick the ball.
    """
    try:
        kick_srv = rospy.ServiceProxy('kick', Trigger)
        kick_srv()
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


def attack_ball_with_kick(me, ball):
    dist_to_ball = Utilities.get_distance_between_points(me.xhat, me.yhat, ball.xhat, ball.yhat)
    if dist_to_ball < Constants.kickable_distance:
        kick()
    return attack_ball(me, ball)


def attack_ball(robot, ball):
    """
    Simply pushes the ball along the "vector" from robot to ball
    """
    theta = Utilities.get_angle_between_points(robot.xhat, robot.yhat, ball.xhat, ball.yhat)
    x_c = ball.xhat + Constants.kick_dist*np.cos(theta)
    y_c = ball.yhat + Constants.kick_dist*np.sin(theta)
    theta = Utilities.rad_to_deg(theta)
    
    return (x_c, y_c, theta)


##########################################
# Skills mainly for "defender" position: #
##########################################



def avoid_own_goal():
    pass


##########################################
# Skills mainly for "goalie" position:   #
##########################################
