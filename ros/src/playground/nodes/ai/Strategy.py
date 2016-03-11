from collections import Iterable

import numpy as np

import Skills
import Plays
import Utilities
import Constants

_fourth_field_length = _field_length/4

_ball_defend_position = None

_done = False

_going_home = False
_wait_timer = 0
  
def choose_strategy(robot, opponent, ball, goal):
    # Ultimate goal is to keep track of the opponent's movement/strategy, and alter ours accordingly

    global _going_home, _wait_timer
    if goal:
        print "***** GOAL!! ****** (maybe this is sad...)"
        _going_home = True
        _wait_timer = 0

    if _going_home:
        if _wait_timer < 500: # 7 seconds
            _wait_timer = _wait_timer + 1
            return Constants.goal_position_home
        else:
            print "ready to start"
            _going_home = False

    # if ball['xhat_future'] < Constants.goal_position_home[0] + _field_length/4:
    #	return _strong_defense(robot, ball)
    # else:
    	# return _strong_offense(robot, ball)
    # return _aggressive_defense(robot, ball)
    # return Skills.set_up_kick(ball, 0)
    # return Skills.defend_goal_in_arc(ball)
    return _aggressive_offense(robot, opponent, ball)
    #return Plays.shoot(robot, ball,0)




def _strong_offense(robot, opponent, ball):

    return _hack_offense(robot, ball)

    # for now we want to make one robot kick the ball into the open goal
    #
    # arctan2([y], [x])
    #theta_ball_to_goal      = Utilities.get_angle_between_points(ball['xhat'], ball['yhat'], Constants.goal_position_opp[0] ], Constants.goal_position_opp[1])
    #theta_ball_to_goal_deg  = theta_ball_to_goal*180/np.pi
    ##theta_bot_to_goal       = Utilities.get_angle_between_points(robot['xhat'], robot['yhat'], Constants.goal_position_opp[0], Constants.goal_position_opp[1])
    #theta_bot_to_goal_deg   = theta_bot_to_goal*180/np.pi
    #dist_from_ball          = Utilities.get_distance_between_points(robot['xhat'], robot['yhat'], ball['xhat'], ball['yhat'])

    #print("theta_ball_to_goal: {}\t\ttheta_bot_to_goal: {}\r".format(theta_ball_to_goal_deg, theta_bot_to_goal_deg))

    # if ball is past goal (primitive goal scored)
    #if _goal_scored:
     #   return  (-(_field_length/4), 0, 0)


    #if (ball['xhat'] > Constants.goal_position_opp[0]): #might have to tweak this a little bit
    #    #ball is in goal, don't move robot anywhere
    #    return (robot['xhat'], robot['yhat'], robot['thetahat'])
    #else:
    #   # if robot is behind the ball and aligned towards goal
    #    if ( Utilities.close(theta_ball_to_goal_deg, theta_bot_to_goal_deg) and \
    #        dist_from_ball <= (_des_dist_from_ball+_robot_half_width)): #taking into account 1/2 robot width
    #        #kick ball towards goal 6 inches
    #        x_c = ball['xhat'] + (_des_dist_from_ball+_kick_dist)*np.cos(theta_ball_to_goal)
    #        y_c = ball['yhat'] + (_des_dist_from_ball+_kick_dist)*np.sin(theta_ball_to_goal)
    #        return (x_c, y_c, theta_ball_to_goal_deg)
    #    else: 
    #        #get aligned with ball facing goal
    #        x_c = ball['xhat'] - (_des_dist_from_ball+_robot_half_width)*np.cos(theta_ball_to_goal)
    #        y_c = ball['yhat'] - (_des_dist_from_ball+_robot_half_width)*np.sin(theta_ball_to_goal)
    #        return (x_c, y_c, theta_ball_to_goal_deg) 

def _aggressive_offense(robot, opponent, ball):
    
    section = Utilities.get_field_section(ball['xhat'])
    future_section = Utilities.get_field_section(ball['xhat_future'])

    allytoball = Utilities.get_distance_between_points(robot['xhat'], robot['yhat'], ball['xhat'], ball['yhat'])
    opptoball  = Utilities.get_distance_between_points(opponent['xhat'], opponent['yhat'], ball['xhat'], ball['yhat'])

    if   section == 1:
        if (allytoball < opptoball and robot['xhat'] < ball['xhat']):
            return Skills.attack_ball(robot, ball)
        else:
            return _strong_defense(robot, ball)
    elif section == 2:
        if (allytoball < opptoball and robot['xhat'] < ball['xhat']):
            return Skills.attack_ball(robot, ball)
        else:
            dist_to_maintain = 0.70
            return Skills.stay_between_points_at_distance(Constants.goal_position_home[0], Constants.goal_position_home[1], ball['xhat_future'], ball['yhat_future'], dist_to_maintain)
    elif section == 3:
        if ball['yhat'] < 0:
            return Plays.shoot(robot, ball, -0.75)
        else:
            return Plays.shoot(robot, ball, 0.75)
    else: #section is 4
        if ball['yhat'] < 0:
            return Plays.shoot(robot, ball, -0.75)
        else:
            return Plays.shoot(robot, ball, 0.75)
    return (robot['xhat'], robot['yhat'], robot['thetahat'])



def _strong_defense(robot, ball):
    global _ball_defend_position

    #if _goal_scored:
     #   return  (-(_field_length/4), 0, 0)

    #for now we want to make one robot defend the goal
    theta_c = Utilities.get_angle_between_points(Constants.goal_position_home[0], Constants.goal_position_home[1], ball['xhat_future'], ball['yhat_future'])
    theta_c_deg = Utilities.rad_to_deg(theta_c)
    x_c =  Constants.goalie_x_pos

    # defends at yhat future
    y_c = ball['yhat_future']

    if abs(ball['xhat_future']) > abs(x_c):
        if _ball_defend_position is None:
            _ball_defend_position = ball
    else:
        _ball_defend_position = None

    if _ball_defend_position is not None:
        y_c = _ball_defend_position['yhat_future']

    y_c = _limit_goalie_y(y_c, ball)

    #Constants.goal_position_home[1] + (_goal_box_length+_robot_half_width)*np.sin(theta_c)
    theta_c_deg = 0
    return (x_c, y_c, theta_c_deg)
    
def _aggressive_defense(robot, ball):

    x_c, y_c, theta_c = Skills.stay_between_points_at_distance(Constants.goal_position_home[0], 0, ball['xhat_future'], ball['yhat_future'], 2.0/3)
    return (x_c, y_c, theta_c)


def _goal_scored(robot, ball):
    if ball['xhat'] > Constants.goal_position_opp[0] or ball['xhat'] < Constants.goal_position_home[0]:
        return True
    else:
        return False


def _limit_goalie_y(y_c, ball):
    # keeps robot in goal
    if (ball['yhat_future'] > _goal_box_width/2):
        y_c = _goal_box_width/2
    elif (ball['yhat_future'] < -_goal_box_width/2):
        y_c = -_goal_box_width/2

    return y_c

# limits robot to not hit walls
def _keep_inside_field(x_c, y_c):
    
    if x_c > Constants.goal_position_opp[0]:
        x_c = -_goalie_x_pos
    elif x_c < Constants.goal_position_home[0]:
        x_c = Constants.goal_position_home

    if y_c > Constants.field_width/2:
        y_c = Constants.field_width/2 - Constants.robot_width
    elif y_c < -Constants.field_width/2:
        y_c = -Constants.field_width/2 + Constants.robot_width

    return (x_c, y_cConstants