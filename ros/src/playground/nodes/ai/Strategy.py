from collections import Iterable

import numpy as np
import Skills
import Plays

# field constants. Distances measured in meters
_field_length       = 3.68 # (12ft)
_field_width        = 2.62 # (8.58 ft)
_robot_width        = 0.1841 # (7.25 in)
_robot_half_width   = _robot_width/2
_goal_box_width     = 0.660 # (26 in)
_goal_box_length    = 0.1143 #(4.5 in)
_goal_position_home = [-_field_length/2, 0, 0] #this could change depending on camera
_goal_position_opp  = [-_goal_position_home[0], 0, 0]
_des_dist_from_ball = 0.0762 #(3.0in)
_kick_dist          = 0.1524 #(6.0in)
_goalie_x_pos       = _goal_position_home[0] + _goal_box_length + _robot_half_width

_fourth_field_length = _field_length/4
_half_field          = 0

_ball_defend_position = None

_done = False
  
def choose_strategy(robot, opponent, ball):
    # if ball['xhat_future'] < _goal_position_home[0] + _field_length/4:
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
    theta_ball_to_goal      = np.arctan2([ ball['yhat'] - _goal_position_opp[1] ], [ _goal_position_opp[0] - ball['xhat'] ])
    theta_ball_to_goal_deg  = theta_ball_to_goal*180/np.pi
    theta_bot_to_goal       = np.arctan2([ robot['yhat'] - _goal_position_opp[1] ], [ _goal_position_opp[0] - robot['xhat'] ])
    theta_bot_to_goal_deg   = theta_bot_to_goal*180/np.pi
    dist_from_ball          = _get_distance(robot, ball)

    #print("theta_ball_to_goal: {}\t\ttheta_bot_to_goal: {}\r".format(theta_ball_to_goal_deg, theta_bot_to_goal_deg))

    # if ball is past goal (primitive goal scored)
    #if _goal_scored:
     #   return  (-(_field_length/4), 0, 0)


    if (ball['xhat'] > _goal_position_opp[0]): #might have to tweak this a little bit
        #ball is in goal, don't move robot anywhere
        return (robot['xhat'], robot['yhat'], robot['thetahat'])
    else:
        # if robot is behind the ball and aligned towards goal
        if ( _close(theta_ball_to_goal_deg, theta_bot_to_goal_deg) and \
             #_close(theta_ball_to_goal_deg, robot['thetahat']) and \
            dist_from_ball <= (_des_dist_from_ball+_robot_half_width)): #taking into account 1/2 robot width
            #kick ball towards goal 6 inches
            x_c = ball['xhat'] + (_des_dist_from_ball+_kick_dist)*np.cos(theta_ball_to_goal)
            y_c = ball['yhat'] + (_des_dist_from_ball+_kick_dist)*np.sin(theta_ball_to_goal)
            return (x_c, y_c, theta_ball_to_goal_deg)
        else: 
            #get aligned with ball facing goal
            x_c = ball['xhat'] - (_des_dist_from_ball+_robot_half_width)*np.cos(theta_ball_to_goal)
            y_c = ball['yhat'] - (_des_dist_from_ball+_robot_half_width)*np.sin(theta_ball_to_goal)
            return (x_c, y_c, theta_ball_to_goal_deg) 

def _aggressive_offense(robot, opponent, ball):
    
    section = _get_field_section(ball['xhat'])
    future_section = _get_field_section(ball['xhat_future'])

    if   section == 1:
        if _close([robot['xhat'], robot['yhat']], [ball['xhat'], ball['yhat']], 0.1) and not _close([robot['xhat'], robot['yhat']], [opponent['xhat'], opponent['yhat']], 0.35):
            #kick the ball towards the goal
            Skills.kick()
        else:
            Skills.defend_goal_in_arc(ball)
    elif section == 2:
        if _close([robot['xhat'], robot['yhat']], [ball['xhat'], ball['yhat']], 0.1) and not _close([robot['xhat'], robot['yhat']], [opponent['xhat'], opponent['yhat']], 0.25):
            #kick the ball towards the goal
            Skills.kick()
        else:
            return Skills.stay_between_points_at_distance(ball['xhat_future'], ball['yhat_future'], _goal_position_home[0], _goal_position_home[1], 0.20)
    elif section == 3:
        if ball['yhat'] < 0:
            Plays.shoot(robot, ball, -0.75)
        else:
            Plays.shoot(robot, ball, 0.75)
    else: #section is 4
        if ball['yhat'] < 0:
            Plays.shoot(robot, ball, -0.75)
        else:
            Plays.shoot(robot, ball, 0.75)

def _hack_offense(robot, ball):

    STOP_THRESH = 1.75 # m

    if robot['xhat'] > STOP_THRESH:
        return (0, 0, robot['thetahat'])

    theta_ball_to_goal      = np.arctan2([ ball['yhat'] - _goal_position_opp[1] ], [ _goal_position_opp[0] - ball['xhat'] ])

    robot_width = 0.1841
    offset_behind_ball = 2.5*robot_width

    global _done

    x = ball['xhat'] - offset_behind_ball*np.cos(theta_ball_to_goal)
    y = ball['yhat'] - offset_behind_ball*np.sin(-2*theta_ball_to_goal)

    if not _close(robot['xhat'], x, tolerance=0.100) or not _close(robot['yhat'], y, tolerance=0.100) and not _done:
        _done = True
        return (x, y, robot['thetahat'])

    # kick
    kick_point = (STOP_THRESH+.500, 0, robot['thetahat'])
    return kick_point





def _strong_defense(robot, ball):
    global _ball_defend_position

    #if _goal_scored:
     #   return  (-(_field_length/4), 0, 0)

    #for now we want to make one robot defend the goal
    theta_c = np.arctan2([ _goal_position_home[1] + ball['yhat_future'] ], [ ball['xhat_future'] - _goal_position_opp[0] ])
    theta_c_deg = theta_c*180/np.pi
    x_c =  _goalie_x_pos#_goal_position_home[0] + (_goal_box_length+_robot_half_width)*np.cos(theta_c)

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

    #_goal_position_home[1] + (_goal_box_length+_robot_half_width)*np.sin(theta_c)
    theta_c_deg = 0
    return (x_c, y_c, theta_c_deg)
    
def _aggressive_defense(robot, ball):

    x_c, y_c, theta_c = Skills.stay_between_points_at_distance(_goal_position_home[0], 0, ball['xhat_future'], ball['yhat_future'], 2.0/3)

    return (x_c, y_c, theta_c)





def _get_distance(object_1, object_2):
    x_dist = object_1['xhat'] - object_2['xhat']
    y_dist = object_1['yhat'] - object_2['yhat']
    distance = np.sqrt(x_dist**2 + y_dist**2)
    return distance

def _get_field_section(x_pos):
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


def _goal_scored(robot, ball):
    if ball['xhat'] > _goal_position_opp[0] or ball['xhat'] < _goal_position_home[0]:
        return True
    else:
        return False

def _close(a, b, tolerance):
    return abs(a - b) <= tolerance



def _limit_goalie_y(y_c, ball):
    # keeps robot in goal
    if (ball['yhat_future'] > _goal_box_width/2):
        y_c = _goal_box_width/2
    elif (ball['yhat_future'] < -_goal_box_width/2):
        y_c = -_goal_box_width/2

    return y_c

# limits robot to not hit walls
def _keep_inside_field(x_c, y_c):
    
    if x_c > _goal_position_opp[0]:
        x_c = -_goalie_x_pos
    elif x_c < _goal_position_home[0]:
        x_c = _goal_position_home

    if y_c > _field_width/2:
        y_c = _field_width/2 - _robot_width
    elif y_c < -_field_width/2:
        y_c = -_field_width/2 + _robot_width

    return (x_c, y_c)

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