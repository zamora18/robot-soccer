import numpy as np

import Roles
import Plays
import Skills
import Utilities
import Constants

from GameObjects import Ball, Robot

#Variables for tracking opponent's strategy
_avg_dist_between_opponents         = 0
_averaging_factor                   = 0
_percent_time_ball_in_our_half      = 0
_percent_time_opponents_in_our_half = 0
_our_score                          = 0
_opponent_score                     = 0
_goal_check_counter                 = 0
_goal_counter_max                   = 10 # Is this a good number?

# If we decide to go for a trick shot at the beginning
_beginning_trick_shot = False

# ally1 is designated as the "main" attacker, or the robot closest to the opponent's goal at the beginning of the game
# ally2 is designated as the "main" defender, or the robot closest to our goal at the beginnning of the game

def choose_strategy(me, my_teammate, opponent1, opponent2, ball, goal, one_v_one=False):
    global _avg_dist_between_opponents, _averaging_factor, _percent_time_ball_in_our_half, _percent_time_opponents_in_our_half
    global _our_score, _opponent_score
    global _goal_check_counter, _goal_counter_max
    # update_opponents_strategy_variables(opponent1, opponent2, ball)
    # Check to see if someone scored a goal
    
    one_v_one = True

    #for now, we will just focus on aggressive offense
    # if goal:
    #     # MAKE A DEBOUNCED GOAL SO THAT WE DON'T THINK THERE'S A GOAL WHEN IT'S JUST "CLOSE" AND NOT ALL THE WAY IN
    #     return reset_positions_after_goal(me)
    if (one_v_one):
        (x,y,theta) = one_on_one(me, opponent1, ball)
        (x_c, y_c) = Utilities.limit_xy_too_close_to_walls(x,y)
        return (x_c, y_c, theta)
    else:
        (x,y,theta) = aggressive_offense(me, my_teammate, opponent1, opponent2, ball)
        (x_c, y_c) = Utilities.limit_xy_too_close_to_walls(x,y)
        return (x_c, y_c, theta)


def aggressive_offense(me, my_teammate, opponent1, opponent2, ball):
    global _beginning_trick_shot
    section = Utilities.get_field_section(ball.xhat)

    if me.ally1:
        # if not Plays.beginning_trick_shot_done():
        #     return Plays.shoot_off_the_wall(me, ball)
        if section == 1:
            return Roles.offensive_defender(me, my_teammate, opponent1, opponent2, ball)
        elif section == 2:
            return Roles.offensive_defender(me, my_teammate, opponent1, opponent2, ball)
        elif section == 3:
            return Roles.offensive_attacker(me, my_teammate, opponent1, opponent2, ball)
        elif section == 4:
            return Roles.offensive_attacker(me, my_teammate, opponent1, opponent2, ball)
        else:
            return (me.xhat, me.yhat, me.thetahat) #default, returns current pos
    else:
        if   section == 1:
            return Roles.offensive_goalie(me, my_teammate, opponent1, opponent2, ball)
        elif section == 2:
            return Roles.offensive_defender(me, my_teammate, opponent1, opponent2, ball)
        elif section == 3:
            return Roles.offensive_attacker(me, my_teammate, opponent1, opponent2, ball)
        elif section == 4:
            return Roles.offensive_attacker(me, my_teammate, opponent1, opponent2, ball)
        else:
            return (me.xhat, me.yhat, me.thetahat) #default, returns current pos


def aggressive_defense(me, my_teammate, opponent1, opponent2, ball):
    section = Utilities.get_field_section(ball.xhat)

    if me.ally1:
        if   section == 1:
            return Roles.defensive_defender(me, my_teammate, opponent1, opponent2, ball)
        elif section == 2:
            return Roles.defensive_defender(me, my_teammate, opponent1, opponent2, ball)
        elif section == 3:
            return Roles.defensive_attacker(me, my_teammate, opponent1, opponent2, ball)
        elif section == 4:
            return Roles.defensive_attacker(me, my_teammate, opponent1, opponent2, ball)
        else:
            return (me.xhat, me.yhat, me.thetahat) #default, returns current pos
    else:
        if   section == 1:
            return Roles.defensive_goalie(me, my_teammate, opponent1, opponent2, ball)
        elif section == 2:
            return Roles.defensive_goalie(me, my_teammate, opponent1, opponent2, ball)
        elif section == 3:
            return Roles.defensive_defender(me, my_teammate, opponent1, opponent2, ball)
        elif section == 4:
            return Roles.defensive_defender(me, my_teammate, opponent1, opponent2, ball)
        else:
            return (me.xhat, me.yhat, me.thetahat) #default, returns current pos


def passive_aggressive(me, my_teammate, opponent1, opponent2, ball): #AKA, mild offense/defense
    section = Utilities.get_field_section(ball.xhat)

    if me.ally1:
        if   section == 1:
            return Roles.neutral_defender(me, my_teammate, opponent1, opponent2, ball)
        elif section == 2:
            return Roles.neutral_defender(me, my_teammate, opponent1, opponent2, ball)
        elif section == 3:
            return Roles.neutral_attacker(me, my_teammate, opponent1, opponent2, ball)
        elif section == 4:
            return Roles.neutral_attacker(me, my_teammate, opponent1, opponent2, ball)
        else:
            return (me.xhat, me.yhat, me.thetahat) #default, returns current pos
    else:
        if   section == 1:
            return Roles.neutral_goalie(me, my_teammate, opponent1, opponent2, ball)
        elif section == 2:
            return Roles.neutral_defender(me, my_teammate, opponent1, opponent2, ball)
        elif section == 3:
            return Roles.neutral_defender(me, my_teammate, opponent1, opponent2, ball)
        elif section == 4:
            return Roles.neutral_attacker(me, my_teammate, opponent1, opponent2, ball)
        else:
            return (me.xhat, me.yhat, me.thetahat) #default, returns current pos


def one_on_one(me, opponent1, ball):
    global _beginning_trick_shot
    my_teammate = None
    opponent2 = None
    section = Utilities.get_field_section(ball.xhat)

    # if not _beginning_trick_shot:
    #     _beginning_trick_shot = True
    #     return Plays.shoot_off_the_wall(me, ball)
    if   section == 1:
        return Roles.offensive_goalie(me, my_teammate, opponent1, opponent2, ball, True)
    elif section == 2:
        return Roles.offensive_defender(me, my_teammate, opponent1, opponent2, ball, True)
    elif section == 3:
        return Roles.offensive_attacker(me, my_teammate, opponent1, opponent2, ball, True)
    elif section == 4:
        return Roles.offensive_attacker(me, my_teammate, opponent1, opponent2, ball, True)
    else:
        return (me.xhat, me.yhat, me.thetahat) #default, returns current pos

def check_for_goal(ball):
    pass
#     global
#     if ball.xhat < (Constants.goal_position_home[0]+Constants.goal_score_threshold):
#         _goal_check_counter = _goal_check_counter + 1
#         if _goal_check_counter >= _goal_counter_max:
#             _opponent_score = _opponent_score + 1
#             _goal_check_counter = 0
#             goal = True
#     elif ball.xhat > (Constants.goal_position_opp[0]+Constants.goal_score_threshold):
#         _goal_check_counter = _goal_check_counter + 1
#         if _goal_check_counter >= _goal_counter_max:
#             _our_score = _our_score + 1
#             _goal_check_counter = 0
#             goal = True

def update_opponents_strategy_variables(opponent1, opponent2, ball):
    global _avg_dist_between_opponents, _averaging_factor, _percent_time_ball_in_our_half, _percent_time_opponents_in_our_half
    _averaging_factor = _averaging_factor + 1

    new_dist_between_opponents = Utilities.get_distance_between_points(opponent1.xhat, opponent1.yhat, opponent2.xhat, opponent2.yhat)
    _avg_dist_between_opponents = (_avg_dist_between_opponents + new_dist_between_opponents)/_averaging_factor

    if Utilities.is_in_our_half(ball):
        _percent_time_ball_in_our_half = (_percent_time_ball_in_our_half + 1)/_averaging_factor
    else:
        _percent_time_ball_in_our_half = _percent_time_ball_in_our_half/_averaging_factor

    if Utilities.is_in_our_half(opponent1) and Utilities.is_in_our_half(opponent2):
        _percent_time_opponents_in_our_half = (_percent_time_opponents_in_our_half + 2)/_averaging_factor # if both players are on our side, then they are playing very offensively
    elif Utilities.is_in_our_half(opponent1):
        _percent_time_opponents_in_our_half = (_percent_time_opponents_in_our_half + 1)/_averaging_factor
    elif Utilities.is_in_our_half(opponent2):
        _percent_time_opponents_in_our_half = (_percent_time_opponents_in_our_half + 1)/_averaging_factor
    else:
        _percent_time_opponents_in_our_half = _percent_time_opponents_in_our_half/_averaging_factor



def reset_positions_after_goal(me):
    if me.ally1:
        return (Constants.ally1_start_pos[0], Constants.ally1_start_pos[1], Constants.ally1_start_pos[2])
    else:
        return (Constants.ally2_start_pos[0], Constants.ally2_start_pos[1], Constants.ally2_start_pos[2])
