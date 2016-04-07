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

# For detecting goal 
_is_goal_global = False
_goal_check_counter                 = 0
_GOAL_COUNTER_MAX                   = 2 # 10 for real life, 2 for simulator

_resume_game_counter                = 0
_RESUME_GAME_MAX                    = 500 # (5 seconds)

"""
THINGS TO CHANGE WHEN GOING FROM SIMULATOR TO REAL LIFE TESTING:
    - Strategy.py:      _GOAL_COUNTER_MAX
    - Constants.py:     goal_score_threshold

"""
 
"""
Notes of things I have changed that may need to be changed back:
    (April 7th)
    - Changed Constants.distance_behind_ball_for_kick = robot_half_width so the approach is better. With kicker, approach can be smaller
    - Changed Constants.kickable_distance = 0.4 from 0.5
    - Utilities.robot_close_to_point() Changed tolerances from 0.1 to 0.7

    Changed these to use BALL FUTURE POSITIONS:
    - Skills.py:        go_behind_ball_facing_target()
    -                   attack_ball()
    -                   attack_ball_towards_goal()
    - 

Things I'm currently/need to work on:
    - Using future positions of ball for attack/shooting
    - Collision avoidance with our own robots. Fix am_i_too_close_to_teammate to have robot future positions.
"""



# ally1 is designated as the "main" attacker, or the robot closest to the opponent's goal at the beginning of the game
# ally2 is designated as the "main" defender, or the robot closest to our goal at the beginnning of the game
def choose_strategy(me, my_teammate, opponent1, opponent2, ball, goal, one_v_one=False):
    global _avg_dist_between_opponents, _averaging_factor, _percent_time_ball_in_our_half, _percent_time_opponents_in_our_half
    global _our_score, _opponent_score
    global _is_goal_global
    update_opponents_strategy_variables(opponent1, opponent2, ball)
    
    one_v_one = True # FOR SIMULATOR I NEED TO UNCOMMENT THIS
    # Check to see if someone scored a goal
    check_for_goal(ball) # This has the goal debouncer in it, will update global variable _is_goal_global, and calls update_score()
    
    if _is_goal_global:
        if are_robots_in_reset_position(me, my_teammate):
            if done_waiting_for_resume_game():
                # Reset variables so that gameplay can continue
                _is_goal_global = False # this will allow the gameplay to restart again.
        return reset_positions_after_goal(me)

    else: # No goal, keep playing
        opp_strong_offense = (_percent_time_ball_in_our_half >= 0.50 and _avg_dist_between_opponents <=  1.5 )  
        #for now, we will just focus on aggressive offense
        if (one_v_one):
            (x,y,theta) = one_on_one(me, opponent1, ball)
            (x_c, y_c) = Utilities.limit_xy_too_close_to_walls(x,y)
            return (x, y, theta) # TOOK OUT X_C, Y_C
        else:
            (x,y,theta) = aggressive_offense(me, my_teammate, opponent1, opponent2, ball)
            (x_c, y_c) = Utilities.limit_xy_too_close_to_walls(x,y)
            return (x, y, theta) # TOOK OUT X_C, Y_C


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
            return Roles.offensive_goalie(me, my_teammate, opponent1, opponent2, ball) #This used to be offensive defender, but i want to see the goalie do it's thing
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
    my_teammate = None
    opponent2 = None
    section = Utilities.get_field_section(ball.xhat)
    # print ("section = {}" .format(section))
    # if Utilities.i_am_stuck(me):
    #     return Skills.get_unstuck(me)
    # else:
        # if not Plays.beginning_trick_shot_done():
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




################################################################
###                     Helper Functions                     ###
################################################################

def check_for_goal(ball):
    global _goal_check_counter, _GOAL_COUNTER_MAX, _is_goal_global
    # If someone just scored, then don't do anything
    if not _is_goal_global:
        far_enough_away_from_goal = 0.10
        if abs(ball.xhat) <= Constants.field_length/2 - far_enough_away_from_goal:
            # Reset Counter because ball is far enough away from the goal
            _goal_check_counter = 0 
        elif abs(ball.xhat) >= Constants.field_length/2 + Constants.goal_score_threshold:
            # Update counter
            _goal_check_counter = _goal_check_counter + 1
            if _goal_check_counter >= _GOAL_COUNTER_MAX:
                _goal_check_counter = 0
                _is_goal_global = True
                # Update the score here, so it only does it once
                update_score(ball)


def update_score(ball):
    global _our_score, _opponent_score
    if ball.xhat > 0:
        print "GOOOOOAAAAAAALLLLLLLAAAAASSSSSSSOOOOOOO!!!!"
        _our_score = _our_score + 1
    else:
        print "NOOOO, They scored =("
        _opponent_score = _opponent_score + 1 

    print("Score is now:\n\tUs: {} \n\tThem: {}".format(_our_score, _opponent_score))


def update_opponents_strategy_variables(opponent1, opponent2, ball):
    global _avg_dist_between_opponents, _averaging_factor, _percent_time_ball_in_our_half, _percent_time_opponents_in_our_half
    _averaging_factor = _averaging_factor + 1

    # Compute the distance between the opponents, if distance is large, then they have a goalie implemented. 
    new_dist_between_opponents = Utilities.get_distance_between_points(opponent1.xhat, opponent1.yhat, opponent2.xhat, opponent2.yhat)
    _avg_dist_between_opponents = (_avg_dist_between_opponents + new_dist_between_opponents)/_averaging_factor

    # Calculate the amount of time the ball is spent in our half
    if Utilities.is_in_our_half(ball):
        _percent_time_ball_in_our_half = (_percent_time_ball_in_our_half + 1)/_averaging_factor
    else:
        _percent_time_ball_in_our_half = _percent_time_ball_in_our_half/_averaging_factor

    # Then Update the amount of time the opponent(s) are in our half.
    if Utilities.is_in_our_half(opponent1) and Utilities.is_in_our_half(opponent2):
        _percent_time_opponents_in_our_half = (_percent_time_opponents_in_our_half + 2)/_averaging_factor # if both players are on our side, then they are playing very offensively
    elif Utilities.is_in_our_half(opponent1):
        _percent_time_opponents_in_our_half = (_percent_time_opponents_in_our_half + 1)/_averaging_factor
    elif Utilities.is_in_our_half(opponent2):
        _percent_time_opponents_in_our_half = (_percent_time_opponents_in_our_half + 1)/_averaging_factor
    else:
        _percent_time_opponents_in_our_half = _percent_time_opponents_in_our_half/_averaging_factor


def are_robots_in_reset_position(me, my_teammate):
    if my_teammate is None:
        return Utilities.robot_close_to_point(me, Constants.ally1_start_pos[0], Constants.ally1_start_pos[1], Constants.ally1_start_pos[2])
    else:
        if me.ally1:
            return Utilities.robot_close_to_point(me, Constants.ally1_start_pos[0], Constants.ally1_start_pos[1], Constants.ally1_start_pos[2]) and Utilities.robot_close_to_point(my_teammate, Constants.ally2_start_pos[0], Constants.ally2_start_pos[1], Constants.ally2_start_pos[2])
        else:
            return Utilities.robot_close_to_point(my_teammate, Constants.ally1_start_pos[0], Constants.ally1_start_pos[1], Constants.ally1_start_pos[2]) and Utilities.robot_close_to_point(me, Constants.ally2_start_pos[0], Constants.ally2_start_pos[1], Constants.ally2_start_pos[2])


def reset_positions_after_goal(me):
    if me.ally1:
        return (Constants.ally1_start_pos[0], Constants.ally1_start_pos[1], Constants.ally1_start_pos[2])
    else:
        return (Constants.ally2_start_pos[0], Constants.ally2_start_pos[1], Constants.ally2_start_pos[2])


def done_waiting_for_resume_game():
    global _resume_game_counter, _RESUME_GAME_MAX

    _resume_game_counter = _resume_game_counter + 1
    if _resume_game_counter >= _RESUME_GAME_MAX:
        _resume_game_counter = 0
        return True

    return False


