import numpy as np

import Plays
import Skills
import Utilities
import Constants

_offensive = 0
_defensive = 1
_neutral = 2

_ball_defend_position = None

_recently_kicked            = False
_kicker_wait_counter        = 0
_KICKER_WAIT_MAX            = 75 # 750 ms in between kicks. (3/4 of a second)

###################
# ATTACKER ROLES: #
###################
def offensive_attacker(me, my_teammate, opponent1, opponent2, ball, one_v_one=False):
    global _offensive
    # Ideas for this Role:
    # check to see who has possession first!
    # Always try and shoot the ball right at the goal. 
    return attacker(me, my_teammate, opponent1, opponent2, ball, _offensive, one_v_one)

def defensive_attacker(me, my_teammate, opponent1, opponent2, ball):
    global _defensive

    # Ideas for this Role:
    # check to see who has possession first!
     # If there is a clear shot at the goal, then shoot
      # otherwise, if me.ally2, then try and pass it to the teammate
      # Or if me.ally1, then try and shoot it off the walls
    return attacker(me, my_teammate, opponent1, opponent2, ball, _defensive)

def neutral_attacker(me, my_teammate, opponent1, opponent2, ball):
    global _neutral
    return attacker(me, my_teammate, opponent1, opponent2, ball, _neutral)



###################
# DEFENDER ROLES: #
###################
def offensive_defender(me, my_teammate, opponent1, opponent2, ball, one_v_one=False):
    global _offensive
    # If me.ally2, then try and pass it to ally1 (not kicking, but )
    return defender(me, my_teammate, opponent1, opponent2, ball, _offensive, one_v_one)

def defensive_defender(me, my_teammate, opponent1, opponent2, ball):
    global _defensive
    return defender(me, my_teammate, opponent1, opponent2, ball, _defensive)


def neutral_defender(me, my_teammate, opponent1, opponent2, ball):
    global _neutral
    return defender(me, my_teammate, opponent1, opponent2, ball, _neutral)




###################
# GOALIE ROLES:   #
###################
def offensive_goalie(me, my_teammate, opponent1, opponent2, ball, one_v_one=False):
    global _offensive
    return goalie(me, my_teammate, opponent1, opponent2, ball, _offensive, one_v_one)

def defensive_goalie(me, my_teammate, opponent1, opponent2, ball):
    global _defensive
    return goalie(me, my_teammate, opponent1, opponent2, ball, _defensive)

def neutral_goalie(me, my_teammate, opponent1, opponent2, ball):
    global _neutral
    return goalie(me, my_teammate, opponent1, opponent2, ball, _neutral)



################################################################
# More general functions, that implement the specific strategy #
################################################################

def attacker(me, my_teammate, opponent1, opponent2, ball, strategy, one_v_one=False):
    global _offensive, _defensive, _neutral
    middle_of_goal = 0

    goal_target = 0.6

    if Utilities.am_i_closest_teammate_to_ball(me, my_teammate, ball):
        if Utilities.am_i_closer_to_ball_than_opponents(me, opponent1, opponent2, ball):
            if Utilities.is_ball_behind_robot(me, ball):
                return Skills.avoid_own_goal(me, ball)

            if me.ally1 or one_v_one:
                if ball.yhat > 0: 
                    return Plays.shoot_on_goal(me, ball, goal_target, opponent1, opponent2)
                else: 
                    return Plays.shoot_on_goal(me, ball, -goal_target, opponent1, opponent2)
            else: # I am ally2
                if strategy == _offensive:
                    if (opponent1.xhat < me.xhat and opponent2.xhat < me.xhat):
                        return Plays.shoot_on_goal(me, ball, middle_of_goal, opponent1, opponent2)
                    else:
                        # return Plays.shoot_on_goal(me, ball, middle_of_goal, opponent1, opponent2)
                        return Plays.pass_to_teammate(me, my_teammate, ball) # passes to teammate_future_position
                else: #Defensive
                    if ball.yhat > 0: 
                        return Plays.shoot_on_goal(me, ball, goal_target, opponent1, opponent2)
                    else: 
                        return Plays.shoot_on_goal(me, ball, -goal_target, opponent1, opponent2)

        else: #Basically, we don't have possession and I should be the one to steal the ball
            if Utilities.is_ball_behind_robot(me, ball):
                return Skills.avoid_own_goal(me, ball)
            else:
                closest_opp = Utilities.get_closest_opponent_to_ball(opponent1, opponent2, ball)
                return Plays.steal_ball_from_opponent(me, closest_opp, ball)
    else: #My teammate is closer to the ball than me.
        if Utilities.am_i_too_close_to_teammate(me, my_teammate):
            return Skills.give_my_teammate_some_space(me, my_teammate)
        elif Utilities.is_ball_behind_robot(me, ball):
            return Skills.avoid_own_goal(me, ball)
        else:
            # offensive only if we are behind in points 
            if strategy == _offensive:
                if me.ally1:
                    return Plays.stay_open_for_pass(me, my_teammate, ball)
                else: # I am ally2 
                    return Plays.stay_at_midfield_follow_ball(me, opponent1, opponent2, ball)
            # The secondary robot will always stay at midfield
            else:
                return Plays.stay_at_midfield_follow_ball(me, opponent1, opponent2, ball)



def defender(me, my_teammate, opponent1, opponent2, ball, strategy, one_v_one=False):
    global _offensive, _defensive, _neutral
    
    if strategy == _offensive:
        dist_to_maintain = 0.85
    elif strategy == _defensive:
        dist_to_maintain = 0.65
    else:
        dist_to_maintain = 0.25

    if strategy == _defensive:
        return Plays.stay_open_for_pass(me, my_teammate, ball) 


    if Utilities.am_i_closest_teammate_to_ball(me, my_teammate, ball):
        if Utilities.am_i_closer_to_ball_than_opponents(me, opponent1, opponent2, ball):
            if Utilities.is_ball_behind_robot(me, ball):
                return Skills.avoid_own_goal(me, ball)
            else:
                return Skills.clear_ball_from_half(me, ball)
        else: # I am in charge of stealing the ball
            if Utilities.is_ball_behind_robot(me, ball):
                return Skills.avoid_own_goal(me, ball)
            else:
                closest_opp = Utilities.get_closest_opponent_to_ball(opponent1, opponent2, ball)
                return Plays.steal_ball_from_opponent(me, closest_opp, ball)
    ## My teammate is closer to the ball
    else: 
        if Utilities.am_i_too_close_to_teammate(me, my_teammate):
            return Skills.give_my_teammate_some_space(me, my_teammate)
        else:
            if Utilities.is_ball_behind_robot(me, ball):
                return Skills.avoid_own_goal(me, ball)
            elif strategy == _offensive:
                if me.ally1:
                    # call Utilities.are_both_opponents_attacking_goal?
                    return Plays.stay_open_for_pass(me, my_teammate, ball)
                else: # ally2
                    return Plays.stay_between_points_at_distance(Constants.goal_position_home[0], Constants.goal_position_home[1], ball.xhat, ball.yhat, dist_to_maintain)
            else: # strategy is defensive
                return Plays.stay_between_points_at_distance(Constants.goal_position_home[0], Constants.goal_position_home[1], ball.xhat, ball.yhat, dist_to_maintain)



def goalie(me, my_teammate, opponent1, opponent2, ball, strategy, one_v_one=False):
    global _ball_defend_position, _offensive, _defensive, _neutral
    global _recently_kicked, _kicker_wait_counter, _KICKER_WAIT_MAX

    # This prevents the goalie from kicking the ball too often
    _kicker_wait_counter = _kicker_wait_counter + 1
    if _kicker_wait_counter >= _KICKER_WAIT_MAX:
        _recently_kicked = False
        _kicker_wait_counter = 0

    # First, check to see if the ball is close enough to actuate the kicker, and kick it away
    (x_pos, y_pos) = Utilities.get_front_of_robot(me)
    distance_from_kicker_to_ball = Utilities.get_distance_between_points(x_pos, y_pos, ball.xhat, ball.yhat)
    
    if (distance_from_kicker_to_ball <=  Constants.kickable_distance and not Utilities.is_ball_behind_robot(me, ball)):
        if not _recently_kicked:
            print "GOALIE KICKING BALL AWAY"
            Skills.kick()
            _recently_kicked = True

    if Utilities.is_ball_behind_robot(me, ball):
        return Skills.avoid_own_goal(me, ball)
    elif strategy == _offensive and Utilities.am_i_closest_teammate_to_ball(me, my_teammate, ball) and Utilities.am_i_closer_to_ball_than_opponents(me, opponent1, opponent2, ball):
        return Skills.attack_ball(me, ball)
    # Goalie is always defending the goal in an arc
    else:
        theta_c = Utilities.get_angle_between_points(Constants.goal_position_home[0], Constants.goal_position_home[1], ball.xhat_future, ball.yhat_future)
        # theta_c_future = Utilities.get_angle_between_points(Constants.goal_position_home[0], Constants.goal_position_home[1], ball.xhat_future, ball.yhat_future)
        theta_c_deg = Utilities.rad_to_deg(theta_c)

        x_c = Constants.goal_position_home[0] + Constants.goalie_radius*np.cos(theta_c)
        y_c = Constants.goal_position_home[1] + Constants.goalie_radius*np.sin(theta_c)

        #Updating global variable that will predict where the ball will intersect with the "goalie arc" and maintain that position
        if abs(ball.xhat_future) > abs(x_c):
            if _ball_defend_position is None:
                _ball_defend_position = ball
        else:
            _ball_defend_position = None

        if _ball_defend_position is not None:
            y_c = _ball_defend_position.yhat_future

        return (x_c, y_c, theta_c_deg)




