import numpy as np

import Plays
import Skills
import Utilities
import Constants

_offensive = 0
_defensive = 1
_neutral = 2

_ball_defend_position = None

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

    if strategy == _offensive:
        goal_target = 0.80
    elif strategy == _defensive:
        goal_target = 0.20
    else:
        goal_target = 0

    if Utilities.am_i_closest_teammate_to_ball(me, my_teammate, ball):
        if Utilities.am_i_closer_to_ball_than_opponents(me, opponent1, opponent2, ball):
            if me.ally1 or one_v_one:
                if ball.yhat > 0: 
                    return Plays.shoot_on_goal(me, ball, goal_target, opponent1, opponent2)
                else: 
                    return Plays.shoot_on_goal(me, ball, -goal_target, opponent1, opponent2)
            else: # I am ally2
                if (opponent1.xhat < me.xhat and opponent2.xhat < me.xhat):
                    return Plays.shoot_on_goal(me, ball, middle_of_goal, opponent1, opponent2)
                else:
                    return Plays.shoot_on_goal(me, ball, middle_of_goal, opponent1, opponent2)
                    # return Plays.pass_to_teammate(me, my_teammate, ball) #maybe he should just kick it directly in front of him?

        else: #Basically, we don't have possession and I should be the one to steal the ball
            closest_opp = Utilities.get_closest_opponent_to_ball(opponent1, opponent2, ball)
            return Plays.steal_ball_from_opponent(me, closest_opp, ball)
    else: #My teammate is closer to the ball than me.
        if Utilities.am_i_too_close_to_teammate(me, my_teammate):
            return Skills.give_my_teammate_some_space(me, my_teammate)
        else:
            if me.ally1:
                return Plays.stay_open_for_pass(me, my_teammate, ball)
            else: # I am ally2 
                field_section = Utilities.get_field_section(ball.xhat) 
                if field_section == 3:
                    return Plays.stay_at_midfield_follow_ball(me, opponent1, opponent2, ball)
                else:
                    return Plays.stay_at_front_quarter_follow_ball(me, opponent1, opponent2, ball)



def defender(me, my_teammate, opponent1, opponent2, ball, strategy, one_v_one=False):
    global _offensive, _defensive, _neutral
    
    if strategy == _offensive:
        if me.ally1:
            dist_to_maintain = 0.85
        else:
            dist_to_maintain = 0.40
    elif strategy == _defensive:
        if me.ally1:
            dist_to_maintain = 0.65
        else:
            dist_to_maintain = 0.20
    else:
        if me.ally1:
            dist_to_maintain = 0.55
        else:
            dist_to_maintain = 0.00

    if Utilities.am_i_closest_teammate_to_ball(me, my_teammate, ball):
        if Utilities.am_i_closer_to_ball_than_opponents(me, opponent1, opponent2, ball):
            if Utilities.is_ball_behind_robot(me, ball):
                return Skills.avoid_own_goal(me, ball)
            # elif me.ally1 and not one_v_one:
            #     print "trying to shoot off the wall"
            #     return Plays.shoot_off_the_wall(me, ball)
            else: #ally2 or 1v1
                return Skills.clear_ball_from_half(me, ball)
        else: # I am in charge of stealing the ball
            closest_opp = Utilities.get_closest_opponent_to_ball(opponent1, opponent2, ball)
            return Plays.steal_ball_from_opponent(me, closest_opp, ball)
    else: #My teammate is closer to the ball
        if Utilities.is_ball_behind_robot(me, ball):
            return Skills.avoid_own_goal(me, ball)
        else:
            if Utilities.am_i_too_close_to_teammate(me, my_teammate):
                return Skills.give_my_teammate_some_space(me, my_teammate)
            else:
                if me.ally1:
                    # call Utilities.are_both_opponents_attacking_goal?
                    return Plays.stay_open_for_pass(me, my_teammate, ball)
                else: # ally2
                    return Plays.stay_between_points_at_distance(Constants.goal_position_home[0], Constants.goal_position_home[1], ball.xhat, ball.yhat, dist_to_maintain)


def goalie(me, my_teammate, opponent1, opponent2, ball, strategy, one_v_one=False):
    global _ball_defend_position, _offensive, _defensive, _neutral

    # First, check to see if the ball is close enough to actuate the kicker, and kick it away
    (x_pos, y_pos) = Utilities.get_front_of_robot(me)
    distance_from_kicker_to_ball = Utilities.get_distance_between_points(x_pos, y_pos, ball.xhat_future, ball.yhat_future) #changed to future and it seemed to work well in the simulator...
    if (distance_from_kicker_to_ball <=  Constants.kickable_distance and not Utilities.is_ball_behind_robot(me, ball)):
        print "KICKING BALL AWAY"
        Skills.kick()

    if Utilities.is_ball_behind_robot(me, ball):
        return Skills.avoid_own_goal(me, ball)
    elif (strategy == _offensive and Utilities.am_i_closer_to_ball_than_opponents(me, opponent1, opponent2, ball)):
        return Skills.clear_ball_from_half(me, ball)
    # Goalie is always defending the goal in an arc
    else:
        theta_c = Utilities.get_angle_between_points(Constants.goal_position_home[0], Constants.goal_position_home[1], ball.xhat, ball.yhat)
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




