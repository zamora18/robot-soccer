import numpy as np

import Plays
import Skills
import Utilities
import Constants

_offensive = 0
_defensive = 1
_nuetral = 2

_ball_defend_position = None


def offensive_attacker(me, my_teammate, opponent1, opponent2, ball, one_v_one=False):
    global _offensive
    
    # Ideas for this Role:
    # check to see who has possession first!
    # Always try and shoot the ball right at the goal. 
    return attacker(me, my_teammate, opponent1, opponent2, ball, _offensive)
    


    # pass

def defensive_attacker(me, my_teammate, opponent1, opponent2, ball):
    global _defensive

    # Ideas for this Role:
    # check to see who has possession first!
     # If there is a clear shot at the goal, then shoot
      # otherwise, if me.ally2, then try and pass it to the teammate
      # Or if me.ally1, then try and shoot it off the walls
    return attacker(me, my_teammate, opponent1, opponent2, ball, _defensive)

def neutral_attacker(me, my_teammate, opponent1, opponent2, ball):
    global _nuetral
    return attacker(me, my_teammate, opponent1, opponent2, ball, _nuetral)




def offensive_defender(me, my_teammate, opponent1, opponent2, ball, one_v_one=False):
    global _offensive
    # If me.ally2, then try and pass it to ally1 (not kicking, but )
    return defender(me, my_teammate, opponent1, opponent2, ball, _offensive)

def defensive_defender(me, my_teammate, opponent1, opponent2, ball):
    global _defensive
    return defender(me, my_teammate, opponent1, opponent2, ball, _defensive)


def neutral_defender(me, my_teammate, opponent1, opponent2, ball):
    global _nuetral
    return defender(me, my_teammate, opponent1, opponent2, ball, _neutral)





def offensive_goalie(me, my_teammate, opponent1, opponent2, ball, one_v_one=False):
    global _offensive
    return goalie(me, my_teammate, opponent1, opponent2, ball, _offensive)

def defensive_goalie(me, my_teammate, opponent1, opponent2, ball):
    global _defensive
    return goalie(me, my_teammate, opponent1, opponent2, ball, _defensive)

def neutral_goalie(me, my_teammate, opponent1, opponent2, ball):
    global _nuetral
    return goalie(me, my_teammate, opponent1, opponent2, ball, _nuetral)



################################################################
# More general functions, that implement the specific strategy #
################################################################

def attacker(me, my_teammate, opponent1, opponent2, ball, strategy):
    global _offensive, _defensive, _neutral
    middle_of_goal = 0

    if (strategy == _offensive):
        goal_target = 0.80
    elif (strategy == _defensive):
        goal_target = 0.20
    else:
        goal_target = 0

    if Utilities.am_i_closest_teammate_to_ball(me, my_teammate, ball):
        if Utilities.am_i_closer_to_ball_than_opponents(me, opponent1, opponent2, ball):
            if me.ally1:
                if ball.yhat > 0: 
                    return Plays.shoot_on_goal(me, ball, goal_target)
                else: 
                    return Plays.shoot_on_goal(me, ball, -goal_target)
            else: # I am ally2
                if (opponent1.xhat < me.xhat and opponent2.xhat < me.xhat):
                    return Plays.shoot_on_goal(me, ball, middle_of_goal)
                else:
                    return Plays.pass_to_teammate(me, my_teammate, ball)

        else: #Basically, I don't have possession and I should be the one to steal the ball
            closest_opp = Utilities.get_closest_opponent_to_ball(opponent1.xhat, opponent1.yhat, opponent2.xhat, opponent2.yhat, ball)
            if (closest_opp == 1):
                opp = opponent1
            else:
                opp = opponent2
            return Plays.steal_ball_from_opponent(me, opp, ball)
    else: #My teammate is closer to the ball than me.
        if me.ally1:
            if (ball.yhat > 0): 
                passing_toggle = -1 
            else: 
                passing_toggle = 1
            return Plays.stay_open_for_pass(me, my_teammate, ball, passing_toggle)
        else: # I am ally2
            return Plays.stay_at_midfield_follow_ball(me, opponent1, opponent2, ball)



def defender(me, my_teammate, opponent1, opponent2, ball, strategy):
    global _offensive, _defensive, _neutral
    
    if strategy == _offensive:
        dist_to_maintain = 0.8
    elif strategy == _defensive:
        dist_to_maintain = 0.2
    else:
        dist_to_maintain = 0.5

    if me.ally1:
        if ball.yhat > 0:
            passing_toggle = -1
        else:
            passing_toggle = 1
        return Plays.stay_open_for_pass(me, my_teammate, ball, passing_toggle)
    else:
        return Plays.stay_between_points_at_distance(me.xhat, me.yhat, opponent1.xhat, opponent1.yhat, dist_to_maintain)



    


def goalie(me, my_teammate, opponent1, opponent2, ball, strategy):
    global _ball_defend_position, _offensive, _defensive, _neutral

    # First, check to see if the ball is close enough to actuate the kicker, and kick it away
    (x_pos, y_pos) = Utilities.get_front_of_robot(me)
    distance_from_kicker_to_ball = Utilities.get_distance_between_points(x_pos, y_pos, ball.xhat_future, ball.yhat_future) #changed to future and it seemed to work well in the simulator...
    if (distance_from_kicker_to_ball <=  Constants.kickable_distance):
        print "KICKING BALL AWAY"
        Skills.kick()

    ############# THIS IF STATEMENT MIGHT NOT BE RIGOROUS ENOUGH, but it will do for now !!!!!!********##########
    ################################################################################################
    # Should I check to see if the ball is behind me? If so, is it too late?
    if (strategy == _offensive and Utilities.our_robot_closer_to_ball(me, opponent1, ball) and not Utilities.is_ball_behind_robot(me, ball)):
        return Skills.attack_ball(me, ball)
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




