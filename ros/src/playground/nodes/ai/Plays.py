import numpy as np

import Skills
import Utilities
import Constants

class ShootState:
    setup = 0
    attack = 1
    shoot = 2

_ball_defend_position = None

_shoot_state = ShootState.setup
_trick_state = ShootState.setup

_offensive = 0
_defensive = 1
_neutral = 2


def attacker(me, my_teammate, opponent1, opponent2, ball, strategy):
	# global _offensive, _defensive, _neutral
	pass

def defender(me, my_teammate, opponent1, opponent2, ball, strategy):
	# global _offensive, _defensive, _neutral
	pass

def goalie(me, my_teammate, opponent1, opponent2, ball, strategy):
	global _ball_defend_position, _offensive, _defensive, _neutral

	# First, check to see if the ball is close enough to actuate the kicker, and kick it away
	(x_pos, y_pos) = Utilities.get_front_of_robot(robot)
    distance_from_kicker_to_ball = Utilities.get_distance_between_points(x_pos, y_pos, ball['xhat_future'], ball['yhat_future']) #changed to future and it seemed to work well in the simulator...
    if (distance_from_kicker_to_ball <=  Utilities.kickable_distance):
    	print "KICKING BALL AWAY"
    	Skills.kick()

	# this might not be rigorous enough, but it will do for now !!!!!!********##########
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
 


	
