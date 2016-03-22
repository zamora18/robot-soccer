import numpy as np

import Roles
import Plays
import Skills
import Utilities
import Constants

from GameObjects import Ball, Robot

#Variables for tracking opponent's strategy
_avg_dist_between_robots    = 0
_averaging_factor           = 0
_time_in_our_half           = 0

# If we decide to go for a trick shot at the beginning
_beginning_trick_shot = False

# ally1 is designated as the "main" attacker, or the robot closest to the opponent's goal at the beginning of the game
# ally2 is designated as the "main" defender, or the robot closest to our goal at the beginnning of the game

def choose_strategy(me, my_teammate, opponent1, opponent2, ball, goal):
	
	#for now, we will just focus on aggressive offense
	if goal:
		return reset_positions_after_goal(me)
	else:
		return aggressive_offense(me, my_teammate, opponent1, opponent2, ball)


def aggressive_offense(me, my_teammate, opponent1, opponent2, ball):
	section = Utilities.get_field_section(ball.xhat)

	if me.ally1:
		if   section == 1:
			return Roles.offensive_attacker(me, my_teammate, opponent1, opponent2, ball)
	    elif section == 2:
	    	return Roles.offensive_attacker(me, my_teammate, opponent1, opponent2, ball)
	    elif section == 3:
	    	return Roles.offensive_defender(me, my_teammate, opponent1, opponent2, ball)
	    elif section == 4:
	    	return Roles.offensive_defender(me, my_teammate, opponent1, opponent2, ball)
	    else:
    		return (robot.xhat, robot.yhat, robot.theta_hat) #default, returns current pos
	else:
		if   section == 1:
			return Roles.offensive_attacker(me, my_teammate, opponent1, opponent2, ball)
	    elif section == 2:
	    	return Roles.offensive_attacker(me, my_teammate, opponent1, opponent2, ball)
	    elif section == 3:
	    	return Roles.offensive_defender(me, my_teammate, opponent1, opponent2, ball)
	    elif section == 4:
	    	return Roles.offensive_goalie(me, my_teammate, opponent1, opponent2, ball)
	    else:
    		return (robot.xhat, robot.yhat, robot.theta_hat) #default, returns current pos


def aggressive_defense(me, my_teammate, opponent1, opponent2, ball):
	section = Utilities.get_field_section(ball.xhat)

	if me.ally1:
		if   section == 1:
			return Roles.defensive_attacker(me, my_teammate, opponent1, opponent2, ball)
	    elif section == 2:
	    	return Roles.defensive_attacker(me, my_teammate, opponent1, opponent2, ball)
	    elif section == 3:
	    	return Roles.defensive_defender(me, my_teammate, opponent1, opponent2, ball)
	    elif section == 4:
	    	return Roles.defensive_defender(me, my_teammate, opponent1, opponent2, ball)
	    else:
    		return (robot.xhat, robot.yhat, robot.theta_hat) #default, returns current pos
	else:
		if   section == 1:
			return Roles.defensive_defender(me, my_teammate, opponent1, opponent2, ball)
	    elif section == 2:
	    	return Roles.defensive_defender(me, my_teammate, opponent1, opponent2, ball)
	    elif section == 3:
	    	return Roles.defensive_goalie(me, my_teammate, opponent1, opponent2, ball)
	    elif section == 4:
	    	return Roles.defensive_goalie(me, my_teammate, opponent1, opponent2, ball)
	    else:
    		return (robot.xhat, robot.yhat, robot.theta_hat) #default, returns current pos


def passive_aggressive(me, my_teammate, opponent1, opponent2, ball): #AKA, mild offense/defense
	section = Utilities.get_field_section(ball.xhat)

	if me.ally1:
		if   section == 1:
			return Roles.neutral_attacker(me, my_teammate, opponent1, opponent2, ball)
	    elif section == 2:
	    	return Roles.neutral_attacker(me, my_teammate, opponent1, opponent2, ball)
	    elif section == 3:
	    	return Roles.neutral_defender(me, my_teammate, opponent1, opponent2, ball)
	    elif section == 4:
	    	return Roles.neutral_defender(me, my_teammate, opponent1, opponent2, ball)
	    else:
    		return (robot.xhat, robot.yhat, robot.theta_hat) #default, returns current pos
	else:
		if   section == 1:
			return Roles.neutral_attacker(me, my_teammate, opponent1, opponent2, ball)
	    elif section == 2:
	    	return Roles.neutral_defender(me, my_teammate, opponent1, opponent2, ball)
	    elif section == 3:
	    	return Roles.neutral_defender(me, my_teammate, opponent1, opponent2, ball)
	    elif section == 4:
	    	return Roles.neutral_goalie(me, my_teammate, opponent1, opponent2, ball)
	    else:
    		return (robot.xhat, robot.yhat, robot.theta_hat) #default, returns current pos


def one_on_one(me, my_teammate, opponent1, opponent2, ball):
	if   section == 1:
		return Roles.offensive_attacker(me, my_teammate, opponent1, opponent2, ball)
    elif section == 2:
    	return Roles.offensive_attacker(me, my_teammate, opponent1, opponent2, ball)
    elif section == 3:
    	return Roles.offensive_defender(me, my_teammate, opponent1, opponent2, ball)
    elif section == 4:
    	return Roles.offensive_goalie(me, my_teammate, opponent1, opponent2, ball)
    else:
		return (robot.xhat, robot.yhat, robot.theta_hat) #default, returns current pos


def reset_positions_after_goal(me):
	pass