import Plays
import Skills
import Utilities
import Constants

_offensive = 0
_defensive = 1
_nuetral = 2


def offensive_attacker(me, my_teammate, opponent1, opponent2, ball):
	global _offensive
	pass

def defensive_attacker(me, my_teammate, opponent1, opponent2, ball):
	global _defensive
	pass

def neutral_attacker(me, my_teammate, opponent1, opponent2, ball):
	global _nuetral
	pass




def offensive_defender(me, my_teammate, opponent1, opponent2, ball):
	global _offensive
	pass

def defensive_defender(me, my_teammate, opponent1, opponent2, ball):
	global _defensive
	pass

def neutral_defender(me, my_teammate, opponent1, opponent2, ball):
	global _nuetral
	pass




def offensive_goalie(me, my_teammate, opponent1, opponent2, ball):
	global _offensive
	return Plays.goalie(me, my_teammate, opponent1, opponent2, ball, _offensive):

def defensive_goalie(me, my_teammate, opponent1, opponent2, ball):
	global _defensive
	return Plays.goalie(me, my_teammate, opponent1, opponent2, ball, _defensive):

def neutral_goalie(me, my_teammate, opponent1, opponent2, ball):
	global _nuetral
	return Plays.goalie(me, my_teammate, opponent1, opponent2, ball, _nuetral):







