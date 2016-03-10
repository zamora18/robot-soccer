from collections import Iterable
import numpy as np

import Skills

_robot_width        = 0.1841 # (7.25 in)

class ShootState:
    setup = 0
    approach = 1
    shoot = 2


_shoot_state = ShootState.setup

def _close(a, b, tolerance=20.0):
    return abs(a - b) <= tolerance

def _robot_close(robot, x, y, theta):
    return _close(x, robot['xhat'], tolerance=.23) and _close(y, robot['yhat'], tolerance=.23) \
                and _close(theta, robot['thetahat'], tolerance = 25)

def shoot(robot, ball, distance_from_center):
    global _shoot_state

    desired_c = Skills.set_up_kick_facing_goal(ball, distance_from_center)

    # transition
    if(_shoot_state == ShootState.setup):
        x, y, theta = Skills.set_up_kick_facing_goal(ball, 0)
        if _robot_close(robot, *desired_c):
            _shoot_state = ShootState.approach

    elif _shoot_state == ShootState.approach:
        a,b,c,theta = Skills.find_triangle(robot['xhat'], robot['yhat'], \
                                ball['xhat'], ball['yhat'])
        if(c < _robot_width * (3.0/4.0)):
            _shoot_state = ShootState.shoot

    elif _shoot_state == ShootState.shoot:
        _shoot_state = ShootState.setup

    else:
        _shoot_state = ShootState.setup

    #action
    if(_shoot_state == ShootState.setup):
        return desired_c

    elif  _shoot_state == ShootState.approach:
        return Skills.approach_to_kick_facing_goal(robot, ball)

    elif _shoot_state == ShootState.shoot:
        print "KICKING"
        Skills.kick()
        return desired_c
    else:
        return robot['xhat'], robot['yhat'], robot['thetahat']


# def advance_ball(robot, opponent, ball):



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


