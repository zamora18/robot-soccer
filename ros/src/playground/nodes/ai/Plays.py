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
    return _close(x, robot['xhat'], tolerance=.03) and _close(y, robot['yhat'], .03) \
                and _close(theta, robot['thetahat'], tolerance = 10)

def shoot(robot, ball, distance_from_center):
    global _shoot_state

    desired_c = Skills.set_up_kick(ball, distance_from_center)

    # transition
    if(_shoot_state == ShootState.setup):
        x, y, theta = Skills.set_up_kick(ball, 0)
        if _robot_close(robot, *desired_c):
            _shoot_state = ShootState.approach

    elif _shoot_state == ShootState.approach:
        a,b,c,theta = Skills.find_triangle(robot['xhat'], robot['yhat'], \
                                ball['xhat'], ball['yhat'])
        if(c < _robot_width * (3/4)):
            _shoot_state = ShootState.shoot

    elif _shoot_state == ShootState.shoot:
        _shoot_state = ShootState.setup

    else:
        _shoot_state = ShootState.setup

    #action
    if(_shoot_state == ShootState.setup):
        return desired_c

    elif  _shoot_state == ShootState.approach:
        return Skills.approach_to_kick(robot, ball)

    elif _shoot_state == ShootState.shoot:
        Skills.kick()
        return desired_c
    else:
        return robot['xhat'], robot['yhat'], robot['thetahat']





