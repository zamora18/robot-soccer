import Skills

class ShootState:
    setup = 0
    shoot = 1


_shoot_state = _Shoot_State.setup

def _close(a, b, tolerance=20.0):
    return abs(a - b) <= tolerance

def _robot_close(robot, x, y, theta):
    return _close(x, robot['xhat']) and _close(y, robot['yhat']) \
                and _close(theta, robot['thetahat'])

def shoot(robot, distance_from_center):
    global _shoot_state

    desired_c = Skills.set_up_kick(ball, distance_from_center)

    # transition
    if(_shoot_state == setup):
        x, y, theta = Skills.set_up_kick(ball, 0)
        if _robot_close(robot, *desired_c):
            _shoot_state = ShootState.shoot
    else:
        _shoot_state = ShootState.setup

    #action
    if(_shoot_state == setup):
        return desired_c
    else:
        Skills.kick()
        return desired_c





