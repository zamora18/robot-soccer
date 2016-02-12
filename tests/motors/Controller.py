import numpy as np
from repeated_timer import RepeatedTimer

import Odometry
import motion

from controllers import PID

PID_x = PID(0.75, 0.025, 0, 1, 0.05, integrator_limit=0.05)

_set_point = (0, 0, 0)

velocities = (0, 0, 0)

def init():
    pass

def set_commanded_position(x, y, theta):
    global _set_point
    _set_point = (x, y, theta)
    return True

def get_commanded_position():
    return _set_point

def update(time_since_last_update):
    global velocities

    # Break out variables for easy access
    x = Odometry.x
    y = Odometry.y
    theta = Odometry.theta
    x_c = _set_point[0]
    y_c = _set_point[1]
    theta_c = _set_point[2]

    vx = PID_x.update(x_c,x,time_since_last_update)
    vy = 0
    w  = 0

    velocities = (vx, vy, w)

    return velocities