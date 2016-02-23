import numpy as np

from controllers import PID

PID_x = PID(1.5, 0, 0, 2, 0.05, integrator_limit=0.05)
PID_y = PID(1.5, 0, 0, 2, 0.05, integrator_limit=0.05)
PID_theta = PID(1.5, 0, 0, 180, 0.05, integrator_limit=0.05)

_set_point = (0, 0, 0)

velocities = (0, 0, 0)

def init():
    pass

def set_commanded_position(x, y, theta):
    """Set Commanded Position

    x_c, y_c, theta_c. These will tell the controller where it wants to go.

    theta_c (degrees) can be given on the interval [0, 360].
    This function can also receive theta from [-360, 0].

    """
    global _set_point

    # Because theta is periodic with 360 degrees
    # This also deals with theta being negative
    theta = theta % 360

    _set_point = (x, y, theta)
    return True

def get_commanded_position():
    return _set_point

def update(time_since_last_update, xhat, yhat, thetahat):
    global velocities

    # Break out variables for easy access
    x_c = _set_point[0]
    y_c = _set_point[1]
    theta_c = _set_point[2]

    Ts = time_since_last_update

    # Initialize velocities
    vx = vy = w = 0

    # Only control the positions that aren't 'close'
    if not _close(x_c, xhat):
        vx = PID_x.update(x_c, xhat, Ts)

    if not _close(y_c, yhat):
        vy = PID_y.update(y_c, yhat, Ts)

    if vy == 0 and vx == 0 and not _close(theta_c, thetahat, tolerance=2.5): # degrees
        ccw_dist = _ccw_distance(thetahat, theta_c)
        cw_dist  = _cw_distance(thetahat, theta_c)

        if (ccw_dist <= cw_dist):
            # Go CCW (normal)
            w  = abs(PID_theta.update(theta_c, thetahat, Ts))
        else:
            # Go CW (reversed)
            w  = -1*abs(PID_theta.update(theta_c, thetahat, Ts))

    velocities = (vx, vy, w)

    return velocities

def _close(a, b, tolerance=0.05):
    return abs(a - b) <= tolerance

def _ccw_distance(x, x_c):
    return abs((x - (360 + x_c))) % 360

def _cw_distance(x, x_c):
    x_c_complement = x_c - 360
    return abs((x - (x_c_complement))) % 360
