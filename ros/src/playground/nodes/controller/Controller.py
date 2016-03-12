import numpy as np

from controllers import PID

PID_x = PID(2.5, 0, 0, 2, 0.05, integrator_limit=0.05)
PID_y = PID(2.5, 0, 0, 2, 0.05, integrator_limit=0.05)
PID_theta = PID(2, 0, 0, 180, 0.05, integrator_limit=0.05)

_set_point = (0, 0, 0)

# A flag to determine whether or not we are at our set point
_arrived = False

velocities = (0, 0, 0)

def init():
    pass

def set_commanded_position(x, y, theta):
    """Set Commanded Position

    x_c, y_c, theta_c. These will tell the controller where it wants to go.

    theta_c (degrees) can be given on the interval [0, 360].
    This function can also receive theta from [-360, 0].

    """
    global _set_point, _arrived

    # We aren't there yet!
    _arrived = False

    # Because theta is periodic with 360 degrees
    # This also deals with theta being negative
    theta = theta % 360

    _set_point = (x, y, theta)
    return True

def get_commanded_position():
    return _set_point

def update(time_since_last_update, xhat, yhat, thetahat):
    global velocities, _arrived

    if _arrived:
        # Don't even try
        return (0, 0, 0)

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

    if not _close(theta_c, thetahat, tolerance=5): # degrees
        # Since the max distance you should ever go is 180 degrees,
        # test to see so that the commanded value is proportional to
        # the error between commanded and actual.
        # Basicaly, this makes going in circles cooler.
        if abs(thetahat-theta_c) > 180:
            if theta_c < thetahat:
                theta_c = theta_c + 360
            else:
                theta_c = theta_c - 360

        w  = PID_theta.update(theta_c, thetahat, Ts, window_error=False)

    # Are we there yet?
    _arrived = (vx == 0 and vy == 0 and w == 0)

    velocities = (vx, vy, w)

    return velocities

def _close(a, b, tolerance=0.010):
    return abs(a - b) <= tolerance
