import numpy as np

from controllers import PID

PID_x = PID(0.65, 0.01, 0, 1, 0.05, integrator_limit=0.05)
PID_y = PID(0.65, 0.01, 0, 1, 0.05, integrator_limit=0.05)
PID_theta = PID(0.5, 0, 0, 1, 0.05, integrator_limit=0.05)

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

    if vy == 0 and vx == 0 and not _close(theta_c, thetahat, tolerance=10): # 10 degrees
#        print 'theta'
        #reverse = False
        #if (theta_c - thetahat) > 180:
        #    thetahat = thetahat - 180
        #    theta_c = theta_c + 180
        #    reverse = True
        w  = PID_theta.update(theta_c, thetahat, Ts)
        #if reverse:
        #    w = -1*w
#if ((theta_c + np.pi) - thetahat) > 0:
        #    w = -1*w

    velocities = (vx, vy, w)

    #print("velocities: {}\r".format(velocities))

    return velocities

def _close(a, b, tolerance=0.05):
    return abs(a - b) <= tolerance
