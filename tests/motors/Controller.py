import numpy as np
from repeated_timer import RepeatedTimer

import Odometry
import motion

_set_point = (0, 0, 0)

velocities = (0, 0, 0)

def init():
    pass

def set_commanded_position(x, y, theta):
    global _set_point
    _set_point = (x, y, theta)

def update(time_since_last_update):
    global velocities

    # Break out variables for easy access
    x = Odometry.x
    y = Odometry.y
    theta = Odometry.theta
    x_c = _set_point[0]
    y_c = _set_point[1]
    theta_c = _set_point[2]

    vx = PID_x(x_c,x,0.5,0,0,1,time_since_last_update,0.05)
    vy = 0
    w  = 0

    velocities = (vx, vy, w)

    return velocities



def PID_x(x_c,x,kp,ki,kd,limit,Ts,tau):
    # Declare persistent/static variables
    if "integrator" not in PID_x.__dict__: PID_x.integrator = 0
    if "xdot" not in PID_x.__dict__: PID_x.xdot = 0
    if "error_d1" not in PID_x.__dict__: PID_x.error_d1 = 0
    if "x_d1" not in PID_x.__dict__: PID_x.x_d1 = 0

    # compute the error
    error = x_c - x

#    print "\t\t `x error: {}\r".format(error)

    # update derivative of x
    PID_x.xdot = _tustin_derivative(tau, Ts, PID_x.xdot, PID_x.x_d1, x)

    # update integral of error
    if ki and abs(PID_x.xdot)<0.05:
        PID_x.integrator = _tustin_integral(Ts, PID_x.integrator, error,
                                                    PID_x.error_d1)

    # update delayed variables for next time
    PID_x.error_d1 = error
    PID_x.x_d1 = x

    # compute the PID control signal
    u_unsat = kp*error + ki*PID_x.integrator - kd*PID_x.xdot;
    u = u_unsat
#    u = _sat(u_unsat, limit)

    # more integrator anti-windup
    if ki:
        PID_x.integrator = PID_x.integrator + Ts/ki*(u-u_unsat)

    return u


def _sat(val, limit):
    out = 0

    if val > limit:
        out = limit
    elif val < limit:
        out = -limit
    else:
        out = val

    return out

def _tustin_derivative(tau,Ts,xdot,x_d1,x):
    return (2*tau-Ts)/(2*tau+Ts)*xdot + 2/(2*tau+Ts)*(x-x_d1)

def _tustin_integral(Ts,integrator,x,x_d1):
    return integrator + (Ts/2)*(x+x_d1)
