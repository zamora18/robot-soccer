import time

import numpy as np

import motion
import wheelbase as w
import param as p

x = 0
y = 0
theta = 0

#def init(x=0,y=0,theta=0):
#    globals()['x'] = x
#    globals()['y'] = y
#    globals()['theta'] = theta

def init():
    global x,y,theta
    x = y = theta = 0


def update(time_since_last_update):
    global x, y, theta

    # We need this for Euler's method of derivation
    T = time_since_last_update

    # Get the current world velocities based on current motor speeds
    (vx, vy, omega, s1, s2, s3) = motion.get_velocities()

    # Put data matrix form for easy manipulation
    V = np.matrix([ [vx],[vy],[omega] ])
    P = np.matrix([ [x],[y],[theta] ])

    # Propogate differential equations using Euler's method
    P_next = T*V + P

    # Unpack and update current positions
    x = P_next.getA()[0][0]
    y = P_next.getA()[1][0]
    theta = P_next.getA()[2][0]

    return (x, y, theta, s1,s2,s3)

def RK4():
    """Runge-Kutta 4

    Inspiration:
    http://rosettacode.org/wiki/Runge-Kutta_method
    http://www.cs.cmu.edu/afs/cs.cmu.edu/academic/class/16311/www/s07/labs/NXTLabs/Lab%203.html
    """
    pass
