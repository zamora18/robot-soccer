import time

import numpy as np

import motion
import wheelbase as w
import param as p

x = 0
y = 0
theta = 0

# Keep track of what the previous s1, s2, s3 where
_prev = { 's1': 0, 's2': 0, 's3': 0 }
_data_spikes = { 's1': 0, 's2': 0, 's3': 0 }
_data_spike_tolerance = 1000

def _is_data_spike(s1, s2, s3):
    diff1 = abs(s1 - _prev['s1'])
    diff2 = abs(s2 - _prev['s2'])
    diff3 = abs(s3 - _prev['s3'])

    if diff1 > _data_spike_tolerance:
        # s1 had a spike
        _data_spikes['s1'] += 1
        return True

    if diff2 > _data_spike_tolerance:
        # s2 had a spike
        _data_spikes['s2'] += 1
        return True

    if diff3 > _data_spike_tolerance:
        # s3 had a spike
        _data_spikes['s3'] += 1
        return True

    # No spike, so update for next time
    _data_spikes['s1'] = s1
    _data_spikes['s2'] = s2
    _data_spikes['s3'] = s3

    return False


def init(x=0,y=0,theta=0):
   globals()['x'] = x
   globals()['y'] = y
   globals()['theta'] = theta


def update(time_since_last_update):
    global x, y, theta

    # We need this for Euler's method of derivation
    T = time_since_last_update

    # Get the current world velocities based on current motor speeds
    (vx, vy, omega, s1, s2, s3) = motion.get_velocities()

    # Check if there was a data spike
    if _is_data_spike(s1, s2, s3):
        print _data_spikes

    # Put data matrix form for easy manipulation
    V = np.matrix([ [vx],[vy],[omega] ])
    P = np.matrix([ [x],[y],[theta] ])

    # Propogate differential equations using Euler's method
    P_next = T*V + P

    # Unpack and update current positions
    x = P_next.getA()[0][0]
    y = P_next.getA()[1][0]
    theta = P_next.getA()[2][0]

    return (x, y, theta, s1, s2, s3)

def RK4():
    """Runge-Kutta 4

    Inspiration:
    http://rosettacode.org/wiki/Runge-Kutta_method
    http://www.cs.cmu.edu/afs/cs.cmu.edu/academic/class/16311/www/s07/labs/NXTLabs/Lab%203.html
    """
    pass
