#!/usr/bin/python
import numpy as np

class Vector(object):
    """Vector object
    Create an object with an x, y, theta based on magnitude and angle
    """
    def __init__(self, r, theta=0):
        """
        theta is expected in radians
        """
        super(Vector, self).__init__()
        self.r = r
        self.theta = theta
        self.x = r*np.cos(theta)
        self.y = r*np.sin(theta)


R = .0282977488817 # radius of wheel
r = .075 # radius from robot center to each wheel

# r_k is a vector that points from center of robot to center of each wheel
r1 = Vector(r,theta=np.pi/3)
r2 = Vector(r,theta=np.pi)
r3 = Vector(r,theta=5*np.pi/3)

# s_k is a unit vector that points in the direction of spin
s1 = Vector(1,theta=(r1.theta + np.pi/2))
s2 = Vector(1,theta=(r2.theta + np.pi/2))
s3 = Vector(1,theta=(r3.theta + np.pi/2))

# Create the M matrix that relates body and world coordinates
mSub = np.matrix([ [s1.x, s1.y, (s1.y*r1.x - s1.x*r1.y)],
                   [s2.x, s2.y, (s2.y*r2.x - s2.x*r2.y)],
                   [s3.x, s3.y, (s3.y*r3.x - s3.x*r3.y)]
                 ])
M = (1.0/R)*mSub

def rot(theta):
    """Rotation matrix

    Rotate CCW between two coordinate frames by theta.

    Used to rotate from between the world and body frames.
    
    The body frame is the intermediate coordinate frame that
    links the world velocities to the robot wheel speeds.

    theta is expected in radians.
    """
    return np.matrix([ [ np.cos(theta),np.sin(theta),0.0],
                       [-np.sin(theta),np.cos(theta),0.0],
                       [       0.0    ,      0.0    ,1.0]
                     ])


def world_to_wheel_speeds(vx, vy, w, theta):
    """World velocities to wheel speeds

    w and theta are expected in radians
    """
    result = M*rot(theta)*np.matrix([ [vx],[vy],[w] ])
    
    OMEGA1 = result.getA()[0][0]
    OMEGA2 = result.getA()[1][0]
    OMEGA3 = result.getA()[2][0]
    
    return (OMEGA1, OMEGA2, OMEGA3)


def wheel_speeds_to_world(OMEGA1, OMEGA2, OMEGA3, theta):
    """Wheel speeds to world velocities

    theta is expected in radians
    w is returned in radians
    """
    R_T = rot(theta).transpose()
    M_inv = np.linalg.inv(M)

    result = R_T*M_inv*np.matrix([ [OMEGA1],[OMEGA2],[OMEGA3] ])
    
    vx = result.getA()[0][0]
    vy = result.getA()[1][0]
    w  = result.getA()[2][0]

    return (vx, vy, w)