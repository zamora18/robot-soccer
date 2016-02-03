#!/usr/bin/python
import numpy as np

class Vector(object):
    """Vector object
    Create an object with an x, y, theta based on magnitude and angle
    """
    def __init__(self, r, theta=0):
        super(Vector, self).__init__()
        self.r = r
        self.theta = theta
        self.x = r*np.cos(theta)
        self.y = r*np.sin(theta)


R = .0282977488817 # radius of wheel
r = .075 # radius from robot center to each wheel

# r_k is a vector that points from center of robot to center of each wheel
r1 = Vector(r,theta=0)
r2 = Vector(r,theta=np.pi)
r3 = Vector(r,theta=3*np.pi/2)

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

def Rot(theta):
    """Rotation matrix
    """
    return np.matrix([ [ np.cos(theta),np.sin(theta),0.0],
                       [-np.sin(theta),np.cos(theta),0.0],
                       [       0.0    ,      0.0    ,1.0]
                     ])


def get_wheels_angular_velocity(vx, vy, omega, theta):
    result = M*Rot(theta)*np.matrix([ [vx],[vy],[omega] ])
    tmp = (result.getA()[0][0], result.getA()[1][0], result.getA()[2][0])
    return tmp

# -------------------------------------

# def getWheelVel(x,y,omega):
#   desired = matrix( [[x],
#                      [y],
#                      [omega]] )
                   
#   result = M*desired

#   return result.getA()[0][0], result.getA()[1][0], result.getA()[2][0]
  
# def getXYOmega(v1,v2,v3):
#   velocity = matrix( [[v1],
#                       [v2],
#                       [v3]] )
#   Minv = linalg.inv(M)
  
#   result = Minv*velocity
  
#   return result.getA()[0][0], result.getA()[1][0], result.getA()[2][0]

# def getRobotXYOmega(x,y,omega,theta):
#   desired = matrix( [[x],
#                      [y],
#                      [omega]] )
#   desired = R(theta)*desired
#   return desired
  
# def getRobotXYOmegaAsTuple(x, y, omega, theta):
#   desired = getRobotXYOmega(x, y, omega, theta)
#   asArray = desired.getA()
#   return asArray[0][0], asArray[1][0], asArray[2][0]

# def getWheelVelTheta(x,y,omega,theta):
#   desired = getRobotXYOmega(x, y, omega, theta)
                   
#   result = M*desired

#   return result.getA()[0][0], result.getA()[1][0], result.getA()[2][0]

# def radianToQpps(radian):
#   result = int(radian * 19820.0 / (2*math.pi))
#   #print result 
#   if result > 308420:
#     return 308420
#   elif result < -308420:
#     return -308420
#   else:
#     return result