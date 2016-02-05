import wheelbase as w
import param as p
import numpy as np

_x = 0
_y = 0
_theta = 0 # same as heading

PULSES_PER_REV = 19822
PULSES_PER_RADIAN = (PULSES_PER_REV / (2.0*np.pi))

def drive(vx,vy,omega):
    # Convert from world to robot velocities
    (OMEGA1, OMEGA2, OMEGA3) = p.get_wheels_angular_velocity(vx, vy, omega, _theta)

    # Convert rad/s to qpps
    s1 = int(OMEGA1*PULSES_PER_RADIAN)
    s2 = int(OMEGA2*PULSES_PER_RADIAN)
    s3 = int(OMEGA3*PULSES_PER_RADIAN)

    # Drive the motors
    w.Speed(w.M1, s1)
    w.Speed(w.M2, s2)
    w.Speed(w.M3, s3)

def stop():
    w.kill()
