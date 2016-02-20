import numpy as np

import wheelbase as w
import param as p

PULSES_PER_REV = 19822
PULSES_PER_RADIAN = (PULSES_PER_REV / (2.0*np.pi))

def drive(vx,vy,omega,smooth=False,theta=0):
    """drive
    Take in linear velocity and angular velocity and drive the robot with it!
    vx      : velocity in x direction (world)
    vy      : velocity in y direction (world)
    omega   : angular velocity in degrees (world)
    """
    # Convert omega (rad) into (degrees)
    omega = omega*np.pi/180

    # Convert from world to robot wheel speeds
    (OMEGA1, OMEGA2, OMEGA3) = p.world_to_wheel_speeds(vx, vy, omega, theta)

    # Convert rad/s to qpps
    s1 = int(OMEGA1*PULSES_PER_RADIAN)
    s2 = int(OMEGA2*PULSES_PER_RADIAN)
    s3 = int(OMEGA3*PULSES_PER_RADIAN)

    # Drive the motors, smoothly if specified
    if smooth:
        # This will take 1 second for it to get to the specified
        # speed (third param) using the specified accel (second param)
        w.SpeedAccel(w.M1, abs(s1), s1)
        w.SpeedAccel(w.M2, abs(s2), s2)
        w.SpeedAccel(w.M3, abs(s3), s3)

    else:
        w.Speed(w.M1, s1)
        w.Speed(w.M2, s2)
        w.Speed(w.M3, s3)

    return (s1, s2, s3)

def get_velocities():
    # Ask RoboClaw for encoder speed (not raw speed)
    (s1, s2, s3) = (w.ReadSpeed(w.M1)[1], w.ReadSpeed(w.M2)[1], w.ReadSpeed(w.M3)[1])

    # Convert from qpps to rad/s --> robot wheel speeds
    OMEGA1 = s1/PULSES_PER_RADIAN
    OMEGA2 = s2/PULSES_PER_RADIAN
    OMEGA3 = s3/PULSES_PER_RADIAN

    # Convert from wheel angular speeds to world frame velocities
    (vx, vy, omega) = p.wheel_speeds_to_world(OMEGA1, OMEGA2, OMEGA3, 0)

    # Convert from radians to degrees
    omega = omega*180/pi

    return (vx, vy, omega, s1, s2, s3)

def stop(smooth=True):
    if smooth:
        w.SpeedAccel(w.M1, abs(w.QPPS.M1), 1)
        w.SpeedAccel(w.M2, abs(w.QPPS.M2), 1)
        w.SpeedAccel(w.M3, abs(w.QPPS.M3), 1)
    
    else:
        drive(0,0,0)

def kill():
    """Kill
    Call this only if you don't care about how the bot stops
    """
    w.kill()
