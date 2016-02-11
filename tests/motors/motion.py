import wheelbase as w
import param as p
import numpy as np

_theta = 0 # same as heading

PULSES_PER_REV = 19822
PULSES_PER_RADIAN = (PULSES_PER_REV / (2.0*np.pi))

def drive(vx,vy,omega):
    # Convert from world to robot wheel speeds
    (OMEGA1, OMEGA2, OMEGA3) = p.world_to_wheel_speeds(vx, vy, omega, _theta)

    # Convert rad/s to qpps
    s1 = int(OMEGA1*PULSES_PER_RADIAN)
    s2 = int(OMEGA2*PULSES_PER_RADIAN)
    s3 = int(OMEGA3*PULSES_PER_RADIAN)

    # Drive the motors
    w.Speed(w.M1, s1)
    w.Speed(w.M2, s2)
    w.Speed(w.M3, s3)

    return (s1, s2, s3)

def drive_smooth(vx,vy,omega):
    # Convert from world to robot wheel speeds
    (OMEGA1, OMEGA2, OMEGA3) = p.world_to_wheel_speeds(vx, vy, omega, _theta)

    # Convert rad/s to qpps
    s1 = int(OMEGA1*PULSES_PER_RADIAN)
    s2 = int(OMEGA2*PULSES_PER_RADIAN)
    s3 = int(OMEGA3*PULSES_PER_RADIAN)

    # Drive the motors
    w.SpeedAccel(w.M1, s1*2, s1)
    w.SpeedAccel(w.M2, s2*2, s2)
    w.SpeedAccel(w.M3, s3*2, s3)

    return (s1, s2, s3)

def get_velocities():
    # Ask RoboClaw for encoder speed (not raw speed)
    (s1, s2, s3) = (w.ReadSpeed(w.M1)[1], w.ReadSpeed(w.M2)[1], w.ReadSpeed(w.M3)[1])

    # Convert from qpps to rad/s --> robot wheel speeds
    OMEGA1 = s1/PULSES_PER_RADIAN
    OMEGA2 = s2/PULSES_PER_RADIAN
    OMEGA3 = s3/PULSES_PER_RADIAN

    # Convert from wheel angular speeds to world frame velocities
    (vx, vy, omega) = p.wheel_speeds_to_world(OMEGA1, OMEGA2, OMEGA3, _theta)

    return (vx, vy, omega)

def stop():
    drive(0,0,0)

def smooth_stop():
    w.SpeedAccel(w.M1,w.QPPS.M1*1,1)
    w.SpeedAccel(w.M2,w.QPPS.M2*1,1)
    w.SpeedAccel(w.M3,w.QPPS.M3*1,1)

def kill():
    """Kill
    Call this only if you don't care about how the bot stops
    """
    w.kill()
