import time
import sys
import imp

import numpy as np
from Getch import _Getch

from repeated_timer import RepeatedTimer

motion = imp.load_source('motion', '../../ros/src/playground/nodes/motion/motion.py')
w = imp.load_source('wheelbase', '../../ros/src/playground/nodes/motion/wheelbase.py')
Odometry = imp.load_source('Odometry', '../../ros/src/playground/nodes/odometry/Odometry.py')
Controller = imp.load_source('Controller', '../../ros/src/playground/nodes/controller/Controller.py')
Getch = imp.load_source('Getch', '../../ros/src/playground/nodes/teleop/Getch.py')

import calibrator as c

_motion_timer = None
_odom_timer = None
_ctrl_timer = None

_motion_timer_period = 1.0/6
_odom_timer_period = 1.0/7
_ctrl_timer_period = 1.0/10

_vx = 0.5
_vy = 0.5
_w = np.pi
_velocities = (0, 0, 0)

_xhat = 0
_yhat = 0
_thetahat = 0
_velocityhat = 0

_set_speed = True

_smooth = True
_odom_on = True
_ctrl_on = False
_previous_action = None

def _action_requires_stop(action):
    if action == _previous_action:
        return False

    if action == 'UP' and _previous_action != 'DOWN':
        return True

    if action == 'DOWN' and _previous_action != 'UP':
        return True

    if action == 'LEFT' and _previous_action != 'RIGHT':
        return True

    if action == 'RIGHT' and _previous_action != 'LEFT':
        return True

    if action == 'SPIN_CW' and _previous_action != 'SPIN_CCW':
        return True

    if action == 'SPIN_CCW' and _previous_action != 'SPIN_CW':
        return True

def _go_home():
    # Get current robot position, rounded to 1 decimal place
    xhat = round(_xhat, 1)
    yhat = round(_yhat 1)
    thetahat = round(_thetahat, 1)

    # Set a tolerance
    tolerance = 0.1

    if abs(xhat) > tolerance:
        dt = abs(xhat / _vx)
        sign = -1 if xhat > 0 else 1
        print("motion.drive({},{},{}) for {} s\r".format(sign*_vx, 0, 0, dt))
        motion.drive(sign*_vx, 0, 0)
        time.sleep(dt)
        motion.stop()
        time.sleep(0.5)

    if abs(yhat) > tolerance:
        dt = abs(yhat / _vy)
        sign = -1 if yhat > 0 else 1
        print("motion.drive({},{},{}) for {} s\r".format(0, sign*_vy, 0, dt))
        motion.drive(0, sign*_vy, 0)
        time.sleep(dt)
        motion.stop()
        time.sleep(0.5)

    if abs(thetahat) > tolerance:
        # since it's periodic...
#        thetahat = round(thetahat%(2*np.pi) ,2)
        dt = abs(thetahat / _w)
        sign = -1 if thetahat > 0 else 1
        print("motion.drive({},{},{}) for {} s\r".format(0, 0, sign*_w, dt))
        motion.drive(0, 0, sign*_w)
        time.sleep(dt)
        motion.stop()
        time.sleep(0.5)

def _ask_for_point():
    """Ask for point
    Asks user for a point to go to. User enters three floats, separated
    by spaces, with no other characters; x, y, theta respectively
    """
    usr = raw_input("Input point, (syntax: \"x y theta\"): ")

    usr_list = usr.split()

    if len(usr_list) != 3:
        print("\n\rEnter the point correctly, dummy.\r\n")
        return False

    try:
        x = float(usr_list[0])
        y = float(usr_list[1])
        theta = float(usr_list[2])
    except:
        print("\n\rThere was a problem making your input a float.\r\n")
        return False

    print("Going to: ({}, {}, {})\r".format(x,y,theta))
    return Controller.set_commanded_position(x, y, theta)

def _close(a, b, tolerance=0.050):
    return abs(a - b) <= tolerance


def _deal_with_calibration():

    usr = raw_input("What do you want? ('calibrate 48 2 t', 'test 0.6 1.5 m1 m2 m3') ").upper()

    action = usr.split()[0]
    other = usr.split()[1:]

    if action == 'CALIBRATE':
        try:
            speed = int(other[0])
        except:
            speed = 48

        try:
            sleep_time = float(other[1])
        except:
            sleep_time = 2

        try:
            set_PID = (other[2] == 'T')
        except:
            set_PID = True

        c.calibrate(speed=speed,sleep_time=sleep_time,set_PID=set_PID)

    elif action == 'TEST':
        try:
            velocity = float(other[0])
        except:
            velocity = 0.6

        try:
            sleep_time = float(other[1])
        except:
            sleep_time = 2

        try:
            M1QPPS = float(other[2])
            M2QPPS = float(other[3])
            M3QPPS = float(other[4])
        except:
            M1QPPS = None
            M2QPPS = None
            M3QPPS = None

        c.test_calibration(velocity=velocity,sleep_time=sleep_time,
                            M1QPPS=M1QPPS,M2QPPS=M2QPPS,M3QPPS=M3QPPS)


def _handle_motion_timer():
    global _set_speed, _velocityhat
    if _set_speed:
        motion.drive(*_velocities,smooth=_smooth)
        _set_speed = False

    (vx, vy, w, s1, s2, s3) = motion.get_velocities()
    _velocityhat = (vx, vy, w)


def _handle_odom_timer():
    global _ctrl_on, _velocities, _set_speed, _xhat, _yhat, _thetahat
    if _odom_on:
        odom = Odometry.update(_odom_timer_period, _velocityhat)
        print "{}\r".format(odom)

        _xhat = odom[0]
        _yhat = odom[1]
        _thetahat = odom[2]

        if _ctrl_on:
            x_c, y_c, theta_c = Controller.get_commanded_position()
            if _close(_xhat, x_c) and _close(_yhat, y_c) and \
                    _close(_thetahat, theta_c, tolerance=1000000*(np.pi/12)):
                _ctrl_on = False
                print("\r\n*** Reached Set Point within Tolerances ***\r\n")
                _motion_timer.stop()
                motion.stop()
                time.sleep(0.5)
                _velocities = (0, 0, 0)
                _set_speed = True
                _motion_timer.start()


def _handle_ctrl_timer():
    global _set_speed
    global _velocities
    if _ctrl_on:
        _velocities = Controller.update(_ctrl_timer_period, _xhat, _yhat, _thetahat)
        #print("Vel: {}\r".format(_velocities))
        _set_speed = True

def get_battery():
    return w.ReadMainBatteryVoltage()[1]/10.0

def get_action():
    getch = _Getch()
    k = getch()
    if k == 'w':
        return 'UP'
    elif k == 's':
        return 'DOWN'
    elif k == 'd':
        return 'RIGHT'
    elif k == 'a':
        return 'LEFT'
    elif k == 'x':
        return 'SPIN_CW'
    elif k == 'z':
        return 'SPIN_CCW'
    elif k == 'h':
        return 'SET_HOME'
    elif k == 'u':
        return 'TOGGLE_SMOOTH'
    elif k == 'o':
        return 'TOGGLE_ODOM'
    elif k == 'b':
        return 'BREAKPOINT'
    elif k == 'B':
        return 'BATTERY'
    elif k == 'H':
        return 'GO_HOME'
    elif k == 'G':
        return 'GO_TO_POINT'
    elif k == 'c':
        return 'TOGGLE_CNTRL'
    elif k == 'C':
        return 'CALIBRATION_MODE'
    elif k == ' ':
        return 'DIE'

def main():
    global _motion_timer
    _motion_timer = RepeatedTimer(_motion_timer_period, _handle_motion_timer)
    _motion_timer.start()

    global _odom_timer
    _odom_timer = RepeatedTimer(_odom_timer_period, _handle_odom_timer)
    _odom_timer.start()

    global _ctrl_timer
    _ctrl_timer = RepeatedTimer(_ctrl_timer_period, _handle_ctrl_timer)
    _ctrl_timer.start()

    w.init()
    print 'Started.'


    global _velocities, _set_speed, _smooth, _odom_on, _previous_action, _ctrl_on

    while(1):
        action = get_action()
        if action == 'UP':
            _set_speed = True
            _velocities = (0, _vy, 0)

        elif action == 'DOWN':
            _set_speed = True
            _velocities = (0, -_vy, 0)

        elif action == 'RIGHT':
            _set_speed = True
            _velocities = (_vx, 0, 0)

        elif action == 'LEFT':
            _set_speed = True
            _velocities = (-_vx, 0, 0)

        elif action == 'SPIN_CW':
            _set_speed = True
            _velocities = (0, 0, _w)

        elif action == 'SPIN_CCW':
            _set_speed = True
            _velocities = (0, 0, -_w)

        elif action == 'SET_HOME':
            Odometry.init()

        elif action == 'GO_HOME':
            _motion_timer.stop()
            motion.stop()
            time.sleep(1)

            _go_home()

            time.sleep(1)
            _set_speed = True
            _velocities = (0, 0, 0)
            _motion_timer.start()

        elif action == 'GO_TO_POINT':
            _toggle_timers(False)
            motion.stop()
            time.sleep(1)

            _ask_for_point()

            time.sleep(1)
            _ctrl_on = True
            _odom_on = True
            _toggle_timers(True)

        elif action == 'TOGGLE_CNTRL':
            _ctrl_on = not _ctrl_on
            print("Controller: {}".format(_ctrl_on))

        elif action == 'TOGGLE_SMOOTH':
            _smooth = not _smooth
            print("Smooth: {}".format(_smooth))

        elif action == 'TOGGLE_ODOM':
            _odom_on = not _odom_on
            print("Odom: {}".format(_odom_on))

        elif action == 'BATTERY':
            print("\n\r*** Battery: {} ***\n\r".format(get_battery()))

        elif action == 'BREAKPOINT':
            _toggle_timers(False)
            import ipdb; ipdb.set_trace()
            _toggle_timers(True)

        elif action == 'CALIBRATION_MODE':
            _toggle_timers(False)

            _deal_with_calibration()

            time.sleep(1)
            _toggle_timers(True)

        elif action == 'DIE':
            _toggle_timers(False)
            motion.stop()
            w.kill()
            return sys.exit(0)

        else:
            _set_speed = True
            _velocities = (0, 0, 0)

        # handle stopping before changing directions
        if _action_requires_stop(action):
            _set_speed = False
            motion.stop()
            time.sleep(0.4)
            _set_speed = True

        _previous_action = action

        print "{}\r".format(_velocities)


def _toggle_timers(on):
    if on:
        _motion_timer.start()
        _odom_timer.start()
        _ctrl_timer.start()

    else:
        _motion_timer.stop()
        _odom_timer.stop()
        _ctrl_timer.stop()


if __name__ == '__main__':
    main()
