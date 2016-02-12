import time
import sys

import numpy as np
from Getch import _Getch

from repeated_timer import RepeatedTimer
import motion
import wheelbase as w
import Odometry
import Controller

_motion_timer = None
_odom_timer = None
_ctrl_timer = None

_motion_timer_period = 1.0/10
_odom_timer_period = 1.0/6
_ctrl_timer_period = 1.0/10

_vx = 0.5
_vy = 0.5
_w = np.pi
_velocities = (0, 0, 0)

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
    x = round(Odometry.x, 1)
    y = round(Odometry.y, 1)
    theta = round(Odometry.theta, 1)

    # Set a tolerance
    tolerance = 0.1

    if abs(x) > tolerance:
        dt = abs(x / _vx)
        sign = -1 if x > 0 else 1
        print("motion.drive({},{},{}) for {} s\r".format(sign*_vx, 0, 0, dt))
        motion.drive(sign*_vx, 0, 0)
        time.sleep(dt)
        motion.stop()
        time.sleep(0.5)

    if abs(y) > tolerance:
        dt = abs(y / _vy)
        sign = -1 if y > 0 else 1
        print("motion.drive({},{},{}) for {} s\r".format(0, sign*_vy, 0, dt))
        motion.drive(0, sign*_vy, 0)
        time.sleep(dt)
        motion.stop()
        time.sleep(0.5)

    if abs(theta) > tolerance:
        # since it's periodic...
#        theta = round(theta%(2*np.pi) ,2)
        dt = abs(theta / _w)
        sign = -1 if theta > 0 else 1
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


def _handle_motion_timer():
    global _set_speed
    if _set_speed:
        motion.drive(*_velocities,smooth=_smooth)
        _set_speed = False


def _handle_odom_timer():
    if _odom_on:
        print "{}\r".format(Odometry.update(_odom_timer_period))

        if _ctrl_on:
            x_c, y_c, theta_c = Controller.get_commanded_position()
            if _close(Odometry.x, x_c) and _close(Odometry.y, y_c) and
                    _close(Odometry.theta, theta_c, tolerance=(np.pi/32)):
                _ctrl_on = False


def _handle_ctrl_timer():
    global _set_speed
    global _velocities
    if _ctrl_on:
        _velocities = Controller.update(_ctrl_timer_period)
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
    elif k == 'C':
        return 'TOGGLE_CNTRL'
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
            _motion_timer.stop()
            _odom_timer.stop()
            _ctrl_timer.stop()
            motion.stop()
            time.sleep(1)

            _ask_for_point()

            time.sleep(1)
            _ctrl_on = True
            _odom_on = True
            _motion_timer.start()
            _odom_timer.start()
            _ctrl_timer.start()

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
            _odom_timer.stop()
            _motion_timer.stop()
            import ipdb; ipdb.set_trace()
            _odom_timer.start()
            _motion_timer.start()

        elif action == 'DIE':
            _motion_timer.stop()
            _odom_timer.stop()
            _ctrl_timer.stop()
            motion.stop()
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

if __name__ == '__main__':
    main()
