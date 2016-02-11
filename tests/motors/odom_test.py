import select
import signal
import sys
import termios
import time
import tty

import numpy as np

from repeated_timer import RepeatedTimer
import motion
import wheelbase as w
import Odometry

class _Getch:
    """Gets a single character from standard input.  Does not echo to the
screen."""
    def __init__(self):
        try:
            self.impl = _GetchWindows()
        except ImportError:
            self.impl = _GetchUnix()

    def __call__(self): return self.impl()


class _GetchUnix:
    def __init__(self):
        import tty, sys

    def __call__(self):
        import sys, tty, termios
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setraw(sys.stdin.fileno())
            ch = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return ch


class _GetchWindows:
    def __init__(self):
        import msvcrt

    def __call__(self):
        import msvcrt
        return msvcrt.getch()



_motion_timer = None
_odom_timer = None

_motion_timer_period = 1.0/100
_odom_timer_period = 1.0/10

_vx = 0.5
_vy = 0.5
_w = np.pi
_velocities = (0, 0, 0)

_set_speed = True


def handle_motion_timer():
    global _set_speed
    if _set_speed:
        motion.drive(*_velocities)
        _set_speed = False


def handle_odom_timer():
    pass
    (x, y, theta) = Odometry.update(_odom_timer_period)
    print "{}\r".format((x,y,theta))

def get_direction():
    getch = _Getch()
    k = getch()
    if k == 'w' or k == 'W':
        return 'UP'
    elif k == 's' or k == 'S':
        return 'DOWN'
    elif k == 'd' or k == 'D':
        return 'RIGHT'
    elif k == 'a' or k == 'A':
        return 'LEFT'
    elif k == 'x' or k == 'X':
        return 'SPIN_CW'
    elif k == 'z' or k == 'Z':
        return 'SPIN_CCW'
    elif k == 'h' or k == 'H':
        return 'SET_HOME'
    elif k == ' ':
        _motion_timer.stop()
        _odom_timer.stop()
        motion.stop()
        return sys.exit(0)

def main():
    global _motion_timer
    _motion_timer = RepeatedTimer(_motion_timer_period, handle_motion_timer)
    _motion_timer.start()

    global _odom_timer
    _odom_timer = RepeatedTimer(_odom_timer_period, handle_odom_timer)
    _motion_timer.start()

    w.init()
    print 'started'


    global _velocities, _set_speed

    while(1):
        dir = get_direction()
        if dir == 'UP':
            _set_speed = True
            _velocities = (0, _vy, 0)

        elif dir == 'DOWN':
            _set_speed = True
            _velocities = (0, -_vy, 0)

        elif dir == 'RIGHT':
            _set_speed = True
            _velocities = (_vx, 0, 0)

        elif dir == 'LEFT':
            _set_speed = True
            _velocities = (-_vx, 0, 0)

        elif dir == 'SPIN_CW':
            _set_speed = True
            _velocities = (0, 0, _w)

        elif dir == 'SPIN_CCW':
            _set_speed = True
            _velocities = (0, 0, -_w)

        elif dir == 'SET_HOME':
            Odometry.init()

        else:
            _set_speed = True
            _velocities = (0, 0, 0)

        print "{}\r".format(_velocities)

if __name__ == '__main__':
    main()
