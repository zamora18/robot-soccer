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
_ctrlr_timer = None

_motion_timer_period = 1.0/100
_odom_timer_period = 1.0/6
_ctrlr_timer_period = 1.0/100

_vx = 0.5
_vy = 0.5
_w = np.pi
_velocities = (0, 0, 0)

_set_speed = True

_smooth = False
_odom_on = True
_ctrlr_on = True
_previous_action = None

def _handle_motion_timer():
    global _set_speed
    if _set_speed:
        motion.drive(*Controller.velocities,smooth=_smooth)
        _set_speed = False

def _handle_odom_timer():
    if _odom_on:
        print "{}\r".format(Odometry.update(_odom_timer_period))

def _handle_ctrlr_timer():
    if _ctrlr_on:
        print "{}\r".format(Controller.update(_ctrlr_timer_period))

def main():
    global _motion_timer
    _motion_timer = RepeatedTimer(_motion_timer_period, _handle_motion_timer)
    _motion_timer.start()

    global _odom_timer
    _odom_timer = RepeatedTimer(_odom_timer_period, _handle_odom_timer)
    _odom_timer.start()

    global _ctrlr_timer
    _ctrlr_timer = RepeatedTimer(_ctrlr_timer_period, _handle_ctrlr_timer)
    _ctrlr_timer.start()

    w.init()
    print 'started'

    time.sleep(4)
    print 'Going to (1, 0, 0)'

    Controller.set_commanded_position(1, 0, 0)

    time.sleep(15)

    _ctrlr_timer.stop()
    _odom_timer.stop()
    _motion_timer.stop()

    sys.exit(0)


if __name__ == '__main__':
    main()
