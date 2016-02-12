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

_motion_timer_period = 1.0/3
_odom_timer_period = 1.0/6
_ctrlr_timer_period = 1.0/10

_set_speed = False

_smooth = False
_odom_on = True
_ctrlr_on = True
_previous_action = None

def _handle_motion_timer():
    global _set_speed
    if _set_speed:
#        vx, vy, w = Controller.velocities
#        print("*** Setting speed: {},{},{}\r".format(vx,vy,w))
        motion.drive(*Controller.velocities,smooth=_smooth)
#        _set_speed = False

def _handle_odom_timer():
    if _odom_on:
        print "{}\r".format(Odometry.update(_odom_timer_period))

def _handle_ctrlr_timer():
    if _ctrlr_on:
        Controller.update(_ctrlr_timer_period)
        global _set_speed
        _set_speed = True
        #print "{}\r".format(Controller.update(_ctrlr_timer_period))

def main():
    w.init()
    motion.stop()

    time.sleep(1)



    global _motion_timer
    _motion_timer = RepeatedTimer(_motion_timer_period, _handle_motion_timer)
    _motion_timer.start()

    global _odom_timer
    _odom_timer = RepeatedTimer(_odom_timer_period, _handle_odom_timer)
    _odom_timer.start()

    global _ctrlr_timer
    _ctrlr_timer = RepeatedTimer(_ctrlr_timer_period, _handle_ctrlr_timer)
    _ctrlr_timer.start()

    print 'started'

    time.sleep(4)
    print '\r\n**** Going to (0.5, 0, 0) ****\r\n'

    Controller.set_commanded_position(0.5, 0, 0)

    time.sleep(15)

    _ctrlr_timer.stop()
    _odom_timer.stop()
    _motion_timer.stop()

    motion.stop()

    sys.exit(0)


if __name__ == '__main__':
    main()
