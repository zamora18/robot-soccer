#!/usr/bin/env python

# Delete this file

import rospy
from playground.msg import coords
# from rcv3 import roboclaw as r
# import velchange as v

import motion
import wheelbase as w

import numpy as np

_x = 0
_y = 0
_theta = 0



_state = 'SPIN' # 'SQUARE', 'HOME'


def spin(speed):
    print "spinning!"
    motion.drive(0,0,speed)
	# r.Open('/dev/ttySAC0',38400)
	# r.SpeedM1(0x80, speed)
	# r.SpeedM2(0x80, -speed)
	# r.SpeedM1(0x81, speed)


def callback(data):
    #rospy.loginfo(rospy.get_caller_id() + "I heard (%s,%s,%s)", data.robot_x,data.robot_y,data.robot_theta)

    global _x
    global _y
    global _theta

    _x = data.robot_x
    _y = data.robot_y
    _theta = data.robot_theta


def _isClose(x, val, tolerance=15):
    return ( abs(x-val) <= tolerance )


def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # node are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'talker' node so that multiple talkers can
    # run simultaneously.
    rospy.init_node('listener', anonymous=True)

    rospy.Subscriber("vision", coords, callback)

    counter = 0

    global _state

    # init wheelbase
    w.init()

    rate = rospy.Rate(100) # 100hz
    while not rospy.is_shutdown():

        counter = counter+1

	print _state

        if _state == 'SPIN':
            if counter == 1:
            	spin(2*np.pi)

            if counter == 300:
                counter = 0
		spin(0)
                _state = 'SLEEP1'

        elif _state == 'SLEEP1':
            if counter == 200:
                counter = 0
                _state = 'SQUARE1'




        # ---------------------------
        # Sqauares


        elif _state == 'SQUARE1':
            motion.drive(.5,0,0)

            if counter == 200:
                counter = 0
                _state = 'SQUARE2'

        elif _state == 'SQUARE2':
            motion.drive(0,.5,0)

            if counter == 200:
                counter = 0
                _state = 'SQUARE3'

        elif _state == 'SQUARE3':
            motion.drive(-0.5,0,0)

            if counter == 200:
                counter = 0
                _state = 'SQUARE4'

        elif _state == 'SQUARE4':
            motion.drive(0,-.5,0)

            if counter == 200:
                counter = 0
		motion.stop()
                _state = 'SLEEP2'

        # ---------------------------

        elif _state == 'SLEEP2':
            if counter == 200:
                counter = 0
                _state = 'HOME'
            
        elif _state == 'HOME':
            if not _isClose(_theta, 180, tolerance=60):
                print "going to 0 theta"
                motion.drive(0,0,np.pi)
                # v.goXYOmega(0, 0, 3)
        
            elif not _isClose(_x, 0) or not _isClose(_y, 0):
                print "going home"
                sign_x = -1 if _x<0 else 1
                sign_y = -1 if _y<0 else 1

                vx = 1 if abs(_x)>abs(_y) else abs(_x)/abs(_y)
                vy = 1 if abs(_y)>abs(_x) else abs(_y)/abs(_x)

                vx = -1*sign_x*vx
                vy = -1*sign_y*vy

		scale = 0.5

                # v.goXYOmega(vx, vy, 0)
                motion.drive(-vx/scale,-vy/scale,0)

            else:
                # v.goXYOmega(0, 0, 0)
                motion.stop()
		break
            
        rate.sleep()

    # spin() simply keeps python from exiting until this node is stopped
    # rospy.spin()

if __name__ == '__main__':
    listener()
