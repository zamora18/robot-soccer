#!/usr/bin/env python

import rospy
from playground.msg import coords

import velchange as v

_x = 0
_y = 0
_theta = 0

def callback(data):
    #rospy.loginfo(rospy.get_caller_id() + "I heard (%s,%s,%s)", data.robot_x,data.robot_y,data.robot_theta)

    global _x
    global _y
    global _theta

    _x = data.robot_x
    _y = data.robot_y
    _theta = data.robot_theta


def _isClose(x, val):
    TOLERANCE = 2.0
    return ( abs(x-val) <= TOLERANCE )


def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # node are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'talker' node so that multiple talkers can
    # run simultaneously.
    rospy.init_node('listener', anonymous=True)

    rospy.Subscriber("vision", coords, callback)

    rate = rospy.Rate(100) # 100hz
    while not rospy.is_shutdown():

        if not _isClose(_theta, 0):
            v.goXYOmega(0, 0, 1.75)
    
        elif not _isClose(_x, 0) or not _isClose(_y, 0):
            v.goXYOmega(x, y, 0)

        else:
            v.goXYOmega(0, 0, 0)
            
        rate.sleep()

    # spin() simply keeps python from exiting until this node is stopped
    # rospy.spin()

if __name__ == '__main__':
    listener()
