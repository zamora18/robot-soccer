#!/usr/bin/env python

import roslib; roslib.load_manifest('playground')
import rospy
from geometry_msgs.msg import Pose2D
from playground.msg import coords

import numpy as np

_hack_period = 1.0/100

_ball_x = 0
_ball_y = 0

robopub = None

def _handle_vision_coords(msg):
    # rospy.loginfo(rospy.get_caller_id() + "I heard (%s,%s,%s)", data.linear.x,data.linear.y,data.angular.z)
    global _ball_x, _ball_y
    _ball_x = msg.ball_x/100.0
    _ball_y = msg.ball_y/100.0


    robo_msg = Pose2D()
    robo_msg.x = msg.robot_x/100.0
    robo_msg.y = msg.robot_y/100.0
    robo_msg.theta = msg.robot_theta*(np.pi/180.0)
    robopub.publish(robo_msg)

def main():
    rospy.init_node('hack', anonymous=False)

    # Don't use odom node with this
    global robopub
    robopub = rospy.Publisher('estimated_position', Pose2D, queue_size=10)

    rospy.Subscriber('vision', coords, _handle_vision_coords)
    pub = rospy.Publisher('desired_position', Pose2D, queue_size=10)

    rate = rospy.Rate(int(1/_hack_period))
    while not rospy.is_shutdown():

        msg = Pose2D()
        msg.x = _ball_x
        msg.y = _ball_y
        msg.theta = 0

        pub.publish(msg)

        rate.sleep()

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    main()
