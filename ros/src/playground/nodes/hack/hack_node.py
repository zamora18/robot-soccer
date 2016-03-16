#!/usr/bin/env python

import roslib; roslib.load_manifest('playground')
import rospy
from geometry_msgs.msg import Pose2D
from playground.msg import RobotState

import numpy as np

robopub = None

def _handle_vision_robot_position(msg):
    robo_msg = RobotState()

    robo_msg.vision_x = msg.x
    robo_msg.vision_y = msg.y
    robo_msg.vision_theta = msg.theta
    robo_msg.correction = True

    robo_msg.xhat = msg.x
    robo_msg.yhat = msg.y
    robo_msg.thetahat = msg.theta

    robopub.publish(robo_msg)

def main():
    rospy.init_node('hack', anonymous=False)

    global robopub
    robopub = rospy.Publisher('robot_state', RobotState, queue_size=10)

    rospy.Subscriber('vision_ally_position', Pose2D, _handle_vision_robot_position)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    main()
