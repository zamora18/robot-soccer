#!/usr/bin/env python

import roslib; roslib.load_manifest('playground')
import rospy
from geometry_msgs.msg import Twist, Pose2D
from playground.msg import EncoderEstimates

import numpy as np

import motion
import wheelbase

_smooth = True
_theta = 0

def _handle_velocity_command(msg):
    # rospy.loginfo(rospy.get_caller_id() + "I heard (%s,%s,%s)", data.linear.x,data.linear.y,data.angular.z)

    # So every time I hear a message, update the motion?
    velocities = (msg.linear.x, msg.linear.y, msg.angular.z)
    motion.drive(*velocities, smooth=_smooth, theta=_theta)

def _handle_theta(msg):
    global _theta
    _theta = msg.theta

def main():
    rospy.init_node('motion', anonymous=False)

    rospy.Subscriber('vel_cmds', Twist, _handle_velocity_command)
    pub = rospy.Publisher('encoder_estimates', EncoderEstimates, queue_size=10)

    # Hack
    rospy.Subscriber('estimated_position', Pose2D, _handle_theta)

    # init wheelbase
    wheelbase.init()

    rospy.spin()
    return

    rate = rospy.Rate(10) # 100hz
    while not rospy.is_shutdown():


        (vx, vy, w, s1, s2, s3) = motion.get_velocities()
        estimate = EncoderEstimates()
        estimate.world_velocities.vx = vx
        estimate.world_velocities.vy = vy
        estimate.world_velocities.w  = w
        estimate.pulses.s1 = s1
        estimate.pulses.s2 = s2
        estimate.pulses.s3 = s3
        pub.publish(estimate)
            
        rate.sleep()

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    main()