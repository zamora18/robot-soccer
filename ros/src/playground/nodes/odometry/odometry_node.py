#!/usr/bin/env python

import roslib; roslib.load_manifest('playground')
import rospy
from geometry_msgs.msg import Pose2D
from playground.msg import EncoderEstimates

import numpy as np

import Odometry

_odom_period = 1.0/100

# (vx, vy, w, s1, s2, s3)
_velocities = (0, 0, 0, 0, 0, 0)

def _handle_encoder_estimates(msg):
    # rospy.loginfo(rospy.get_caller_id() + "I heard (%s,%s,%s)", data.linear.x,data.linear.y,data.angular.z)
    global _velocities

    vx = msg.world_velocities.vx
    vy = msg.world_velocities.vy
    w  = msg.world_velocities.w
    s1 = msg.pulses.s1
    s2 = msg.pulses.s2
    s3 = msg.pulses.s3

    _velocities = (vx, vy, w, s1, s2, s3)


def main():
    rospy.init_node('odometry', anonymous=False)

    rospy.Subscriber('encoder_estimates', EncoderEstimates, _handle_encoder_estimates)
    pub = rospy.Publisher('estimated_position', Pose2D, queue_size=10)

    rate = rospy.Rate(int(1/_odom_period))
    while not rospy.is_shutdown():

        (xhat, yhat, thetahat) = Odometry.update(_odom_period, _velocities)

        msg = Pose2D()
        msg.x = xhat
        msg.y = yhat
        msg.theta = thetahat

        pub.publish(msg)

        rate.sleep()

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    main()
