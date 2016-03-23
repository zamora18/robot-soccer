#!/usr/bin/env python
import os

import roslib; roslib.load_manifest('playground')
import rospy
from geometry_msgs.msg import Twist, Pose2D
from std_srvs.srv import Trigger, TriggerResponse
from playground.msg import EncoderEstimates, RobotState
from playground.srv import RoboClawRPC, RoboClawRPCResponse

import numpy as np

import motion
import wheelbase

_smooth = False
_theta = 0

# -----------------------------------------------------------------------------

def _handle_velocity_command(msg):
    velocities = (msg.linear.x, msg.linear.y, msg.angular.z)
    motion.drive(*velocities, smooth=_smooth, theta=_theta)

# -----------------------------------------------------------------------------

def _handle_theta(msg):
    global _theta
    _theta = msg.thetahat

# -----------------------------------------------------------------------------

def _get_battery_voltage(req):
    resp = wheelbase.ReadMainBatteryVoltage()
    return RoboClawRPCResponse(resp[0], str(resp[1]/10.0))

# -----------------------------------------------------------------------------

def _kick(req):
    os.system("echo 1 > /sys/class/gpio/gpio200/value; sleep .07; echo 0 > /sys/class/gpio/gpio200/value")
    return TriggerResponse(True, "GPIO actuated!")

# -----------------------------------------------------------------------------

def _shutdown_hook():
  print "Killing Robot"
  motion.kill()

# -----------------------------------------------------------------------------

def main():
    rospy.init_node('motion', anonymous=False)

    # Register a shutdown hook to kill motion
    rospy.on_shutdown(_shutdown_hook)

    rospy.Subscriber('vel_cmds', Twist, _handle_velocity_command)
    pub = rospy.Publisher('encoder_estimates', EncoderEstimates, queue_size=10)

    # Services
    rospy.Service('/motion/main_battery', RoboClawRPC, _get_battery_voltage)
    rospy.Service('/kick', Trigger, _kick)

    # So that we know the robot's theta
    rospy.Subscriber('robot_state', RobotState, _handle_theta)

    # Get this robot's motor's QPPS from rosparam
    m1 = rospy.get_param('M1QPPS', None)
    m2 = rospy.get_param('M2QPPS', None)
    m3 = rospy.get_param('M3QPPS', None)

    # init wheelbase
    wheelbase.init(m1qpps=m1, m2qpps=m2, m3qpps=m3)

    rospy.spin()
    return

    rate = rospy.Rate(10) # 100hz
    while not rospy.is_shutdown():


        (vx, vy, w, s1, s2, s3) = motion.get_velocities(theta=_theta)
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
