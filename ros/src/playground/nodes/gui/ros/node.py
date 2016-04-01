#!/usr/bin/env python
import rospy
import random
from std_msgs.msg import Int8


from playground.srv import RoboClawRPC, RoboClawRPCResponse

def _get_battery_voltage(req):
    resp = 160
    return RoboClawRPCResponse(True, str(resp/10.0))


def talker():
    pub = rospy.Publisher('random_value', Int8)


    rospy.Service('/ally1/battery', RoboClawRPC, _get_battery_voltage)


    rospy.init_node('random_value')
    while not rospy.is_shutdown():
        pub.publish(random.random()*10)
        rospy.sleep(0.1)

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException: pass