#!/usr/bin/env python
import rospy
import random

from provider_power.msg import powerMsg


def sim_power():
    rospy.init_node('sim_power', anonymous=True)
    pub = rospy.Publisher('/provider_power/power', powerMsg, queue_size=10)
    rate = rospy.Rate(1)  # 10hz
    while not rospy.is_shutdown():
        msg = powerMsg()
        for i in range(0, 4):
            for j in range(0, 8):
                msg.slave = i
                msg.cmd = j
                msg.data = random.uniform(12.5, 17.3)
                pub.publish(msg)
        rate.sleep()

if __name__ == '__main__':
    try:
        sim_power()
    except rospy.ROSInterruptException:
        pass
