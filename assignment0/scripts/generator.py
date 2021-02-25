#!/usr/bin/env python

import rospy
from random import randint
from std_msgs.msg import Int8
from assignment0.msg import TwoInt

def generator():
    pub = rospy.Publisher('/numbers', TwoInt, queue_size=10)
    rospy.init_node('generator', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
	msg=TwoInt()
        msg.num1 = randint(0, 100)
	msg.num2 = randint(0, 100)
	pub.publish(msg)
	rospy.loginfo(msg)
        rate.sleep()

if __name__ == '__main__':
    try:
        generator()
    except rospy.ROSInterruptException:
        pass
