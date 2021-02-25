#!/usr/bin/env python

import rospy
from std_msgs.msg import Int8
from std_msgs.msg import Int16
from assignment0.msg import TwoInt

def callback(data):
    summary = data.num1 + data.num2
    pub = rospy.Publisher('/sum', Int16, queue_size=10)
    pub.publish(data.num1+data.num2)
    rospy.loginfo(rospy.get_caller_id() + 'I recieved %s, and Sum = %s', data, summary)



def adder():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('adder', anonymous=True)

    rospy.Subscriber('/numbers', TwoInt, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    adder()
