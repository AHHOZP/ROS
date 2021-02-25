#!/usr/bin/env python
""" timed_out_and_back.py - Version 1.3 2019-09-22
	Created for COMS 4733 lab1 by Ruoran Shi and Zhenhao Li
"""

""" timed_out_and_back.py - Version 1.2 2014-12-14

    A basic demo of the using odometry data to move the robot along
    and out-and-back trajectory.

    Created for the Pi Robot Project: http://www.pirobot.org
    Copyright (c) 2012 Patrick Goebel.  All rights reserved.

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.5
    
    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details at:
    
    http://www.gnu.org/licenses/gpl.html
      
"""

import rospy
from geometry_msgs.msg import Twist
from math import pi

class OutAndBack():
    def __init__(self):
        # Give the node a name
        rospy.init_node('out_and_back', anonymous=False)
        # Set rospy to execute a shutdown function when exiting
        rospy.on_shutdown(self.shutdown)
        
        Q,q,R,r,T,t = 'Q','q','R','r','T','t'
        
        # Publisher to control the robot's speed
        self.cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        
        # How fast will we update the robot's movement?
        rate = 50
        
        # Set the equivalent ROS rate variable
        ros_rate = rospy.Rate(rate)
        
        # Set the linear speed to 0.2 meters per second 
        linear_speed = 0.2
        linear_speedMinus = -0.2
        
        # Set the rotation speed to 1.0 radians per second
        angular_speed = 1.0
        angular_speedMinus = -1.0
        
        while 1:
            try:
                Cmd = input("Please input operation command. T for translation, R for rotation, or Q for quit:")
                if Cmd == 'T' or Cmd == 't':
                    # Set the travel distance
                    goal_distance = input("Please input number of distance to translate: ")
                    move_cmd = Twist()
                    # How long should it take us to get there?
                    if(goal_distance<0):
                        linear_duration = goal_distance / linear_speedMinus
                        move_cmd.linear.x = linear_speedMinus
                    else:
                        linear_duration = goal_distance / linear_speed
                        move_cmd.linear.x = linear_speed        
                    ticks = int(linear_duration * rate)
                    for i in range(ticks):
                        self.cmd_vel.publish(move_cmd)
                        ros_rate.sleep()
                    move_cmd = Twist()
                    self.cmd_vel.publish(move_cmd)
                    rospy.sleep(1)
                
                elif Cmd == 'R' or Cmd == 'r':
                    # Set the rotation angle
                    goal_angle = input("Please input number of angle(Deg) to rotate: ")
                    move_cmd = Twist()
                    # How long should it take to rotate?
                    if(goal_angle<0):
                        angular_duration = (goal_angle * pi / 180) / angular_speedMinus
                        move_cmd.angular.z = angular_speedMinus
                    else:
                        angular_duration = (goal_angle * pi / 180) / angular_speed
                        move_cmd.angular.z = angular_speed   
                    ticks = int(angular_duration * rate)
                    for i in range(ticks):
                        self.cmd_vel.publish(move_cmd)
                        ros_rate.sleep()
                    move_cmd = Twist()
                    self.cmd_vel.publish(move_cmd)
                    rospy.sleep(1)

                elif Cmd == 'Q' or Cmd == 'q':
                    break
                else:
                	print "Invalid input"
            except:
                print "Invalid input"
                continue
            
        # Stop the robot
        self.cmd_vel.publish(Twist())
        
    def shutdown(self):
        # Always stop the robot when shutting down the node.
        rospy.loginfo("Stopping the robot...")
        self.cmd_vel.publish(Twist())
        rospy.sleep(1)
 
if __name__ == '__main__':
    try:
        OutAndBack()
    except:
        rospy.loginfo("Out-and-Back node terminated.")

