#!/usr/bin/env python
""" odom_bug2.py 2019-10-06
    A simulation of bug2 algorithm
    Created for COMS 4733 lab2 by Ruoran Shi and Zhenhao Li
"""
"""    
    odom_out_and_back.py - Version 1.1 2013-12-20

    A basic demo of using the /odom topic to move a robot a given distance
    or rotate through a given angle.

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
from geometry_msgs.msg import Twist, Point, Quaternion
import tf
from rbx1_nav.transform_utils import quat_to_angle, normalize_angle
from math import radians, copysign, sqrt, pow, pi
from sensor_msgs.msg import LaserScan
import numpy

class OutAndBack():              
    def __init__(self):
        # Give the node a name
        rospy.init_node('odom_bug2', anonymous=False)

        # Set rospy to execute a shutdown function when exiting       
        rospy.on_shutdown(self.shutdown)
        
        # Publisher to control the robot's speed
        self.cmd_vel = rospy.Publisher('/cmd_vel_mux/input/teleop', Twist, queue_size=5)
        
        # Laser
        self.range_ahead=None
        self.scan_sub = rospy.Subscriber('/scan', LaserScan, self.scan_callback)

        # Initialize the tf listener
        self.tf_listener = tf.TransformListener()
        
        # Give tf some time to fill its buffer
        rospy.sleep(2)
        
        # Set the odom frame
        self.odom_frame = '/odom'
        
        # Find out if the robot uses /base_link or /base_footprint
        try:
            self.tf_listener.waitForTransform(self.odom_frame, '/base_footprint', rospy.Time(), rospy.Duration(1.0))
            self.base_frame = '/base_footprint'
        except (tf.Exception, tf.ConnectivityException, tf.LookupException):
            try:
                self.tf_listener.waitForTransform(self.odom_frame, '/base_link', rospy.Time(), rospy.Duration(1.0))
                self.base_frame = '/base_link'
            except (tf.Exception, tf.ConnectivityException, tf.LookupException):
                rospy.loginfo("Cannot find transform between /odom and /base_link or /base_footprint")
                rospy.signal_shutdown("tf Exception")  
                    
        # Enter the loop
        flag = 1
        while not self.goal_reached() and not rospy.is_shutdown():
            self.follow_mline()
            self.move_forward()
            if(self.range_ahead <1):
                print "obstacle detected"
                self.stop()
                if(self.follow_edge() == 0):
                    flag = 0
                    break
                else:
                    self.turn_back()
        if(flag==1):            
            print("GOAL!")
                                
        # Stop the robot for good
        self.cmd_vel.publish(Twist())
                  
    def get_odom(self):
        # Get the current transform between the odom and base frames
        try:
            (trans, rot)  = self.tf_listener.lookupTransform(self.odom_frame, self.base_frame, rospy.Time(0))
        except (tf.Exception, tf.ConnectivityException, tf.LookupException):
            rospy.loginfo("TF Exception")
            return

        return (Point(*trans), quat_to_angle(Quaternion(*rot)))
            
    def scan_callback(self, msg):
        self.range_ahead = numpy.mean(msg.ranges[len(msg.ranges)/2-2:len(msg.ranges)/2+3])
        self.range_right = numpy.mean(msg.ranges[0:4])
        
    def stop(self):
        move_cmd = Twist()
        self.cmd_vel.publish(move_cmd)
        rospy.sleep(0.5)
        
    def obstacle_right(self):
        if(numpy.isnan(self.range_right) or self.range_right>2):
            return 0
        else:
            return 1
            
    def move_forward(self):
        move_t = Twist()
        move_t.linear.x = 0.29
        self.cmd_vel.publish(move_t)
        rospy.sleep(0.5)
  
    def turn_right(self):
        move_r = Twist()
        move_r.angular.z = -0.23
        self.cmd_vel.publish(move_r)
        rospy.sleep(0.5)
        
    def turn_left(self):
        move_r = Twist()
        if(self.range_ahead<1):
            self.cmd_vel.publish(move_r)
            rospy.sleep(0.5)
            move_r.angular.z = pi/2
            self.cmd_vel.publish(move_r)
            rospy.sleep(0.5)
        elif(self.obstacle_right()):
            move_r.angular.z = 0.23
            self.cmd_vel.publish(move_r)
            rospy.sleep(0.5)
        
    def turn_back(self):
        print("turning back...")
        move_r = Twist()
        (cur_position, cur_rotation) = self.get_odom()
        move_r.angular.z = -cur_rotation
        self.cmd_vel.publish(move_r)
        rospy.sleep(0.5)
        (cur_position, cur_rotation) = self.get_odom()
        if abs(cur_rotation) > 0.5:
            move_r.angular.z = 0.5
            self.cmd_vel.publish(move_r)
            rospy.sleep(0.5)
            (cur_position, cur_rotation) = self.get_odom()
        while abs(cur_rotation) > 0.1:
            move_r.angular.z = 0.1
            self.cmd_vel.publish(move_r)
            rospy.sleep(0.5)
            (cur_position, cur_rotation) = self.get_odom()
        if abs(cur_rotation) > 0.05:
            move_r.angular.z = 0.07
            self.cmd_vel.publish(move_r)
            rospy.sleep(0.5)
            (cur_position, cur_rotation) = self.get_odom()
                        
    def follow_edge(self):
        # Store hit point
        (hit_position, hit_rotation) = self.get_odom()
        print("hit obstacle at:", hit_position.x, hit_position.y)
        # Keep track of the distance traveled
        distance = 0
        # Turn left until the object is no longer detected to the right of the robot
        self.turn_left()
        while(self.obstacle_right()):
            self.turn_left()
        #Follow the obstacle as below until m-line reached or hit point reached
        (cur_position, cur_rotation) = self.get_odom()
        while(1):
            #Move forward a small distance
            self.move_forward()
            (cur_position, cur_rotation) = self.get_odom()
            distance = distance + sqrt(pow((cur_position.x - hit_position.x), 2) + pow((cur_position.y - hit_position.y), 2))
            #If object not detected on right, turn slightly right
            if(not self.obstacle_right()):
                self.turn_right()
            #If object close on right, turn left and move forward    
            else:
                self.turn_left()
                self.move_forward()
                (cur_position, cur_rotation) = self.get_odom()
                distance = distance + sqrt(pow((cur_position.x - hit_position.x), 2) + pow((cur_position.y - hit_position.y), 2))
            #If hit point reached conclude impossible
            if(abs(cur_position.x-hit_position.x)<0.5 and abs(cur_position.y-hit_position.y)<0.1 and distance>10):
                print("impossible")
                return 0
            #If m-line reached
            if(abs(cur_position.y)<0.1 and cur_position.x < 10.1 and cur_position.x-hit_position.x>1):
                (cur_position, cur_rotation) = self.get_odom()
                print("m-line reached at:", cur_position.x, cur_position.y)
                return 1
                
    def goal_reached(self):     
        (position, rotation) = self.get_odom()
        x_target = 10
        y_target = 0
        if(abs(position.x-x_target)<0.1 and abs(position.y-y_target)<0.1):
            return 1
        else:
            return 0   
             
    def follow_mline(self):
        move_r = Twist()
        (cur_position, cur_rotation) = self.get_odom()
        if cur_position.y>0.05:
            move_r.angular.z = -0.1
            self.cmd_vel.publish(move_r)
            rospy.sleep(1)
        elif cur_position.y<-0.05:
            move_r.angular.z = 0.1
            self.cmd_vel.publish(move_r)
            rospy.sleep(1)
                
    def shutdown(self):
        # Always stop the robot when shutting down the node.
        rospy.loginfo("Stopping the robot...")
        self.cmd_vel.publish(Twist())
        rospy.sleep(0.5)
 
if __name__ == '__main__':
    try:
        OutAndBack()
    except:
        rospy.loginfo("Out-and-Back node terminated.")

