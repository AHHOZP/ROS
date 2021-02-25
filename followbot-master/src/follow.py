#!/usr/bin/env python
import rospy, cv2, cv_bridge, numpy
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
class Follower:
    def __init__(self):
    
        rospy.init_node('follower')
        
        # sub and pub message
        self.image_sub = rospy.Subscriber('camera/rgb/image_raw',Image, self.image_callback)
        self.cmd_vel_pub = rospy.Publisher('cmd_vel_mux/input/teleop',Twist, queue_size=1)
        
        # initialize
        self.bridge = cv_bridge.CvBridge()
        cv2.namedWindow("window_yellow", 1)
        cv2.namedWindow("window_yellow2", 1)
        
        # Give some time to fill its buffer
        rospy.sleep(2)
        self.twist = Twist()
        self.h, self.w, self.d = self.image.shape
        
        while not rospy.is_shutdown():
            mask_yellow, mask_yellow2 = self.generate_mask()
            print("yellow2:",mask_yellow2.sum())
            # stop at RGB sign
            stop_threshold = 6194130
            flag_turn = mask_yellow2.sum()>stop_threshold
            if(not flag_turn):
                print("< threshold")
                self.follow_yellow(mask_yellow)
            else:
                print(">= threshold")
                # 1 turn left, -1 turn right
                flag_turn = self.red_center(mask_yellow2)
                rospy.sleep(0.1)
                self.twist = Twist() 
                self.twist.linear.x = 0.5
                while(flag_turn):
                    mask_yellow, mask_yellow2 = self.generate_mask()
                    self.follow_yellow(mask_yellow)
                    flag_turn = mask_yellow2.sum()>stop_threshold
                    rospy.sleep(0.1)
                    
                self.twist = Twist()
                if(flag_turn == 1):
                    self.twist.angular.z = 1.3
                elif(flag_turn == -1):
                    self.twist.angular.z = -1.3
                print ("turn")
                self.cmd_vel_pub.publish(self.twist)
                rospy.sleep(1)
                    
            # imshow
            cv2.imshow("window_yellow",  mask_yellow)
            cv2.imshow("window_yellow2",  mask_yellow2)
            cv2.waitKey(3)               
        
    def image_callback(self,  msg):
        self.image = self.bridge.imgmsg_to_cv2(msg,desired_encoding='bgr8')
        
    def slice_mask(self, mask, t, b):
        search_top = 3*self.h/4 - t
        search_bot = 3*self.h/4 + b
        mask[0:search_top, 0:self.w] = 0
        mask[search_bot:self.h, 0:self.w] = 0
        return mask
              
    def generate_mask(self):
        hsv = cv2.cvtColor(self.image, cv2.COLOR_BGR2HSV)
        lower_yellow = numpy.array([15,0,0])
        upper_yellow = numpy.array([36,255,255])
        mask_yellow = cv2.inRange(hsv, lower_yellow, upper_yellow)
        mask_yellow2 = cv2.inRange(hsv, lower_yellow, upper_yellow)
        
        # only use 20 rows
        mask_yellow = self.slice_mask(mask_yellow, 0, 20)
        mask_yellow2 = self.slice_mask(mask_yellow2, 0, 80)
        mask_yellow2[:, 0:50] = 0
        mask_yellow2[:, self.w-50:self.w] = 0
        
        return mask_yellow, mask_yellow2
        
    def red_center(self, mask_red):
        M = cv2.moments(mask_red)
        if M['m00']  >  0:
            cx = int(M['m10']/M['m00'])
            cy = int(M['m01']/M['m00'])
            cv2.circle(mask_red,  (cx,  cy), 20, (0,0,255), -1)
        cv2.imshow("window_yellow2",  mask_red)
        if(cx < self.w/2):
            print("right")
            return -1
        else:
            print("left")
            return 1
            
#        search_left = 3*self.h/4 - t
#        search_right = 3*self.h/4 + b
#        mask[0:search_top, 0:self.w] = 0
#        mask[search_bot:self.h, 0:self.w] = 0
#        
#            err = cx - self.w/2
#            self.twist.linear.x = 0.5
#            self.twist.angular.z = -float(err) / 100
#            self.cmd_vel_pub.publish(self.twist) 
               
    def follow_yellow(self, mask_yellow):
        M = cv2.moments(mask_yellow)
        if M['m00']  >  0:
            cx = int(M['m10']/M['m00'])
            cy = int(M['m01']/M['m00'])
            cv2.circle(mask_yellow,  (cx,  cy), 20, (0,0,255), -1)
            err = cx - self.w/2
            self.twist.linear.x = 0.5
            self.twist.angular.z = -float(err) / 100
            self.cmd_vel_pub.publish(self.twist)
        
#    def stop_at_sign():
#        pass

follower  =  Follower()
rospy.spin()
print("b")
