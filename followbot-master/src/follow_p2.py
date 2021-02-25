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
        cv2.namedWindow("window_green", 1)
        cv2.namedWindow("window_blue", 1)
        cv2.namedWindow("window_red", 1)
        
        # Give some time to fill its buffer
        rospy.sleep(2)
        self.twist = Twist()
        self.h, self.w, self.d = self.image.shape
        while not rospy.is_shutdown():
            mask_yellow, mask_green, mask_blue, mask_red = self.generate_mask()
            # stop at RGB sign
            stop_threshold = 1000000
            print("green:", mask_green.sum())
            flag_green = mask_green.sum()>stop_threshold
            flag_blue = mask_blue.sum()>stop_threshold
            flag_red = mask_red.sum()>stop_threshold
            if(not(flag_green or flag_blue or flag_red)):
                print("< threshold")
                self.follow_yellow(mask_yellow)
            else:
                print(">= threshold")
                self.twist = Twist() 
                self.twist.linear.x = 0.5
                for i in range(18):
                    mask_yellow, mask_green, mask_blue, mask_red = self.generate_mask()
                    self.cmd_vel_pub.publish(self.twist)
                    cv2.imshow("window_yellow",  mask_yellow)
                    cv2.imshow("window_green",  mask_green)
                    cv2.imshow("window_blue",  mask_blue)
                    cv2.imshow("window_red",  mask_red)
                    cv2.waitKey(3)
                    rospy.sleep(0.1)
                    
                self.twist = Twist() 
                if(flag_green):
                    self.twist.angular.z = 1.3
                elif(flag_blue):
                    self.twist.angular.z = -1.3
                elif(flag_red):
                    break
                self.cmd_vel_pub.publish(self.twist)
                rospy.sleep(1)
            # imshow
            cv2.imshow("window_yellow",  mask_yellow)
            cv2.imshow("window_green",  mask_green)
            cv2.imshow("window_blue",  mask_blue)
            cv2.imshow("window_red",  mask_red)
            cv2.waitKey(3)               
                    
            
            # stop at RGB sign
#        marker = 0
#        stop_threshold = 1000000
#        #print("green:", mask_green.sum())
#        if(mask_green.sum()<stop_threshold and mask_blue.sum()<stop_threshold and mask_red.sum()<stop_threshold):
#            #print("< threshold")

#            
#            if marker == 1:
#                self.twist = Twist()
#                self.twist.angular.z = 1.0
#                for i in range(5):
#                    self.cmd_vel_pub.publish(self.twist)
#                    rospy.sleep(0.1)
#            
#            else:
#                self.follow_yellow(mask_yellow, w)        
#            
#            marker = 0
#            
#        else:
#            #print(">= threshold")
#            self.twist = Twist()
#            self.twist.linear.x = 0.2
#            #for i in range(5):
#            self.cmd_vel_pub.publish(self.twist)
#            rospy.sleep(0.1)
#            # self.twist.angular.z = 
#            #print(self.twist)
#            #self.twist = Twist()
#            #self.cmd_vel_pub.publish(self.twist)
#            #rospy.sleep(0.1)
#            marker = 1

        
    def image_callback(self,  msg):
        self.image = self.bridge.imgmsg_to_cv2(msg,desired_encoding='bgr8')
        
    def slice_mask(self, mask):
        search_top = 3*self.h/4
        search_bot = 3*self.h/4 + 20
        mask[0:search_top, 0:self.w] = 0
        mask[search_bot:self.h, 0:self.w] = 0
        return mask
              
    def generate_mask(self):
        hsv = cv2.cvtColor(self.image, cv2.COLOR_BGR2HSV)
        #lower_yellow = numpy.array([10,10,10])
        #upper_yellow = numpy.array([255,255,250])
        lower_yellow = numpy.array([15,0,0])
        upper_yellow = numpy.array([36,255,255])
        lower_green = numpy.array([36,0,0])
        upper_green = numpy.array([70,255,250])
        lower_blue = numpy.array([90,0,0])
        upper_blue = numpy.array([140,255,250])
        lower_red = numpy.array([0,20,70])
        upper_red = numpy.array([10,255,250])
        mask_yellow = cv2.inRange(hsv, lower_yellow, upper_yellow)
        mask_green = cv2.inRange(hsv, lower_green, upper_green)
        mask_blue = cv2.inRange(hsv, lower_blue, upper_blue)
        mask_red = cv2.inRange(hsv, lower_red, upper_red)
        
        # only use 20 rows
        mask_yellow = self.slice_mask(mask_yellow)
        mask_green = self.slice_mask(mask_green)
        mask_blue = self.slice_mask(mask_blue)
        mask_red = self.slice_mask(mask_red)
        
        return mask_yellow, mask_green, mask_blue, mask_red
        
    def follow_yellow(self, mask_yellow):
        M = cv2.moments(mask_yellow)
        if M['m00']  >  0:
            cx = int(M['m10']/M['m00'])
            cy = int(M['m01']/M['m00'])
            cv2.circle(mask_yellow,  (cx,  cy), 20, (0,0,255), -1)
            err = cx - self.w/2
            self.twist.linear.x = 0.2
            self.twist.angular.z = -float(err) / 100
            self.cmd_vel_pub.publish(self.twist)
        
#    def stop_at_sign():
#        pass

follower  =  Follower()
rospy.spin()
print("b")
