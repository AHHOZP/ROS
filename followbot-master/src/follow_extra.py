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
        
        # Give some time to fill its buffer
        rospy.sleep(2)
        self.twist = Twist()
        self.h, self.w, self.d = self.image.shape
        self.template = []
        
        # load template
        for i in range(1,6):
#            aaa1 = numpy.load("red_mask_%s.npy"% i)
#            aaa2 = numpy.load("yellow_mask_%s.npy"% i)
#            aaa1 = numpy.array(aaa1,dtype=numpy.float64)
#            aaa2 = numpy.array(aaa2,dtype=numpy.float64)
#            self.template.append(aaa1+aaa2)
            aaa_r = cv2.imread("redred_mask_%s.jpg"% i)
            lower_red = numpy.array([0, 0, 1])
            upper_red = numpy.array([255, 255, 255])
            aaa = cv2.inRange( cv2.cvtColor(aaa_r, cv2.COLOR_BGR2HSV), lower_red, upper_red)
            self.template.append(aaa)
        self.turn_number = 0
        
        while not rospy.is_shutdown():
            mask_yellow, mask_yellow_t = self.generate_mask()
            res = cv2.matchTemplate(mask_yellow_t, self.template[self.turn_number], cv2.TM_CCOEFF_NORMED)
            res_max = numpy.max(res)
            print(res_max)

            # stop at RGB sign
            flag_red = res_max > 0.53
            #if(not flag_red):
            if(not flag_red):
                print("< 0.53")
                self.follow_yellow(mask_yellow)
            else:
                print(">= 0.53")
                # 1 turn left, -1 turn right
                rospy.sleep(0.1)
                self.twist = Twist() 
                self.twist.linear.x = 0.53
                while(flag_red):
                    mask_yellow, mask_yellow_t = self.generate_mask()
                    res = cv2.matchTemplate(mask_yellow_t, self.template[self.turn_number], cv2.TM_CCOEFF_NORMED)
                    res_max = numpy.max(res)
                    flag_red = res_max >0.53
                    rospy.sleep(0.1)
                for i in range(20):
                    mask_yellow, mask_red = self.generate_mask()
                    self.cmd_vel_pub.publish(self.twist)
                    cv2.imshow("window_yellow",  mask_yellow)
                    cv2.waitKey(3)
                    rospy.sleep(0.1)
                    
                self.twist = Twist()
                if(self.turn_number % 2 == 0):
                    self.twist.angular.z = 1.3
                elif(self.turn_number % 2 == 1):
                    self.twist.angular.z = -1.3
                print ("turn")
                self.cmd_vel_pub.publish(self.twist)
                rospy.sleep(1)
                self.turn_number += 1
                    
            # imshow
            cv2.imshow("window_yellow", mask_yellow)
            cv2.waitKey(3)               
        
    def image_callback(self,  msg):
        self.image = self.bridge.imgmsg_to_cv2(msg,desired_encoding='bgr8')
        
    def slice_mask(self, mask, t, b):
        search_top = 3*self.h/4 - t
        search_bot = 3*self.h/4 + b
        new_mask = mask.copy()
        new_mask[0:search_top, 0:self.w] = 0
        new_mask[search_bot:self.h, 0:self.w] = 0
        return new_mask
              
    def generate_mask(self):
        hsv = cv2.cvtColor(self.image, cv2.COLOR_BGR2HSV)
        lower_yellow = numpy.array([15,0,0])
        upper_yellow = numpy.array([36,255,255])
        mask_yellow = cv2.inRange(hsv, lower_yellow, upper_yellow)
        
        # only use a slice near 3/4 height
        # for turn
        mask_yellow_t = self.slice_mask(mask_yellow, 30, 60)
        # for follow
        mask_yellow = self.slice_mask(mask_yellow, 0, 20)
        
        return mask_yellow, mask_yellow_t
        
#    def red_center(self, mask_red):
#        M = cv2.moments(mask_red)
#        if M['m00']  >  0:
#            cx = int(M['m10']/M['m00'])
#            cy = int(M['m01']/M['m00'])
#            cv2.circle(mask_red,  (cx,  cy), 20, (0,0,255), -1)
#        left = 0
#        while(mask_red[:, left].sum() < 1000):
#            left += 1
#        right = self.w-1
#        while(mask_red[:, right].sum() < 1000):
#            right -= 1
#        cv2.circle(mask_red,  (left,  cy), 10, (0,0,255), -1)
#        cv2.circle(mask_red,  (right,  cy), 10, (0,0,255), -1)
#        cv2.imshow("window_red",  mask_red)
#        if(cx - left < right - cx):
#            print("right")
#            return -1
#        else:
#            print("left")
#            return 1
            
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

follower = Follower()
rospy.spin()
print("b")
