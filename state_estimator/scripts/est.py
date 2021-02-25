#!/usr/bin/env python

# Columbia Engineering
# MECS 4602 - Fall 2018

import math
import numpy
import time

import rospy

from state_estimator.msg import RobotPose
from state_estimator.msg import SensorData

class Estimator(object):
    def __init__(self):

        # Publisher to publish state estimate
        self.pub_est = rospy.Publisher("/robot_pose_estimate", RobotPose, queue_size=1)

        # Initial estimates for the state and the covariance matrix
        self.x = numpy.zeros((3,1))
        self.P = numpy.zeros((3,3))

        # Covariance matrix for process (model) noise
        self.V = numpy.zeros((3,3))
        self.V[0,0] = 0.0025
        self.V[1,1] = 0.0025
        self.V[2,2] = 0.005

        self.step_size = 0.01

        # Subscribe to command input and sensory output of robot
        rospy.Subscriber("/sensor_data", SensorData, self.sensor_callback)
        
    # This function gets called every time the robot publishes its control 
    # input and sensory output. You must make use of what you know about 
    # extended Kalman filters to come up with an estimate of the current
    # state of the robot and covariance matrix.
    # The SensorData message contains fields 'vel_trans' and 'vel_ang' for
    # the commanded translational and rotational velocity respectively. 
    # Furthermore, it contains a list 'readings' of the landmarks the
    # robot can currently observe
    def estimate(self, sens):
        # get u command sent to robot
        vel_t = sens.vel_trans
        vel_a = sens.vel_ang

        #initial white noise
        w0 = numpy.array([[.1,0],[0,0.05]])
        w1 = numpy.array([[.1,0,0,0],[0,0.05,0,0],[0,0,.1,0],[0,0,0,0.05]])
        w2 = numpy.array([[.1,0,0,0,0,0],[0,0.05,0,0,0,0],[0,0,.1,0,0,0],[0,0,0,0.05,0,0],[0,0,0,0,.1,0],[0,0,0,0,0,0.05]])
        W = [w0,w1,w2]

        # x_pred = f(x,u)
        self.x[0,0] = self.x[0,0] + vel_t * self.step_size * math.cos(self.x[2,0])
        self.x[1,0] = self.x[1,0] + vel_t * self.step_size * math.sin(self.x[2,0])
        self.x[2,0] = self.x[2,0] + vel_a * self.step_size

        #F = df/dxdydtheta
        dis_x = - vel_t * self.step_size * math.sin(self.x[2,0])
        dis_y =   vel_t * self.step_size * math.cos(self.x[2,0])
        self.F = numpy.array([[1,0,dis_x],
                             [0,1,dis_y],
                             [0,0,1]])

        # P_pred = FPFT+V
        self.P = numpy.dot(self.F, numpy.dot(self.P, numpy.transpose(self.F))) + self.V

        # number of landmarks
        n = len(sens.readings)
        # initialize list
        H_list = []

        # get H and innov
        if n == 0:
            pass
        else:
            Innov = numpy.array([])
            k = 0
            for i in range(n):
                # get sensor data
                lm_x = sens.readings[i].landmark.x
                lm_y = sens.readings[i].landmark.y
                lm_range = sens.readings[i].range
                lm_bearing = sens.readings[i].bearing
                y_sen = numpy.array([lm_range,lm_bearing])

                est_range = math.sqrt(( self.x[0,0]-lm_x)**2 + ( self.x[1,0]-lm_y)**2)
                est_bearing = math.atan2(lm_y -  self.x[1,0], lm_x -  self.x[0,0]) -  self.x[2,0]

                if est_range < 0.1:
                    pass
                else:
                    hx = numpy.array([est_range,est_bearing])

                    innov = y_sen - hx

                    while innov[1] >=  math.pi:
                        innov[1] = innov[1] - 2 * math.pi
                    while innov[1] <= - math.pi:
                        innov[1] = innov[1] + 2 * math.pi

                    Innov = numpy.append(Innov,innov[0])
                    Innov = numpy.append(Innov,innov[1])


                    d_range_dx = (self.x[0,0]-lm_x)/est_range
                    d_range_dy = (self.x[1,0]-lm_y)/est_range
                    d_bearing_dx = (lm_y-self.x[1,0])/est_range**2
                    d_bearing_dy = -(lm_x-self.x[0,0])/est_range**2
                    H = numpy.array([[d_range_dx, d_range_dy, 0], [d_bearing_dx, d_bearing_dy, -1]])

                    H_list.append(H)
                    self.W = W[k]
                    if k == 0 :
                        self.H = H_list[k]
                    else:
                        self.H = numpy.vstack((self.H, H_list[k]))
                    k = k + 1
            if k == 0:
                pass
            else:
                # S = HPHT + W
                S = numpy.dot(self.H, numpy.dot(self.P, numpy.transpose(self.H))) + self.W

                # R = PHT/S
                R = numpy.dot(numpy.dot(self.P,numpy.transpose(self.H)),numpy.linalg.inv(S))

                # X = X + R * innov
                XR = numpy.dot(R, Innov)
                self.x[0,0] = self.x[0,0] + XR[0]
                self.x[1,0] = self.x[1,0] + XR[1]
                self.x[2,0] = self.x[2,0] + XR[2]

                # P = P - RHP
                self.P = self.P - numpy.dot(numpy.dot(R, self.H),self.P)

    
    def sensor_callback(self,sens):

        # Publish state estimate 
        self.estimate(sens)
        est_msg = RobotPose()
        est_msg.header.stamp = sens.header.stamp
        est_msg.pose.x = self.x[0]
        est_msg.pose.y = self.x[1]
        est_msg.pose.theta = self.x[2]
        self.pub_est.publish(est_msg)

if __name__ == '__main__':
    rospy.init_node('state_estimator', anonymous=True)
    est = Estimator()
    rospy.spin()
