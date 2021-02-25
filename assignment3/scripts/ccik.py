#!/usr/bin/env python

import math
import numpy
import rospy
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Transform
from cartesian_control.msg import CartesianCommand
from urdf_parser_py.urdf import URDF
import random
import tf
from threading import Thread, Lock
import time

'''This is a class which will perform both cartesian control and inverse
   kinematics'''
class CCIK(object):
    def __init__(self):
    #Load robot from parameter server
        self.robot = URDF.from_parameter_server()

    #Subscribe to current joint state of the robot
        rospy.Subscriber('/joint_states', JointState, self.get_joint_state)

    #This will load information about the joints of the robot
        self.num_joints = 0
        self.joint_names = []
        self.q_current = []
        self.joint_axes = []
        self.get_joint_info()

    #This is a mutex
        self.mutex = Lock()

    #Subscribers and publishers for for cartesian control
        rospy.Subscriber('/cartesian_command', CartesianCommand, self.get_cartesian_command)
        self.velocity_pub = rospy.Publisher('/joint_velocities', JointState, queue_size=10)
        self.joint_velocity_msg = JointState()

        #Subscribers and publishers for numerical IK
        rospy.Subscriber('/ik_command', Transform, self.get_ik_command)
        self.joint_command_pub = rospy.Publisher('/joint_command', JointState, queue_size=10)
        self.joint_command_msg = JointState()

    '''This is a function which will collect information about the robot which
       has been loaded from the parameter server. It will populate the variables
       self.num_joints (the number of joints), self.joint_names and
       self.joint_axes (the axes around which the joints rotate)'''
    def get_joint_info(self):
        link = self.robot.get_root()
        while True:
            if link not in self.robot.child_map: break
            (joint_name, next_link) = self.robot.child_map[link][0]
            current_joint = self.robot.joint_map[joint_name]
            if current_joint.type != 'fixed':
                self.num_joints = self.num_joints + 1
                self.joint_names.append(current_joint.name)
                self.joint_axes.append(current_joint.axis)
            link = next_link

    '''This is the callback which will be executed when the cartesian control
       recieves a new command. The command will contain information about the
       secondary objective and the target q0. At the end of this callback, 
       you should publish to the /joint_velocities topic.'''
    def get_cartesian_command(self, command):
        self.mutex.acquire()
        #--------------------------------------------------------------------------
        #FILL IN YOUR PART OF THE CODE FOR CARTESIAN CONTROL HERE
        tx = command.x_target.translation.x
        ty = command.x_target.translation.y
        tz = command.x_target.translation.z
        rx = command.x_target.rotation.x
        ry = command.x_target.rotation.y
        rz = command.x_target.rotation.z
        rw = command.x_target.rotation.w
        so = command.secondary_objective
        q0 = command.q0_target

        joint_transforms, b_T_ee_cur = self.forward_kinematics(self.q_current)
        Tl_des = tf.transformations.translation_matrix((tx,ty,tz))
        Ro_des = tf.transformations.quaternion_matrix([rx,ry,rz,rw])
        b_T_ee_des = numpy.dot(Tl_des, Ro_des)
        X = tf.transformations.translation_from_matrix(b_T_ee_cur)
        ee_T_b_cur = numpy.linalg.inv(b_T_ee_cur)
        change_M = numpy.dot(ee_T_b_cur,b_T_ee_des)
        delta_tr = tf.transformations.translation_from_matrix(change_M)
        delta_ro_angle, delta_ro_axis = self.rotation_from_matrix(change_M)
        p = 1
        xdot = delta_tr * p
        vel_rota = delta_ro_angle * p
        vel_rotate = delta_ro_axis * vel_rota
        vel = [xdot[0],xdot[1],xdot[2],vel_rotate[0],vel_rotate[1],vel_rotate[2]]
        J = self.get_jacobian(b_T_ee_cur, joint_transforms)
        JP = numpy.linalg.pinv(J,1.0e-2)
        qdot = numpy.dot(JP, vel)
        if so == True :
            qdot_sec = numpy.zeros(self.num_joints)
            qdot_sec[0] = q0
            I = numpy.identity(self.num_joints)
            JP = numpy.linalg.pinv(J)
            qdot_null = numpy.dot((I - numpy.dot(JP,J)),qdot_sec)
            qdot = qdot + qdot_null

        joint_vel = JointState()
        joint_vel.name = self.joint_names
        joint_vel.velocity = qdot
        joint_vel.header.stamp = rospy.Time.now()
        self.velocity_pub.publish(joint_vel)

        #--------------------------------------------------------------------------
        self.mutex.release()

    '''This is a function which will assemble the jacobian of the robot using the
       current joint transforms and the transform from the base to the end
       effector (b_T_ee). Both the cartesian control callback and the
       inverse kinematics callback will make use of this function.
       Usage: J = self.get_jacobian(b_T_ee, joint_transforms)'''
    def Skew(self,x):
        return numpy.array([[0,-x[2],x[1]],[x[2],0,-x[0]],[-x[1],x[0],0]])

    def get_jacobian(self, b_T_ee, joint_transforms):
        J = numpy.zeros((6,self.num_joints))
        #--------------------------------------------------------------------------
        #FILL IN YOUR PART OF THE CODE FOR ASSEMBLING THE CURRENT JACOBIAN HERE
        V = []
        ee_T_b = numpy.linalg.inv(b_T_ee)
        for j in range(len(joint_transforms)):
            b_T_j = joint_transforms[j]
            ee_T_j = numpy.dot(ee_T_b, b_T_j)
            j_T_ee = numpy.linalg.inv(ee_T_j)
            R = ee_T_j[:3,:3]
            Rt = self.Skew(j_T_ee[:,3])
            RSt = - numpy.dot(R,Rt)
            Vj = numpy.array([[R[0][0],R[0][1],R[0][2],RSt[0][0],RSt[0][1],RSt[0][2]]
                ,[R[1][0],R[1][1],R[1][2],RSt[1][0],RSt[1][1],RSt[1][2]]
                ,[R[2][0],R[2][1],R[2][2],RSt[2][0],RSt[2][1],RSt[2][2]]
                ,[0,0,0,R[0][0],R[0][1],R[0][2]]
                ,[0,0,0,R[1][0],R[1][1],R[1][2]]
                ,[0,0,0,R[2][0],R[2][1],R[2][2]]])
            V.append(Vj)
        '''R = ee_T_b[:3,:3]
        Rt = self.Skew(b_T_ee[:,3])
        RSt = - numpy.dot(R,Rt)
        Vj = numpy.array([[R[0][0],R[0][1],R[0][2],RSt[0][0],RSt[0][1],RSt[0][2]]
            ,[R[1][0],R[1][1],R[1][2],RSt[1][0],RSt[1][1],RSt[1][2]]
            ,[R[2][0],R[2][1],R[2][2],RSt[2][0],RSt[2][1],RSt[2][2]]
            ,[0,0,0,R[0][0],R[0][1],R[0][2]]
            ,[0,0,0,R[1][0],R[1][1],R[1][2]]
            ,[0,0,0,R[2][0],R[2][1],R[2][2]]])
        V.append(Vj)'''

        for i in range(self.num_joints):
            J[:,i] = numpy.dot(V[i][:,3:],self.joint_axes[i])
        #--------------------------------------------------------------------------
        return J

    '''This is the callback which will be executed when the inverse kinematics
       recieve a new command. The command will contain information about desired
       end effector pose relative to the root of your robot. At the end of this
       callback, you should publish to the /joint_command topic. This should not
       search for a solution indefinitely - there should be a time limit. When
       searching for two matrices which are the same, we expect numerical
       precision of 10e-3.'''
    def get_ik_command(self, command):
        self.mutex.acquire()
        #--------------------------------------------------------------------------
        #FILL IN YOUR PART OF THE CODE FOR INVERSE KINEMATICS HERE
        tx = command.translation.x
        ty = command.translation.y
        tz = command.translation.z
        rx = command.rotation.x
        ry = command.rotation.y
        rz = command.rotation.z
        rw = command.rotation.w

        Tl_des = tf.transformations.translation_matrix((tx,ty,tz))
        Ro_des = tf.transformations.quaternion_matrix([rx,ry,rz,rw])
        M_des = numpy.dot(Tl_des, Ro_des)
        tr_des = tf.transformations.translation_from_matrix(M_des)
        ro_angle_des, ro_axis_des = self.rotation_from_matrix(M_des)
        ro_des = ro_angle_des * ro_axis_des
        x_des = numpy.array([tr_des[0],tr_des[1],tr_des[2],ro_des[0],ro_des[1],ro_des[2]])


        for ttt in range(3):
            q_cur = numpy.random.rand(len(self.q_current)) * 2 * math.pi
            t1 = time.time()
            while True:
                joint_transforms, b_T_ee = self.forward_kinematics(q_cur)
                ee_T_b = numpy.linalg.inv(b_T_ee)
                delta_x = numpy.dot(ee_T_b, M_des)
                tr_delta = tf.transformations.translation_from_matrix(delta_x)
                ro_angle_delta, ro_axis_delta = self.rotation_from_matrix(delta_x)
                ro_delta = ro_angle_delta * ro_axis_delta
                x_delta = numpy.array([tr_delta[0],tr_delta[1],tr_delta[2],ro_delta[0],ro_delta[1],ro_delta[2]])
                J = self.get_jacobian(b_T_ee, joint_transforms)
                JP = numpy.linalg.pinv(J)
                delta_q = numpy.dot(JP,x_delta)
                q_cur = q_cur + delta_q
                t2 = time.time()
                if numpy.linalg.norm(x_delta)< 1.0e-6: 
                    joint_posi = JointState()
                    joint_posi.name = self.joint_names
                    joint_posi.position = q_cur
                    joint_posi.header.stamp = rospy.Time.now()
                    self.joint_command_pub.publish(joint_posi)
                    ttt = ttt + 3
                    break
                if abs(t1 - t2) > 10 :
                    print 'failed to reach goal'
                    break


        #--------------------------------------------------------------------------
        self.mutex.release()

    '''This function will return the angle-axis representation of the rotation
       contained in the input matrix. Use like this: 
       angle, axis = rotation_from_matrix(R)'''
    def rotation_from_matrix(self, matrix):
        R = numpy.array(matrix, dtype=numpy.float64, copy=False)
        R33 = R[:3, :3]
        # axis: unit eigenvector of R33 corresponding to eigenvalue of 1
        l, W = numpy.linalg.eig(R33.T)
        i = numpy.where(abs(numpy.real(l) - 1.0) < 1e-8)[0]
        if not len(i):
            raise ValueError("no unit eigenvector corresponding to eigenvalue 1")
        axis = numpy.real(W[:, i[-1]]).squeeze()
        # point: unit eigenvector of R33 corresponding to eigenvalue of 1
        l, Q = numpy.linalg.eig(R)
        i = numpy.where(abs(numpy.real(l) - 1.0) < 1e-8)[0]
        if not len(i):
            raise ValueError("no unit eigenvector corresponding to eigenvalue 1")
        # rotation angle depending on axis
        cosa = (numpy.trace(R33) - 1.0) / 2.0
        if abs(axis[2]) > 1e-8:
            sina = (R[1, 0] + (cosa-1.0)*axis[0]*axis[1]) / axis[2]
        elif abs(axis[1]) > 1e-8:
            sina = (R[0, 2] + (cosa-1.0)*axis[0]*axis[2]) / axis[1]
        else:
            sina = (R[2, 1] + (cosa-1.0)*axis[1]*axis[2]) / axis[0]
        angle = math.atan2(sina, cosa)
        return angle, axis

    '''This is the function which will perform forward kinematics for your 
       cartesian control and inverse kinematics functions. It takes as input
       joint values for the robot and will return an array of 4x4 transforms
       from the base to each joint of the robot, as well as the transform from
       the base to the end effector.
       Usage: joint_transforms, b_T_ee = self.forward_kinematics(joint_values)'''
    def forward_kinematics(self, joint_values):
        joint_transforms = []

        link = self.robot.get_root()
        T = tf.transformations.identity_matrix()

        while True:
            if link not in self.robot.child_map:
                break

            (joint_name, next_link) = self.robot.child_map[link][0]
            joint = self.robot.joint_map[joint_name]

            T_l = numpy.dot(tf.transformations.translation_matrix(joint.origin.xyz), tf.transformations.euler_matrix(joint.origin.rpy[0], joint.origin.rpy[1], joint.origin.rpy[2]))
            T = numpy.dot(T, T_l)

            if joint.type != "fixed":
                joint_transforms.append(T)
                q_index = self.joint_names.index(joint_name)
                T_j = tf.transformations.rotation_matrix(joint_values[q_index], numpy.asarray(joint.axis))
                T = numpy.dot(T, T_j)
            link = next_link
        return joint_transforms, T #where T = b_T_ee

    '''This is the callback which will recieve and store the current robot
       joint states.'''
    def get_joint_state(self, msg):
        self.mutex.acquire()
        self.q_current = []
        for name in self.joint_names:
            self.q_current.append(msg.position[msg.name.index(name)])
        self.mutex.release()


if __name__ == '__main__':
    rospy.init_node('cartesian_control_and_IK', anonymous=True)
    CCIK()
    rospy.spin()
