#!/usr/bin/env python

import rospy
import numpy
import math

import tf
import tf2_ros
import geometry_msgs.msg

def transform_operation(A, B, Transform_Matrix):
    #Create Transform message from A to B
    T = geometry_msgs.msg.TransformStamped()
    T.header.stamp = rospy.Time.now()
    T.header.frame_id = A
    T.child_frame_id = B

    #Get Quaternion and Translation1 from Trans_Base_to_Obj Matrix
    Q = tf.transformations.quaternion_from_matrix(Transform_Matrix)
    TL = tf.transformations.translation_from_matrix(Transform_Matrix)
    T.transform.rotation.x = Q[0]
    T.transform.rotation.y = Q[1]
    T.transform.rotation.z = Q[2]
    T.transform.rotation.w = Q[3]
    T.transform.translation.x = TL[0]
    T.transform.translation.y = TL[1]
    T.transform.translation.z = TL[2]

    #Send Message to TF
    br.sendTransform(T)



def publish_transforms():
    #Mission 1
    #Transform Matrix from Base to Object
    Trans_Base_to_TemObj = tf.transformations.euler_matrix(0.64, 0.64, 0.0, 'sxyz')
    Trans_TemObj_to_Obj = tf.transformations.translation_matrix((1.5, 0.8, 0))
    Trans_Base_to_Obj = numpy.dot(Trans_Base_to_TemObj, Trans_TemObj_to_Obj)
	
    #Transform Base to Object
    transform_operation('base_frame', 'object_frame', Trans_Base_to_Obj)


    #Mission 2
    #Transform Matrix from Base to Robot
    Trans_Base_to_TemRob = tf.transformations.rotation_matrix(1.5, (0, 1, 0), None)
    Trans_TemRob_to_Rob = tf.transformations.translation_matrix((0, 0, -2))
    Trans_Base_to_Rob = numpy.dot(Trans_Base_to_TemRob, Trans_TemRob_to_Rob)
	
    #Transform Base to Robot
    transform_operation('base_frame', 'robot_frame', Trans_Base_to_Rob)

    #Mission 3
    #Transform Matrix from Base to Temporary Camera
    Trans_Rob_to_TemCam = tf.transformations.translation_matrix((0.3, 0, 0.3))
    Trans_Base_to_TemCam = numpy.dot(Trans_Base_to_Rob, Trans_Rob_to_TemCam)

    #Calculate Transform Matrix from Temporary Camera to Object
    Trans_TemCam_to_Base = numpy.linalg.inv(Trans_Base_to_TemCam)
    Trans_TemCam_to_Obj = numpy.dot(Trans_TemCam_to_Base, Trans_Base_to_Obj)
    Point_O_of_Obj = (0, 0, 0, 1)
    Point_Obj_of_TemCam = numpy.dot(Trans_TemCam_to_Obj, Point_O_of_Obj)

    #Calculate Rotation Axes and Angle
    TemCam_X = (1, 0, 0)
    #print Point_Obj_of_TemCam
    Rotation_Axe = numpy.cross(TemCam_X, Point_Obj_of_TemCam[:3])
    Norm_Rotation_Axe = numpy.linalg.norm(Rotation_Axe)
    Norm_TemCam_X = numpy.linalg.norm(TemCam_X)
    Norm_Point_Obj_of_TemCam = numpy.linalg.norm(Point_Obj_of_TemCam[:3])
    Rotation_Angle = math.asin(Norm_Rotation_Axe / (Norm_TemCam_X * Norm_Point_Obj_of_TemCam))

    #Transform Matrix from Base to Camera
    Trans_TemCam_to_Cam = tf.transformations.rotation_matrix(Rotation_Angle, Rotation_Axe, None)
    Trans_Base_to_Cam = numpy.dot(Trans_Base_to_TemCam, Trans_TemCam_to_Cam)

    #Transform Matrix from Robot to Camera
    Trans_Rob_to_Base = numpy.linalg.inv(Trans_Base_to_Rob)
    Trans_Rob_to_Cam = numpy.dot(Trans_Rob_to_Base, Trans_Base_to_Cam)
	
    #Transform Robot to Camera
    transform_operation('robot_frame', 'camera_frame', Trans_Rob_to_Cam)




if __name__ == '__main__':
    rospy.init_node('solution')

    br = tf2_ros.TransformBroadcaster()
    rospy.sleep(0.1)

    while not rospy.is_shutdown():
        publish_transforms()
        rospy.sleep(0.1)
