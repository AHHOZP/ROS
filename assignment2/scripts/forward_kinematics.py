#!/usr/bin/env python

import numpy
import geometry_msgs.msg
import rospy
from sensor_msgs.msg import JointState
import tf
import tf2_ros
import tf.msg
from urdf_parser_py.urdf import URDF
  
    
#Our main class for computing Forward Kinematics
class ForwardKinematics(object):

    #Initialization
    def __init__(self):
        """Announces that it will publish forward kinematics results, in the form of tfMessages.
        "tf" stands for "transform library", it's ROS's way of communicating around information
        about where things are in the world"""
        self.pub_tf = rospy.Publisher("/tf", tf.msg.tfMessage, queue_size=1)

        #Loads the robot model, which contains the robot's kinematics information
        self.robot = URDF.from_parameter_server()

        #Subscribes to information about what the current joint values are.
        rospy.Subscriber("joint_states", JointState, self.callback)
    
    """This function will transform a 4x4 transformation matrix T into a ros message 
    which can be published. In addition to the transform itself, the message
    also specifies who is related by this transform, the parent and the child.
    It is an optional function which you may use as you see fit."""
    def Mat_to_Mes_pub(self, T, child, parent):
        t = geometry_msgs.msg.TransformStamped()
        t.header.frame_id = parent
        t.header.stamp = rospy.Time.now()
        t.child_frame_id = child
        translation = tf.transformations.translation_from_matrix(T)
        rotation = tf.transformations.quaternion_from_matrix(T)
        t.transform.translation.x = translation[0]
        t.transform.translation.y = translation[1]
        t.transform.translation.z = translation[2]
        t.transform.rotation.x = rotation[0]
        t.transform.rotation.y = rotation[1]
        t.transform.rotation.z = rotation[2]
        t.transform.rotation.w = rotation[3]
        br = tf2_ros.TransformBroadcaster()
        br.sendTransform(t)

    
    def get_joint(self, parent_joint_name):
        (next_joint_name, next_link_name) = self.robot.child_map[parent_joint_name][0]
        next_joint = self.robot.joint_map[next_joint_name]
        return (next_joint_name, next_link_name, next_joint)
    
    def trans_a_to_b(self, joint, q):
	    Translation_a_to_b = tf.transformations.translation_matrix(joint.origin.xyz)
	    rpy_a_to_b = tf.transformations.euler_matrix(joint.origin.rpy[0], joint.origin.rpy[1], joint.origin.rpy[2], axes='sxyz')
	    Rotation_a_to_b = tf.transformations.rotation_matrix(q, joint.axis, None)
	    Tf_a_to_b = numpy.dot(numpy.dot(Translation_a_to_b, rpy_a_to_b), Rotation_a_to_b)
	    return Tf_a_to_b
	    
    def get_joint_link(self,root):
        joint = list()
        joint_name = list()
        link_name = list()
        jointname, linkname, jt= self.get_joint(root)
        joint_name.append(jointname)
        link_name.append(linkname)
        joint.append(jt)
        for i in range(1,8):
            jointname, linkname, jt= self.get_joint(link_name[i-1])
            joint_name.append(jointname)
            link_name.append(linkname)
            joint.append(jt)
        return joint_name, link_name, joint

    """This function is called every time the robot publishes its joint values. You must use
    the information you get to compute forward kinematics.

    The callback you write should iterate through the entire robot chain, and publish 
    the transform for each link you find.
    """
    def callback(self, joint_values):
	    #robot = URDF.from_parameter_server()
	    link_name_root = self.robot.get_root()
	    
	    # get every joint and link
	    (joint_name, link_name, joint) = self.get_joint_link(link_name_root)
	    
	    Trans_b_to = numpy.zeros(((8,4,4)))
	    Trans_1 = numpy.zeros(((8,4,4)))

	    # Matrix calculate
	    Trans_b_to[0] = tf.transformations.translation_matrix(joint[0].origin.xyz)
	    Trans_1[0] = Trans_b_to[0]
	    
	    for i in range(1,len(joint_values.position)+1):
	        Trans_1[i] = self.trans_a_to_b(joint[i], joint_values.position[i-1])
	        Trans_b_to[i] = numpy.dot(Trans_b_to[i-1], Trans_1[i])
	            
	    for i in range(len(joint_values.position)+1):
	        self.Mat_to_Mes_pub(Trans_b_to[i], link_name[i], link_name_root)
	    
	    if len(joint_values.position) == 6:
	        Translation_6_to_7 = tf.transformations.translation_matrix(joint[7].origin.xyz)
	        rpy_6_to_7 = tf.transformations.euler_matrix(joint[7].origin.rpy[0], joint[7].origin.rpy[1], joint[7].origin.rpy[2], axes='sxyz')
	        Trans_6_to_7 = numpy.dot(Translation_6_to_7, rpy_6_to_7)
	        Trans_b_to_7 = numpy.dot(Trans_b_to[6], Trans_6_to_7)
	        self.Mat_to_Mes_pub(Trans_b_to_7, link_name[7], link_name_root)



	    

if __name__ == '__main__':
    rospy.init_node('fwk', anonymous=True)
    fwk = ForwardKinematics()
    rospy.spin()

