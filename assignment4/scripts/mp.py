#!/usr/bin/env python

import numpy
import random
import sys
import math

import geometry_msgs.msg
import moveit_msgs.msg
import moveit_msgs.srv
import rospy
import tf
import moveit_commander
from urdf_parser_py.urdf import URDF
from std_msgs.msg import String
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Transform
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

def convert_to_message(T):
    t = geometry_msgs.msg.Pose()
    position = tf.transformations.translation_from_matrix(T)
    orientation = tf.transformations.quaternion_from_matrix(T)
    t.position.x = position[0]
    t.position.y = position[1]
    t.position.z = position[2]
    t.orientation.x = orientation[0]
    t.orientation.y = orientation[1]
    t.orientation.z = orientation[2]
    t.orientation.w = orientation[3]        
    return t

class MoveArm(object):

    def __init__(self):

        #Loads the robot model, which contains the robot's kinematics information
        self.num_joints = 0
        self.joint_names = []
        self.joint_axes = []
        self.robot = URDF.from_parameter_server()
        self.base = self.robot.get_root()
        self.get_joint_info()

        # Wait for moveit IK service
        rospy.wait_for_service("compute_ik")
        self.ik_service = rospy.ServiceProxy('compute_ik',  moveit_msgs.srv.GetPositionIK)
        print "IK service ready"

        # Wait for validity check service
        rospy.wait_for_service("check_state_validity")
        self.state_valid_service = rospy.ServiceProxy('check_state_validity',  
                                                      moveit_msgs.srv.GetStateValidity)
        print "State validity service ready"

        # MoveIt parameter
        robot_moveit = moveit_commander.RobotCommander()
        self.group_name = robot_moveit.get_group_names()[0]

        #Subscribe to topics
        rospy.Subscriber('/joint_states', JointState, self.get_joint_state)
        rospy.Subscriber('/motion_planning_goal', Transform, self.motion_planning)
        self.current_obstacle = "None"
        rospy.Subscriber('/obstacle', String, self.get_obstacle)

    #Set up publisher
        self.pub = rospy.Publisher('/joint_trajectory', JointTrajectory, queue_size=10)

    '''This callback provides you with the current joint positions of the robot 
     in member variable q_current.'''
    def get_joint_state(self, msg):
        self.q_current = []
        for name in self.joint_names:
            self.q_current.append(msg.position[msg.name.index(name)])

    '''This callback provides you with the name of the current obstacle which
    exists in the RVIZ environment. Options are "None", "Simple", "Hard",
    or "Super". '''
    def get_obstacle(self, msg):
        self.current_obstacle = msg.data








    # helper function
    def search_nearest(self, tree, conf):
        min_distance = float('inf')
        for i in range(len(tree)):
            temp_distance = numpy.linalg.norm(conf - tree[i][1])
            if(min_distance > temp_distance):
                min_distance = temp_distance
                nearest_node = tree[i][1]
        return nearest_node

    def gen_new_node_rand(self, tree, step_size):
        collision_flag = 1
        while(collision_flag == 1):
            # generate rand
            rand_conf = numpy.random.rand(self.num_joints)
            if self.num_joints ==7 :
                for i in range(self.num_joints):
                    rand_conf[i]= rand_conf[i]*2*math.pi - math.pi
            elif self.num_joints == 6:
                for i in range(self.num_joints):
                        rand_conf[i]= rand_conf[i]*4*math.pi - 2*math.pi

            old_node = self.search_nearest(tree, rand_conf)
            direction = numpy.array(rand_conf) - numpy.array(old_node)
            new_node = old_node + direction / numpy.linalg.norm(direction) * step_size
            collision_flag = not self.is_state_valid(new_node)

        # branch = [parent, child]
        OKstep = [old_node, new_node]
        return OKstep

    # generate a new node toward target
    def gen_new_node(self, tree, target, step_size):
        old_node = self.search_nearest(tree, target)
        direction = numpy.array(target) - numpy.array(old_node)
        new_node = old_node + direction / numpy.linalg.norm(direction) * step_size
        if(not self.is_state_valid(new_node) == 1):
            return None

        # branch = [parent, child]
        OKstep = [old_node, new_node]
        return OKstep

    # if newly generated node of tree is in step_size of goal, the tree can reach goal
    def can_reach_goal(self, last_node, step_size):
        return numpy.linalg.norm(numpy.array(goal_conf) - last_node) < step_size

    # if newly generated node of one tree is in step_size of a node in the other tree, two trees can connect
    def try_connect(self, tree_list, tree_index, step_size):
        # tree_list[tree_index] just added a new node to its last position
        new_node = tree_list[tree_index][-1][1]
        nearest_node = self.search_nearest(tree_list[not tree_index], new_node)
        if(numpy.linalg.norm(new_node - nearest_node) < step_size):
            return True, new_node, nearest_node
        else:
            return False, None, None

    # used to recover path from last node to root
    def find_parent(self, child, tree):
        for i in range(len(tree)):
            if numpy.array_equal(child, tree[i][1]):
                return tree[i][0]

    def recover_bipath(self, node_in_start_tree, node_in_goal_tree, start_tree, goal_tree):
        path_conf = []
        path_conf.insert(0, node_in_start_tree)
        while(not numpy.array_equal(path_conf[0], self.start_conf)):
            path_conf.insert(0, self.find_parent(path_conf[0], start_tree))
        path_conf.append(node_in_goal_tree)
        while(not numpy.array_equal(path_conf[-1], self.goal_conf)):
            path_conf.append(self.find_parent(path_conf[-1], goal_tree))
        return path_conf

    def smooth_path(self, path_conf, step_size):
        if(len(path_conf)<=2):
            return path_conf
        node1_index = random.randint(0, len(path_conf)-3)
        node2_index = random.randint(node1_index+2, len(path_conf)-1)
        direction = numpy.array(path_conf[node2_index]) - numpy.array(path_conf[node1_index])
        step = direction / numpy.linalg.norm(direction) * step_size
        step_num = int(math.floor(numpy.linalg.norm(direction) / step_size))
        
        new_path = []
        new_node = path_conf[node1_index].copy() + step 
        for i in range(step_num):
            # collide, cannot smooth
            if(not self.is_state_valid(new_node) == 1):
                return path_conf
            # add safe node to new path
            new_path.append(new_node)
            new_node = new_node + step

        # last node of new_path is the same as node2, delete it
        if(len(new_path)>0 and numpy.array_equal(new_path[-1], path_conf[node2_index])):
            del new_path[-1]

        # can smooth
        del path_conf[(node1_index+1):(node2_index)]
        path_conf[node1_index+1:node1_index+1] = new_path
        return path_conf

    # debug function
    def check_path(path_conf):
        for i in range(len(path_conf)-1):
            if(numpy.array_equal(path_conf[i],path_conf[i+1])):
                print("wrong path: identical node",i)
            if(numpy.linalg.norm(path_conf[i]-path_conf[i+1])>0.100000001):
                print("wrong path: exceed step size",i,numpy.linalg.norm(path_conf[i]-path_conf[i+1]))


    def birrt(self):
        if self.num_joints ==7:
            step_size = 0.1
        elif self.num_joints == 6:
            step_size = 0.08
        # tree: list of [parent, child]
        start_tree = [] 
        start_node = [[None, None, None], self.start_conf]
        start_tree.append(start_node)
        goal_tree = []
        goal_node = [[None, None, None], self.goal_conf]
        goal_tree.append(goal_node)
        tree_list = [start_tree, goal_tree]
        tree_index = 0

        while(True):
            # Expand one tree randomly
            new_step = self.gen_new_node_rand(tree_list[tree_index], step_size)
            tree_list[tree_index].append(new_step)
            connected, connect_node1, connect_node2 = self.try_connect(tree_list, tree_index, step_size)
            if(connected):
                break
            
            # Expand the other tree towards q_new
            q_new = new_step[1]
            tree_index = not tree_index # switch to the other tree
            new_step = self.gen_new_node(tree_list[tree_index], q_new, step_size)
            if(new_step != None):
                tree_list[tree_index].append(new_step)
                connected, connect_node1, connect_node2 = self.try_connect(tree_list, tree_index, step_size)
                if(connected):
                    break

        # recover path
        path_conf = []
        if(tree_index == 0):
            # connect_node1 in start_tree, connect_node2 in goal_tree
            return self.recover_bipath(connect_node1, connect_node2, start_tree, goal_tree)
        else:
            return self.recover_bipath(connect_node2, connect_node1, start_tree, goal_tree)

    def birrt_smoothing(self):
        step_size = 0.1
        path_conf = self.birrt()
        for i in range(100):
            path_conf = self.smooth_path(path_conf, step_size)
        return path_conf


    '''This is the callback which will implement your RRT motion planning.
    You are welcome to add other functions to this class (i.e. an
    "is_segment_valid" function will likely come in handy multiple times 
    in the motion planning process and it will be easiest to make this a 
    seperate function and then call it from motion planning). You may also
    create trajectory shortcut and trajectory sample functions if you wish, 
    which will also be called from the motion planning function.'''    
    def motion_planning(self, ee_goal):
        print "Starting motion planning"
    ########INSERT YOUR RRT MOTION PLANNING HERE##########
        tx = ee_goal.translation.x
        ty = ee_goal.translation.y
        tz = ee_goal.translation.z
        rx = ee_goal.rotation.x
        ry = ee_goal.rotation.y
        rz = ee_goal.rotation.z
        rw = ee_goal.rotation.w
        Tl_des = tf.transformations.translation_matrix((tx,ty,tz))
        Ro_des = tf.transformations.quaternion_matrix([rx,ry,rz,rw])
        T_goal = numpy.dot(Tl_des, Ro_des)

        self.start_conf = numpy.array(self.q_current)
        self.goal_conf = numpy.array(self.IK(T_goal))

        if len(self.goal_conf) == 0:
            print 'cannot reach goal point'
        else:

            path_conf = self.birrt_smoothing()

            JointTra = JointTrajectory()
            JointTra.joint_names = self.joint_names
            for i in range(len(path_conf)):
                jtp = JointTrajectoryPoint()
                jtp.positions = path_conf[i]
                JointTra.points.append(jtp)
            self.pub.publish(JointTra)
        ######################################################

    """ This function will perform IK for a given transform T of the end-effector.
    It returns a list q[] of values, which are the result positions for the 
    joints of the robot arm, ordered from proximal to distal. If no IK solution 
    is found, it returns an empy list.
    """
    def IK(self, T_goal):
        req = moveit_msgs.srv.GetPositionIKRequest()
        req.ik_request.group_name = self.group_name
        req.ik_request.robot_state = moveit_msgs.msg.RobotState()
        req.ik_request.robot_state.joint_state.name = self.joint_names
        req.ik_request.robot_state.joint_state.position = numpy.zeros(self.num_joints)
        req.ik_request.robot_state.joint_state.velocity = numpy.zeros(self.num_joints)
        req.ik_request.robot_state.joint_state.effort = numpy.zeros(self.num_joints)
        req.ik_request.robot_state.joint_state.header.stamp = rospy.get_rostime()
        req.ik_request.avoid_collisions = True
        req.ik_request.pose_stamped = geometry_msgs.msg.PoseStamped()
        req.ik_request.pose_stamped.header.frame_id = self.base
        req.ik_request.pose_stamped.header.stamp = rospy.get_rostime()
        req.ik_request.pose_stamped.pose = convert_to_message(T_goal)
        req.ik_request.timeout = rospy.Duration(3.0)
        res = self.ik_service(req)
        q = []
        if res.error_code.val == res.error_code.SUCCESS:
            q = res.solution.joint_state.position
        return q

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


    """ This function checks if a set of joint angles q[] creates a valid state,
    or one that is free of collisions. The values in q[] are assumed to be values
    for the joints of the KUKA arm, ordered from proximal to distal. 
    """
    def is_state_valid(self, q):
        req = moveit_msgs.srv.GetStateValidityRequest()
        req.group_name = self.group_name
        req.robot_state = moveit_msgs.msg.RobotState()
        req.robot_state.joint_state.name = self.joint_names
        req.robot_state.joint_state.position = q
        req.robot_state.joint_state.velocity = numpy.zeros(self.num_joints)
        req.robot_state.joint_state.effort = numpy.zeros(self.num_joints)
        req.robot_state.joint_state.header.stamp = rospy.get_rostime()
        res = self.state_valid_service(req)
        return res.valid


'''This is a class which you can use to keep track of your tree branches.
It is easiest to do this by appending instances of this class to a list 
(your 'tree'). The class has a parent field and a joint position field (q). 
You can initialize a new branch like this:
RRTBranch(parent, q)
Feel free to keep track of your branches in whatever way you want - this
is just one of many options available to you.'''
class RRTBranch(object):
    def __init__(self, parent, q):
        self.parent = parent
        self.q = q


if __name__ == '__main__':
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('move_arm', anonymous=True)
    ma = MoveArm()
    rospy.spin()

