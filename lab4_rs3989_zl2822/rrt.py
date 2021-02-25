# created by Ruoran Shi and Zhenhao Li for 4733 Lab4

from __future__ import division
import pybullet as p
import pybullet_data
import numpy as np
import time
import argparse
import math
import random


UR5_JOINT_INDICES = [0, 1, 2]


def set_joint_positions(body, joints, values):
    assert len(joints) == len(values)
    for joint, value in zip(joints, values):
        p.resetJointState(body, joint, value)


def draw_sphere_marker(position, radius, color):
   vs_id = p.createVisualShape(p.GEOM_SPHERE, radius=radius, rgbaColor=color)
   marker_id = p.createMultiBody(basePosition=position, baseCollisionShapeIndex=-1, baseVisualShapeIndex=vs_id)
   return marker_id


def remove_marker(marker_id):
   p.removeBody(marker_id)


def get_args():
    parser = argparse.ArgumentParser()
    parser.add_argument('--birrt', action='store_true', default=False)
    parser.add_argument('--smoothing', action='store_true', default=False)
    args = parser.parse_args()
    return args

# helper function
def search_nearest(tree, conf):
    min_distance = float('inf')
    for i in range(len(tree)):
        temp_distance = np.linalg.norm(conf - tree[i][1])
        if(min_distance > temp_distance):
            min_distance = temp_distance
            nearest_node = tree[i][1]
    return nearest_node

def gen_new_node_rand(tree, step_size, color):
    collision_flag = 1
    while(collision_flag == 1):
        # generate rand
        rand_conf = np.random.rand(3)
        rand_conf[0]= rand_conf[0]*4*math.pi - 2*math.pi
        rand_conf[1]= rand_conf[1]*4*math.pi - 2*math.pi
        rand_conf[2]= rand_conf[2]*2*math.pi - math.pi
        # collision_flag = collision_fn(rand_conf)
        # if(collision_flag == 1):
        #     continue    
        old_node = search_nearest(tree, rand_conf)
        direction = np.array(rand_conf) - np.array(old_node)
        new_node = old_node + direction / np.linalg.norm(direction) * step_size
        collision_flag = collision_fn(new_node)

    # draw
    end = p.getLinkState(ur5, 3)[0]
    set_joint_positions(ur5, UR5_JOINT_INDICES, old_node)
    start = p.getLinkState(ur5, 3)[0]
    p.addUserDebugLine(start, end, color)

    # branch = [parent, child]
    OKstep = [old_node, new_node]
    return OKstep

# generate a new node toward target
def gen_new_node(tree, target, step_size, color):
    old_node = search_nearest(tree, target)
    direction = np.array(target) - np.array(old_node)
    new_node = old_node + direction / np.linalg.norm(direction) * step_size
    if(collision_fn(new_node) == 1):
        return None

    # draw
    end = p.getLinkState(ur5, 3)[0]
    set_joint_positions(ur5, UR5_JOINT_INDICES, old_node)
    start = p.getLinkState(ur5, 3)[0]
    p.addUserDebugLine(start, end, color)

    # branch = [parent, child]
    OKstep = [old_node, new_node]
    return OKstep

# if newly generated node of tree is in step_size of goal, the tree can reach goal
def can_reach_goal(last_node, step_size):
    if(np.linalg.norm(np.array(goal_conf) - last_node) < step_size):
        # draw
        color = [0, 1, 0] # green
        set_joint_positions(ur5, UR5_JOINT_INDICES, last_node)
        start = p.getLinkState(ur5, 3)[0]
        p.addUserDebugLine(start, goal_position, color)
        return True
    else:
        return False

# if newly generated node of one tree is in step_size of a node in the other tree, two trees can connect
def try_connect(tree_list, tree_index, step_size):
    # tree_list[tree_index] just added a new node to its last position
    new_node = tree_list[tree_index][-1][1]
    nearest_node = search_nearest(tree_list[not tree_index], new_node)
    if(np.linalg.norm(new_node - nearest_node) < step_size):
        # draw
        if(tree_index == 0):
            color = [0, 1, 0] # green
        else:
            color = [0, 0, 1] # blue
        set_joint_positions(ur5, UR5_JOINT_INDICES, new_node)
        start = p.getLinkState(ur5, 3)[0]
        set_joint_positions(ur5, UR5_JOINT_INDICES, nearest_node)
        end = p.getLinkState(ur5, 3)[0]
        p.addUserDebugLine(start, end, color)
        return True, new_node, nearest_node
    else:
        return False, None, None

# used to recover path from last node to root
def find_parent(child, tree):
    for i in range(len(tree)):
        if np.array_equal(child, tree[i][1]):
            return tree[i][0]

def recover_bipath(node_in_start_tree, node_in_goal_tree, start_tree, goal_tree):
    path_conf = []
    path_conf.insert(0, node_in_start_tree)
    while(not np.array_equal(path_conf[0], np.array(start_conf))):
        path_conf.insert(0, find_parent(path_conf[0], start_tree))
    path_conf.append(node_in_goal_tree)
    while(not np.array_equal(path_conf[-1], np.array(goal_conf))):
        path_conf.append(find_parent(path_conf[-1], goal_tree))
    return path_conf

def smooth_path(path_conf, step_size):
    if(len(path_conf)<=2):
        return path_conf
    node1_index = random.randint(0, len(path_conf)-3)
    node2_index = random.randint(node1_index+2, len(path_conf)-1)
    direction = np.array(path_conf[node2_index]) - np.array(path_conf[node1_index])
    step = direction / np.linalg.norm(direction) * step_size
    step_num = int(math.floor(np.linalg.norm(direction) / step_size))
    
    new_path = []
    new_node = path_conf[node1_index].copy() + step 
    for i in range(step_num):
        # collide, cannot smooth
        if(collision_fn(new_node) == 1):
            return path_conf
        # add safe node to new path
        new_path.append(new_node)
        new_node = new_node + step

    # last node of new_path is the same as node2, delete it
    if(len(new_path)>0 and np.array_equal(new_path[-1], path_conf[node2_index])):
        del new_path[-1]

    # can smooth
    del path_conf[(node1_index+1):(node2_index)]
    path_conf[node1_index+1:node1_index+1] = new_path
    return path_conf

def draw_path(path_conf, color):
    set_joint_positions(ur5, UR5_JOINT_INDICES, path_conf[0])
    for i in range(1, len(path_conf)):
        start = p.getLinkState(ur5, 3)[0]
        set_joint_positions(ur5, UR5_JOINT_INDICES, path_conf[i])
        end = p.getLinkState(ur5, 3)[0]
        p.addUserDebugLine(start, end, color, 5)

# debug function
def check_path(path_conf):
    for i in range(len(path_conf)-1):
        if(np.array_equal(path_conf[i],path_conf[i+1])):
            print("wrong path: identical node",i)
        if(np.linalg.norm(path_conf[i]-path_conf[i+1])>0.100000001):
            print("wrong path: exceed step size",i,np.linalg.norm(path_conf[i]-path_conf[i+1]))

def print_tree(tree):
    for i in range(len(tree)):
        print(tree[i])

# rrt GO!
def rrt():
    ###############################################
    # TODO your code to implement the rrt algorithm
    ###############################################
    step_size = 0.1
    bias = 0.5
    # tree: list of [parent, child]
    tree = []
    start_node = [[None, None, None], np.array(start_conf)]
    tree.append(start_node)

    while(not can_reach_goal(tree[-1][1], step_size)):
        if(np.random.rand()>bias):
            new_step = gen_new_node(tree, goal_conf, step_size, [0, 1, 0])
            if(new_step != None):
                tree.append(new_step)
            else:
                tree.append(gen_new_node_rand(tree, step_size, [0, 1, 0]))
        else:
            tree.append(gen_new_node_rand(tree, step_size, [0, 1, 0]))

    # recover path
    path_conf = []
    path_conf.insert(0, np.array(goal_conf))
    path_conf.insert(0, tree[-1][1])
    while(not np.array_equal(path_conf[0], np.array(start_conf))):
        path_conf.insert(0, find_parent(path_conf[0], tree))

    return path_conf

def birrt():
    #################################################
    # TODO your code to implement the birrt algorithm
    #################################################

    step_size = 0.1
    # tree: list of [parent, child]
    start_tree = [] 
    start_node = [[None, None, None], np.array(start_conf)]
    start_tree.append(start_node)
    goal_tree = []
    goal_node = [[None, None, None], np.array(goal_conf)]
    goal_tree.append(goal_node)
    tree_list = [start_tree, goal_tree]
    tree_color = [[0, 1, 0], [0, 0, 1]] # tree_color[0] = green, tree_color[1] = blue
    tree_index = 0

    while(True):
        # Expand one tree randomly
        new_step = gen_new_node_rand(tree_list[tree_index], step_size, tree_color[tree_index])
        tree_list[tree_index].append(new_step)
        connected, connect_node1, connect_node2 = try_connect(tree_list, tree_index, step_size)
        if(connected):
            break
        
        # Expand the other tree towards q_new
        q_new = new_step[1]
        tree_index = not tree_index # switch to the other tree
        new_step = gen_new_node(tree_list[tree_index], q_new, step_size, tree_color[tree_index])
        if(new_step != None):
            tree_list[tree_index].append(new_step)
            connected, connect_node1, connect_node2 = try_connect(tree_list, tree_index, step_size)
            if(connected):
                break

    # recover path
    path_conf = []
    if(tree_index == 0):
        # connect_node1 in start_tree, connect_node2 in goal_tree
        return recover_bipath(connect_node1, connect_node2, start_tree, goal_tree)
    else:
        return recover_bipath(connect_node2, connect_node1, start_tree, goal_tree)

def birrt_smoothing():
    ################################################################
    # TODO your code to implement the birrt algorithm with smoothing
    ################################################################
    step_size = 0.1
    path_conf = birrt()
    for i in range(100):
        path_conf = smooth_path(path_conf, step_size)
    return path_conf



if __name__ == "__main__":
    args = get_args()

    # set up simulator
    physicsClient = p.connect(p.GUI)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setPhysicsEngineParameter(enableFileCaching=0)
    p.setGravity(0, 0, -9.8)
    p.configureDebugVisualizer(p.COV_ENABLE_GUI, False)
    p.configureDebugVisualizer(p.COV_ENABLE_SHADOWS, True)
    p.resetDebugVisualizerCamera(cameraDistance=1.400, cameraYaw=58.000, cameraPitch=-42.200, cameraTargetPosition=(0.0, 0.0, 0.0))

    # load objects
    plane = p.loadURDF("plane.urdf")
    ur5 = p.loadURDF('assets/ur5/ur5.urdf', basePosition=[0, 0, 0.02], useFixedBase=True)
    obstacle1 = p.loadURDF('assets/block.urdf',
                           basePosition=[1/4, 0, 1/2],
                           useFixedBase=True)
    obstacle2 = p.loadURDF('assets/block.urdf',
                           basePosition=[2/4, 0, 2/3],
                           useFixedBase=True)
    obstacles = [plane, obstacle1, obstacle2]

    # start and goal
    start_conf = (-0.813358794499552, -0.37120422397572495, -0.754454729356351)
    start_position = (0.3998897969722748, -0.3993956744670868, 0.6173484325408936)
    goal_conf = (0.7527214782907734, -0.6521867735052328, -0.4949270744967443)
    goal_position = (0.35317009687423706, 0.35294029116630554, 0.7246701717376709)
    goal_marker = draw_sphere_marker(position=goal_position, radius=0.02, color=[1, 0, 0, 1])
    set_joint_positions(ur5, UR5_JOINT_INDICES, start_conf)

    # place holder to save the solution path
    path_conf = None

    # get the collision checking function
    from collision_utils import get_collision_fn
    collision_fn = get_collision_fn(ur5, UR5_JOINT_INDICES, obstacles=obstacles,
                                       attachments=[], self_collisions=True,
                                       disabled_collisions=set())

    if args.birrt:
        if args.smoothing:
            # using birrt with smoothing
            path_conf = birrt_smoothing()
        else:
            # using birrt without smoothing
            path_conf = birrt()
    else:
        # using rrt
        path_conf = rrt()

    if path_conf is None:
        # pause here
        raw_input("no collision-free path is found within the time budget, finish?")
    else:
        ###############################################
        # TODO your code to highlight the solution path
        ###############################################
        check_path(path_conf)
        draw_path(path_conf, [1, 0, 0])

        # execute the path
        while True:
            for q in path_conf:
                set_joint_positions(ur5, UR5_JOINT_INDICES, q)
                time.sleep(0.5)

            