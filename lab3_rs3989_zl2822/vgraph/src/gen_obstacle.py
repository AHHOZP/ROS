#!/usr/bin/env python

import rospy
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Quaternion, Pose, Point, Vector3, Twist
from std_msgs.msg import Header, ColorRGBA
import tf
from rbx1_nav.transform_utils import quat_to_angle, normalize_angle

import numpy as np
from scipy.spatial import ConvexHull

# initialize ros node
rospy.init_node('path_finding')
pub_obstacle = rospy.Publisher('vgraph_markers', Marker, queue_size=5)
pub_route = rospy.Publisher('vgraph_markers', Marker, queue_size=5)
pub_bestroute = rospy.Publisher('vgraph_markers', Marker, queue_size=5)
cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=5)
tf_listener = tf.TransformListener()
odom_frame = '/odom'

try:
    tf_listener.waitForTransform(odom_frame, '/base_footprint', rospy.Time(), rospy.Duration(1.0))
    base_frame = '/base_footprint'
except (tf.Exception, tf.ConnectivityException, tf.LookupException):
    try:
        tf_listener.waitForTransform(odom_frame, '/base_link', rospy.Time(), rospy.Duration(1.0))
        base_frame = '/base_link'
    except (tf.Exception, tf.ConnectivityException, tf.LookupException):
        rospy.loginfo("Cannot find transform between /odom and /base_link or /base_footprint")
        rospy.signal_shutdown("tf Exception")
        
# get current transform between odom and base frames
def get_odom():
    try:
        (trans, rot)  = tf_listener.lookupTransform(odom_frame, base_frame, rospy.Time(0))
    except (tf.Exception, tf.ConnectivityException, tf.LookupException):
        rospy.loginfo("TF Exception")
        return

    return (Point(*trans), quat_to_angle(Quaternion(*rot)))

# turn direction of robot to next point
def turn(Goal):
    move_r = Twist()
    (cur_position, cur_rotation) = get_odom()
    x_x = cur_position.x - Goal[0]
    y_y = cur_position.y - Goal[1]
    k = y_y / x_x
    direction = - abs(cur_rotation - k) / (cur_rotation - k)
    while abs(cur_rotation - k) > 0.5:
        move_r.angular.z = direction * 0.5
        cmd_vel.publish(move_r)
        rospy.sleep(0.5)
        (cur_position, cur_rotation) = get_odom()
    while abs(cur_rotation - k) > 0.05:
        move_r.angular.z = direction * 0.1
        cmd_vel.publish(move_r)
        rospy.sleep(0.5)
        (cur_position, cur_rotation) = get_odom()

# follow the line to next point
def follow_line(Goal):
    (position, rotation) = get_odom()
    distance = ((position.x - Goal[0])**2 + (position.y - Goal[1])**2)**0.5
    if distance > 0.5:
        move_t = Twist()
        move_t.linear.x = .5
        cmd_vel.publish(move_t)
    else :
        move_t = Twist()
        move_t.linear.x = .1
        cmd_vel.publish(move_t)
        
# check whether goal reached
def goal_reached(Goal):
    (position, rotation) = get_odom()
    x_target = Goal[0]
    y_target = Goal[1]
    if(abs(position.x-x_target)<0.1 and abs(position.y-y_target)<0.1):
        return 1
    else:
        return 0

# marker massage
def line_create(r,g,b,id,width):
    marker = Marker()
    marker.header.frame_id = "/map"
    marker.type = marker.LINE_LIST
    marker.action = marker.ADD
    marker.lifetime = rospy.Duration(1000)
    marker.id = id

    # marker scale
    marker.scale.x = width
    marker.scale.y = 0.0
    marker.scale.z = 0.0

    # marker color
    marker.color.a = 1.0
    marker.color.r = r
    marker.color.g = g
    marker.color.b = b

    # marker orientaiton
    marker.pose.orientation.x = 0.0
    marker.pose.orientation.y = 0.0
    marker.pose.orientation.z = 0.0
    marker.pose.orientation.w = 1.0

    # marker position
    marker.pose.position.x = 0.0
    marker.pose.position.y = 0.0
    marker.pose.position.z = 0.0

    return marker

# define each set of obstacle vertices as a point set
pointSets = []
pointSets.append(np.array([[1.00, .0], [1.00, .50], [1.50, .0]]))
pointSets.append(np.array([[2.25, .25], [2.75, .25], [3.25, -.25],[2.75, -.75]]))
pointSets.append(np.array([[4.50, .30], [4.50, 1.30], [5.25, 1.30],[5.25, .30]]))
pointSets.append(np.array([[4.00, -.50], [4.00, -1.00], [4.25, -.50]]))
pointSets.append(np.array([[3.00, 1.00], [3.00, 1.50], [3.50, 1.50], [3.50, 1.00]]))
num_Sets = len(pointSets)

# assume robot to be a 36*36cm square. use its center as the reference point.
ref_width = .36/2.0

# add ref vertices to each obstacle
for i in range(num_Sets):
    num_vertices = pointSets[i].shape[0]
    for j in range(num_vertices):
        x = pointSets[i][j][0]
        y = pointSets[i][j][1]
        pointSets[i] = np.r_[pointSets[i], [[x-ref_width, y-ref_width]]]
        pointSets[i] = np.r_[pointSets[i], [[x-ref_width, y+ref_width]]]
        pointSets[i] = np.r_[pointSets[i], [[x+ref_width, y-ref_width]]]
        pointSets[i] = np.r_[pointSets[i], [[x+ref_width, y+ref_width]]]
        
# grow obstacle 
hull = []
for i in range(num_Sets):
    hull.append(ConvexHull(pointSets[i]))

# vertices for each grown obstacle
vertexSets = []
for i in range(num_Sets):
    p = []
    for j in range(len(hull[i].vertices)):
        p.append(hull[i].points[(hull[i].vertices[j])])
    vertexSets.append(p)

# borderline for each grown obstacle
borderSets = []
for i in range(num_Sets):
    for j in range(-1, len(vertexSets[i])-1, 1):
        lp = []
        lp.append(vertexSets[i][j])
        lp.append(vertexSets[i][j+1])
        borderSets.append(lp)

# start and goal point
start = np.array([.0,.0])
goal = np.array([6.0,.0])

# connecte all obstacle vertices + start + goal
alllineSets = []
for i in range(num_Sets):
    for j in range(len(vertexSets[i])):
            lp = []
            lp.append(start)
            lp.append(vertexSets[i][j])
            alllineSets.append(lp)
            lp = []
            lp.append(vertexSets[i][j])
            lp.append(goal)
            alllineSets.append(lp)

for i in range(num_Sets):
    for j in range(len(vertexSets[i])):
        for k in range(i+1, num_Sets, 1):
            for l in range(len(vertexSets[k])):
                lp = []
                lp.append(vertexSets[i][j])
                lp.append(vertexSets[k][l])
                alllineSets.append(lp)

# helper function for segment intersection detecting
def ccw(A,B,C):
    return (C[1]- A[1]) *  (B[0]- A[0]) > ( B[1]- A[1]) * ( C[0]-A[0])

# return true if line segments AB and CD intersect
def intersect(A,B,C,D):
    return ccw(A,C,D) != ccw(B,C,D) and ccw(A,B,C) != ccw(A,B,D)

# all possible line
OKlineSets = []
for i in range(len(alllineSets)):
    inter = 0
    for j in range(len(borderSets)):
        a = alllineSets[i][0]
        b = alllineSets[i][1]
        c = borderSets[j][0]
        d = borderSets[j][1]
        if (a==c).all() or (a==d).all() or (b==c).all() or (b==d).all():
            pass
        else:
            if intersect(a,b,c,d):
                inter = 1
                break
    if inter == 0 :
        OKlineSets.append(alllineSets[i])

# draw border line
obstacles_grown = line_create(1,1,0,0,.03)
obstacles_grown.points = []
for i in range(len(borderSets)):
    line_point = Point()
    line_point.x = borderSets[i][0][0]
    line_point.y = borderSets[i][0][1]
    line_point.z = 0.0
    obstacles_grown.points.append(line_point)
    line_point = Point()
    line_point.x = borderSets[i][1][0]
    line_point.y = borderSets[i][1][1]
    line_point.z = 0.0
    obstacles_grown.points.append(line_point)
t = 0
while t <= 1:
    pub_obstacle.publish(obstacles_grown)
    rospy.sleep(1)
    t += 1


# draw OKline
Route = line_create(1,1,0,1,.03)
Route.points = []
for i in range(len(OKlineSets)):
    line_point = Point()
    line_point.x = OKlineSets[i][0][0]
    line_point.y = OKlineSets[i][0][1]
    line_point.z = 0.0
    Route.points.append(line_point)
    line_point = Point()
    line_point.x = OKlineSets[i][1][0]
    line_point.y = OKlineSets[i][1][1]
    line_point.z = 0.0
    Route.points.append(line_point)
t = 0
while t <= 1:
    pub_route.publish(Route)
    rospy.sleep(1)
    t += 1

# helper functions for path finding
# find index of path with minimum path length
def find_path_to_span(pathSet):
    i_min = 0
    for i in range(1, len(pathSet)):
        if(pathSet[i][0] < pathSet[i_min][0]):
            i_min = i
    return i_min

def span_path(pathSet, path_index, spanned_vertices):
    old_path = pathSet[path_index]
    # last point of old_path
    cur = old_path[-1]
    if(np.array_equal(cur, goal)):
        # shortest path found
        find_path_flag = 1
        del old_path[0]
        return find_path_flag, old_path
    
    else:
        # shortest path not found
        find_path_flag = 0
        # remove old_path from pathSet
        del pathSet[path_index]
        cur_index = find_vertex(cur, adjList)
        for adj_index in range(1, len(adjList[cur_index])):
            adj = adjList[cur_index][adj_index]
            if(not isSpanned(adj, spanned_vertices)):
                # create newpath and add it to pathSet
                new_path = list(old_path)
                # add adj point
                new_path.append(adj)
                # update path length
                new_path[0] = new_path[0] - np.linalg.norm(cur-goal) + np.linalg.norm(cur-adj) + np.linalg.norm(adj-goal)
                pathSet.append(new_path)
        spanned_vertices.append(cur)
        return find_path_flag, []
        
# find index of a vertex in adjacency list
def find_vertex(vertex, vertexSets):
    for i in range(len(vertexSets)):
        if(np.array_equal(vertex, vertexSets[i][0])):
            return i
    # vertex not found
    return -1
    
# check if a vertex is spanned
def isSpanned(vertex, spanned_vertices):
    for i in range(len(spanned_vertices)):
        if(np.array_equal(vertex, spanned_vertices[i])):
            return 1
    return 0

OKlineSets += borderSets
# sort OKlineSets
for i in range(len(OKlineSets)):
    OKlineSets[i] = sorted(OKlineSets[i], key = lambda vertex: vertex[0])
OKlineSets = sorted(OKlineSets, key = lambda line: (line[0][0], line[0][1]))

# adjacency list of undirected graph
adjList = []
for line_index in range(len(OKlineSets)):
    # process first vertex of this line
    vertex_index = find_vertex(OKlineSets[line_index][0], adjList)
    if(vertex_index == -1 ):
        adjList_item = [OKlineSets[line_index][0], OKlineSets[line_index][1]]
        adjList.append(adjList_item)
    else:
        adjList[vertex_index].append(OKlineSets[line_index][1])
    # do the same to the other vertex
    vertex_index = find_vertex(OKlineSets[line_index][1], adjList)
    if(vertex_index == -1 ):
        adjList_item = [OKlineSets[line_index][1], OKlineSets[line_index][0]]
        adjList.append(adjList_item)
    else:
        adjList[vertex_index].append(OKlineSets[line_index][0])
adjList = sorted(adjList, key = lambda vertex: (vertex[0][0], vertex[0][1]))

# initiallize for path finding
spanned_vertices = []
pathSet = []
# path[0]: f=g+h i.e. pathlength + distance to goal, path[i in range(1 to len)]: vertices
first_path = [0+np.linalg.norm(start-goal), start]
pathSet.append(first_path)

find_path_flag = 0
while (find_path_flag == 0):
    path_index = find_path_to_span(pathSet)
    find_path_flag, best_path = span_path(pathSet, path_index, spanned_vertices)

#draw best path
Route_best = line_create(0.1,0.1,0.8,2,0.08)
Route_best.points = []
for i in range(len(best_path)-1):
    line_point = Point()
    line_point.x = best_path[i][0]
    line_point.y = best_path[i][1]
    line_point.z = 0.0
    Route_best.points.append(line_point)
    line_point = Point()
    line_point.x = best_path[i+1][0]
    line_point.y = best_path[i+1][1]
    line_point.z = 0.0
    Route_best.points.append(line_point)
t = 0
while t <= 1:
    pub_bestroute.publish(Route_best)
    rospy.sleep(1)
    t += 1

# move robot to goal
for i in range(1, len(best_path),1):
    G = best_path[i]
    turn(G)
    while not(goal_reached(G)):
        follow_line(G)
    rospy.sleep(0.5)

# stop robot and shutdown the node
def shutdown():
    # Always stop the robot when shutting down the node.
    rospy.loginfo("Stopping the robot...")
    cmd_vel.publish(Twist())
    rospy.sleep(0.5)

shutdown()
