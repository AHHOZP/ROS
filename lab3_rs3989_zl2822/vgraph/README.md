Usage:
1. Copy package 'vgraph' into path /home/[your username]/catkin_ws/src/
2. Go to path /home/[your username]/catkin_ws/src/vgraph/src. Use 'chmod +x gen_obstacle.py' to make it executable
3. Use 'apt-get install python-numpy python-scipy' to install packages
4. Open a terminal, input 'roslaunch vgraph launch.launch'
5. Open a new terminal, Run script: 'rosrun vgraph gen_obstacle.py’

Method:

1. Bring up ArbotiX turtlebot simulator and RViz as described in Usage. 

2. Growing Obstacle
Define each set of obstacle vertices as a point set. Assume robot to be a 36 times 36cm square. Use its center as reference point. Place the reference point on vertices of each obstacle, then add four vertices of robot to the point set of each obstacle. Use scipy.spatial.ConvexHull to grow obstacle.

3. Create the vgraph of all safe paths 
Create a set of borderline of grown obstacle. Transform it to marker message then publish to topic '/vgraph_markers'
Fully connect all obstacle vertices + start + goal, create a set to store these segments (except the borderline). For each segment in the set, check whether it is intersect with a borderline using math method, if not, add it to a path set since it is safe for our robot to travel.
Transform path set list to marker message then publish to topic '/vgraph_markers'
Add borderline set to the safe path set.

4. Finding shortest path (A* search)
According to safe paths generated in 3, construct a sorted adjacency list of undirected graph. Use Euclidean distance between adjacent vertices as length of edge (g) and Euclidean distance to the goal as heuristic distance (h). Keep spanning vertices with smallest f=g+h, until the algorithm try to span the goal. Claim shortest path found and return it.

5. Following the path to goal
Transform point set of shortest path to marker message then publish to topic '/vgraph_markers'
Use function get_odom() to get odometry information of the robot. Control the robot to follow the line until goal reached.

Video:
https://youtu.be/bSUyPnG3sQA