Usage:
1. Copy file 'rrt.py' into folder 'rrt-master'
2. Go to path .../rrt-master
3. For part1, open a terminal, input "python rrt.py"
4. For part2, open a terminal, input "python rrt.py --birrt"
5. For extra, open a terminal, input "python rrt.py --birrt --smoothing"

Method: 
1. rrt
Create a list as our tree. Each element of the list stores a node and its parent. 
Give a bias (50% chance in this case) to generate a new node toward goal from the nearest node in the tree with step-size of 0.1, if the new node does not collide with obstacles. 
Otherwise, pick a random point in space and generate a new node toward it from the nearest node in the tree with step-size of 0.1. If new node cannot pass collision check, keep trying another random point until new node successfully generated.
Every time when a new node is generated, try to connect it with the goal. If the goal can be reached, use a list to record path from start to goal and then return the list.

2. birrt
Create two tree lists with data structure as above. One (start tree) contains all nodes and relationships from start node, and the other (goal tree) contains all nodes and relationships from goal node.
Pick a random point, and generate a node toward it from the nearest node in one tree (for the first time, generate form start tree) with step-size of 0.1. If new node cannot pass collision check, keep trying another random point until new node q_new successfully generated. 
From the other tree, find the nearest node of the just generated node, expand the tree toward q_new with length of 0.1 if safe.
Keep swapping two trees for expansion towards the other tree as described until they meet. Every time when a new node is generated, try to connect it with the other tree. If two trees can connect, use a list to record path from start to goal. Then return the list.

3. birrt_smoothing
Get the path from birrt.
Choose two random nodes in the path (there should be at least one node between them so that smoothing can be needed).
Try to connect two nodes through generating a series of nodes with step-size of 0.1. If all these nodes pass collision check, the path can be smoothed. Replace all nodes in origin path between two chosen nodes by these new generated nodes, and return the new path.
Try this procedure 100 times, route can be smoothed.



Video: https://youtu.be/hr_t3pTxZLw
