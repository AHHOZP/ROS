Usage:
0. Put the file bug2.py under the path [your workspace]/src/rbx1/rbx1_nav/nodes
1. Open a Terminal, go to the absolute path of world files and run world file:
roslaunch turtlebot_gazebo turtlebot_world.launch world_file:=$PWD/bug2_x.world
x = 0, 1, 2, 3, 5, extra
2. Open another Terminal, set up workspace : source devel/setup.bash 
3. Run script: rosrun rbx1_nav bug2.py
After Robot reaches goal or concludes "impossible", in the Terminal of step 1, press ctrl+C to quit current world and repeat step 1 and 3 to try different world files.


Method:
Main loop:
while robot hasn't reached goal or concluded "impossible"
(1) Adjust direction if robot deviates from m-line
    Define a function using odom to get robot's position and check moving direction. If robot deviates too much from m-line, publish a message to make robot turn back a little. 
(2) Move forward a small distance
    Publish a message to topic'/cmd_vel_mux/input/teleop' to translate. Since m-line is exact x-axis, we just move forward.
(3) Detect obstacle
    Define scan_callback() to subscribe LaserScan. Use average of five range data to compute obstacle distance in front and on the right of the robot. If the distance is smaller than a threshold, conclude obstacle detected.
(4) If obstacle is detected in front of robot, first use odom to get position information of hit-point and store it. Then turn left until the object is no longer detected to the right of the robot. Follow the edge to bypass the obstacle. Move forward a small distance, check whether there is obstacle on the right. If not, turn slightly right. Otherwise, turn left and move forward (if there is also obstacle in front of robot, a larger angle is needed).
(5) Compute distance (with a little grace) to check whether the robot reaches m-line on a position closer to the goal, or returns to hit-point, or reaches goal
    a. If the robot reaches m-line, turn back toward m-line and keep going. Note that if robot's x position is smaller than hit-point or larger than goal (10, 0), we don't consider it as m-line.
    b. If the robot travels all around the obstacle (moved a relatively long distance) and returns to hit-point, conclude "impossible" and end the script
    c. If the robot reaches goal, end the script

Video: 
https://www.youtube.com/watch?v=wPKfZN5gt9Q&list=PLeKqY5m6EWZtbYpwi_CzqJ_c9pwTw5hU6&index=1

