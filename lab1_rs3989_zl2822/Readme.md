Usage:
0. Put the file timed_out_and_back.py under the path [your workspace]/src/rbx1/rbx1_nav/nodes
1. Open a Terminal, set up a turtlebot environment using roslaunch rbx1_bringup fake_turtlebot.launch
2. Open a new Terminal, run our script: rosrun rbx1_nav timed_out_and_back.py
3. Open another new Terminal, start an rviz simulator: rosrun rviz rviz -d `rospack find rbx1_nav`/sim.rviz
4. Back to the Terminal of step 2, input operation command: T for translation, R for rotation, or Q for quit.
(a) For translation, input a positive number in meter to translate forward or a negative number to translate backward.
(b) For rotation, input a positive number in degree to rotate counterclockwise or a negative number to rotate clockwise.
You can continue input a new command when last command is done, until you input Q.

Method:
0. Create a node. Create a publisher to send Twist message in order to control robot's speed. Set publishing rate.
1. Initialize variables to represent commands. Wait for user to input a character as command. 
2.  According to user's input, do corresponding operation like setting publish rate and moving speed. 
  2.1 If 't' or 'T', get user's input of distance to translate. Compute parameters of movement:
       If distance < 0, give a negative linear speed to the message.
       Else, give a positive linear speed to the message.
       Compute duration of translation and transform it to ticks (times to publish).
       Publish the Twist message for ticks times, then send an empty Twist message to stop robot.
  2.2 If 'r' or 'R', get user's input of angle to rotate. Compute parameters of movement:
       If angle < 0, give a negative angular speed to the message.
       Else, give a positive angular speed to the message.
       Compute duration of rotation and transform it to ticks (times to publish).
       Publish the Twist message for ticks times, then send an empty Twist message to stop robot.
  2.3 If 'q' or 'Q', break the loop.
  2.4 Else, give a warning of invalid input then continue listen to input.

Video: https://youtu.be/6XKp3sP2S1A
