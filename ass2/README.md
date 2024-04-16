# Ball Collection Turtlebot
## Overview
This ROS project enables the Turtlebot 3 Burger to navigate to a series of random placed red balls and place them on top of a random placed blue cube autonomously within a Gazebo simulated environment. 
The project is organized within the ass2 directory of the catkin_ws.

## How to Run the Project
1. Start Environment
Open Terminal 1 and launch the Turtlebot 3 Burger in our custom world:
'roslaunch turtlebot3_gazebo turtlebot3_our_world.launch'

2. Map the Environment
In Terminal 2, initiate SLAM for mapping the environment:
 'roslaunch turtlebot3_slam turtlebot3_slam.launch'
This command also launches RViz.

In Terminal 3, initialize the teleoperation node to drive the Turtlebot around for environment exploration:
'roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch'
Explore the environment until satisfied. 

Then, in Terminal 4, save the map:
'rosrun map_server map_saver -f ~/map'
Once the map is saved, close Terminals 1, 2, and 4.

3. Navigate in the Environment
Relaunch the Turtlebot in our world in Terminal 5:
'roslaunch turtlebot3_gazebo turtlebot3_our_world.launch'

Then, in Terminal 6, initiate navigation using the previously saved map:
'roslaunch turtlebot3_navigation turtlebot3_navigation.launch map_file:=$HOME/map.yaml'
Move the robot around slightly using Terminal 3 until the map aligns properly in RViz. Once the robots position is identified, close Terminal 3.

now close all terminals as we are ready to start the project!

4. Start the Project Itself
Open a new terminal and launch the project:
'roslaunch ass2 ass2.launch'
this will start all that is needed for the simulation, navigation, services, and the control_script.

Now, observe the Turtlebot autonomously collecting the red balls and placing them on top of the blue cube.
moreover, the action that the robot does was collected to a file name output.txt that can be found also in this folder.

Contributors
Guy 206922544
Ron 209260645
