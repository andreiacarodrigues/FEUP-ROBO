# Assignment 2 - Robotics

Authors: Andreia Rodrigues (up201404691) and Tiago Filipe (up201610655)

### Requirements:
Ubuntu 16.04 LTS - https://www.ubuntu.com/download/desktop

ROS Kinetic - http://wiki.ros.org/kinetic/Installation/Ubuntu

STDR package - http://wiki.ros.org/stdr_simulator/Tutorials/Set%20up%20STDR%20Simulator

### Directory structure:
Create a new ROS package named assignment_2:
```
catkin_create_pkg assignment_2
```
The src folder in this zip file should replace the newly created package src folder.

Copy the files in the folder stdr_resources/maps to the stdr_simulator package maps folder (stdr_simulator/stdr_resources/maps).

Copy the files in the folder stdr_resources/robot to the stdr_simulator package robots folder (stdr_simulator/stdr_resources/resources/robots).

The launcher files can be copied from the stdr_resources/launchers to the package folder and executed from there.
In alternative you can copy them to the stdr_simulator/stdr_launchers/launch folder.

### How to compile:
Go to the package folder, open the terminal and execute the following commands:
```
catkin_make
source devel/setup.bash
```

### How to execute:
In the package folder open two terminals. The first one will run the stdr launcher with the selected map:

If they're in the package folder:
```
roslaunch v_map_going_left.launch <-- V map with the robot following the wall using the left side of the sensor
roslaunch v_map_going_right.launch <-- V map with the robot following the wall using the right side of the sensor
roslaunch w_map_going_left.launch <--  W map with the robot following the wall using the left side of the sensor
roslaunch w_map_going_right.launch <--  W map with the robot following the wall using the right side of the sensor
```
If they're in the stdr_launchers folder:
```
roslaunch stdr_launchers v_map_going_left.launch <-- V map with the robot following the wall using the left side of the sensor
roslaunch stdr_launchers v_map_going_right.launch <-- V map with the robot following the wall using the right side of the sensor
roslaunch stdr_launchers w_map_going_left.launch <--  W map with the robot following the wall using the left side of the sensor
roslaunch stdr_launchers w_map_going_right.launch <--  W map with the robot following the wall using the right side of the sensor
```
The second one will deploy the robot:
```
rosrun assignment_2 robot robot0 laser_0
```
