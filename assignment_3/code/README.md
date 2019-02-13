# Robot Environment Interaction Using TurtleBot

Andreia Rodrigues, Eduardo Leite, Francisco Queirós and Vincent De Clercq for Robotics (ROBO) course unit from Master in Informatics and Computing Engineering at FEUP.
Developed and tested using a Ubuntu 16.04.5 system with ROS Kinetic.

### Installation intructions:

1. Follow [setup for the TurtleBot3](http://emanual.robotis.com/docs/en/platform/turtlebot3/pc_setup/).

2. In the created Catkin workspace add this repository as a package.

3. Build the workspace with the following command:
    ```
    catkin_make
    ```

### To run the software
1. Setup
    * Physical TurtleBot3:
        1. Follow [setup for the TurtleBot3](http://emanual.robotis.com/docs/en/platform/turtlebot3/joule_setup/#install-linux-ubuntu).
	    2. Bringup the sensors by following [point 7.1 and 7.2 of this guide](http://emanual.robotis.com/docs/en/platform/turtlebot3/bringup/#bringup). (turtlebot3_robot.launch and turtlebot3_realsense.launch necessary)

    * Open a second terminal and deploy the robot:
		1. Open the simulation world on Gazebo by running:
		    ```
		    roslaunch robofinal world.launch
		    ```
2. Run the program by executing the following command while having the Catkin workspace as work directory of the shell
    ```
    rosrun robofinal robofinal <execution mode>
    ```
    Where execution mode should be specified like so:
	* 0 - Follow Wall
	* 1 - Follow Line
	* 2 - Follow Object
	* 3 - Avoid Object


