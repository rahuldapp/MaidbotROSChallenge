# MaidbotROSChallenge
ROS package for Maidbot ROS Challenge

MAIDBOT ROS CHALLENEGE

About: This is a ROS package developed for Maidbot ROS Challenge.

INSTALL: 

- Clone the above package in the src folder of a catkin workspace. 

- Compile the workspace using catkin_make or catkin build as per convenience.

RUN:

- Launch the program using: 

	$ roslaunch robot_with_laser robot_with_laser.launch

DESCRIPTION:

The ROS package robot_with_laser has two nodes, run_robot and run_laser. 

run_laser: Publishes fake LaserScan data from a sensor. Users can plug and play this node with two different sensor configurations using the parameters in the launch file.
I assumed there would be two laser sensors with different configurations so that, we can plug one at a time and see the results. 
LaserOne would publish half data at a minimum range provided and half data at the maximum range provided, where LaserTwo would publish one third data at min range and rest at the max range.
Field of View limits, range limits and number of samples per sensor can be changed in the launch file. Both sensors' data would be published at a maximum frequency of 30Hz.
The sensor is attached to the robot at (0,0,0.3) and is oriented with the robot ie., has the same orientation as the robot.

Here are the sensor configuations:

LaserOne

--------

Min Range: 0.5m

Max Range: 20.0m

Min FOV: -PI rad

Max FOV: PI rad

Max Samples: 720


LaserTwo

--------

Min Range: 0.2m

Max Range: 10.0m

Min FOV: -PI rad

Max FOV: PI rad

Max Samples: 1440

run_robot: This makes the robot (base_link frame) move in a circle. The robot revolves clockwise and counter clockwise per revolution while rotating about itself.