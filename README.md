# TurtleBot Project
> This project is about implementing path planning algorithms on TurtleBot.
> Live demo [_here_](https://youtu.be/o_cbM7b2nys). <!-- If you have the project hosted somewhere, include the link here. -->

## Table of Contents
* [Technologies Used](#technologies-used)
* [Robot Image](#robot-image)
* [Setup](#setup)
* [User Guides](#user-guides)
* [Developer Guides](#developer-guides)
* [Room for Improvement](#room-for-improvement)
* [Contact](#contact)
<!-- * [License](#license) -->


## Technologies Used
- ROS Noetic 
- Visual Studio Code
- Arduino IDE 1.8.19

## Robot Image
![Hokuyo Plus Robot](https://cdn.discordapp.com/attachments/1117499134174314577/1117499582922903662/sistem_foto.jpeg)

## Setup
Do the followings to be able to run the system;
- [Dual Bot Ubuntu 20.04 Installation](https://youtu.be/HhJ1WaNJJqA)
- [Install ROS Noetic](http://wiki.ros.org/noetic/Installation)
- [Install Arduino IDE 1.8.19](https://docs.arduino.cc/software/ide-v1/tutorials/Linux)


## User Guides

### Instructions to Run the Simulation

This README provides detailed instructions to run the simulation. Follow the steps below:

#### Step 1 - Modify File Paths

First, change the URDF and YAML file paths in the gazebo_c.cpp and gazebo_c_2.cpp files.

#### Step 2 - Build Package

In the terminal, use the command catkin_make to build your catkin workspace.

#### Step 3 - Launch Simulation

Run the command roslaunch gazebo_sim create.launch to start the simulation.

Note: If your computer is slow, create.launch may not execute properly. In this case, run the commands below separately in different terminals:

    roscore
    rosrun gazebo_ros gazebo
    rosrun gazebo_sim gazebo_ros_node
    rosrun target_finder target_finder_node_3
    rosrun gazebo_sim gazebo_ros_node2
    rosrun target_finder target_finder_node_2
    rosrun navigation2 navigation_node2
    rosrun navigation navigation_node

Expected Result

The above commands will open Gazebo, spawning two robots and one red obstacle.

The robot with the green back is the leader, while the other robot is the follower. If the code is executed properly, both robots will move towards their goal, with the follower robot tailing the leader.


## Developer Guides
### ROS-ARDUINO communication
For Arduino ROS communication we used this [link](https://maker.pro/arduino/tutorial/how-to-use-arduino-with-robot-operating-system-ros).

### PID speed control
For this part watch this [video](https://youtu.be/HRaZLCBFVDE).

For velocity filter choice you may use the velocity_measure.ino code.

Also for PID parameter determination you may use the velocity_pid_test.ino code.

### Saved Pointcloud Visualization
There are 2 ways for this.
- You can visualize local pointcloud files via Visual Studio Code. There is a pcl library in Visual Studio Code.
- You can visualize by rviz. Run the following commands and change base frame to /base_link add pointcloud2 in rviz 

`roscore`

`rosrun rviz rviz`
- Do the followings in rviz;

Change "Global Options->Fixed Frame" to "/base_link".

Add "PointCloud2".

Change "PointCloud2->Topic" to "/cloud_pcd".

- Run the following command and you will see the data in rviz.

`rosrun pcl_ros pcd_to_pointcloud pointcloud_002.pcd 0.1`



### Writing Movement Algorithm Code
In simulation you need to change the vel_publisher.cpp code to write a movement algorithm.

In real life system, you need to change the laser_real.cpp code.

## Room for Improvement
To do:
- 3-tilt motion code needs update. APF(artificial potential field) 
should be used to go to the positions 120,-120,0
- Theres an issue with pan motion in simulation. Probably the l_2 distance is wrong. Needs to be fixed.
- Constant velocity movement has very noisy result. Should be fixed.
- Kinematic of the simulation is not the same with the real world.
- There is a saving issue for the 3-tilt motion. Needs to be fixed.
- Pan motion should be as fast as possible in 3-tilt.
- 3-tilt tilt motion should be improved. (ex. vel=1/distance).
- [Better Hokuyo](https://www.robotshop.com/products/hokuyo-ust-10lx-scanning-laser-rangefinder) can be used.


## Acknowledgements
Many thanks to Prof Dr H. Işıl Bozma.


## Contact
Created by [@uchihadmali](https://tr.linkedin.com/in/mehmet-ali-y%C4%B1ld%C4%B1r%C4%B1m-99465214a) - feel free to contact me!

Email: maliyldrm@yandex.com


<!-- Optional -->
<!-- ## License -->
<!-- This project is open source and available under the [... License](). -->

<!-- You don't have to include all sections - just the one's relevant to your project -->
