# TurtleBot Project
> This project is about implementing path planning algorithms on TurtleBot.

## Table of Contents
* [Technologies Used](#technologies-used)
* [User Guides](#user-guides)
* [Contact](#contact)
<!-- * [License](#license) -->


## Technologies Used
- ROS Noetic 
- Visual Studio Code

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


## Acknowledgements
Many thanks to Prof Dr H. Işıl Bozma.


## Contact
Created by [@uchihadmali](https://tr.linkedin.com/in/mehmet-ali-y%C4%B1ld%C4%B1r%C4%B1m-99465214a) - feel free to contact me!

Email: maliyldrm@yandex.com


<!-- Optional -->
<!-- ## License -->
<!-- This project is open source and available under the [... License](). -->

<!-- You don't have to include all sections - just the one's relevant to your project -->
