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
### Running Real System
- Connect your PC to the Arduino Board and upload the voltage_control.ino code.
- Remove the connection between the 8-pin Hokuyo connector and the Hokuyo sensor.
- Connect the blue and brown ends of the Hokuyo connector to the voltage supply. Adjust the voltage to the 5V.
- Connect the 8-pin Hokuyo connector and the Hokuyo sensor. Make sure that the VCC and ground connections are correct.
- Connect the USB connection of the Hokuyo sensor and the PC.
- Open terminal and go to the workspace directory. Open different tabs and source them by the following command;

`source ./devel/setup.bash`
- Run the following commands in different terminals;

`roscore`

`rosrun rosserial_python serial_node.py /dev/ttyACM0`

`rosrun urg_node urg_node _serial_port:=/dev/ttyACM1`

`rosrun rviz rviz`
- In the opened Rviz window, do the followings;

Change "Global Options->Fixed Frame" to "map".

Add "PointCloud2".

Change "PointCloud2->Topic" to "/output".

- Run the following commands. This will initiate the mechanism movement and data should be seen on rviz.

`rosrun hokuyo_go laser_real`

- To stop the mechanism, disconnect the motor connections, kill all terminals and upload the voltage_control.ino code to the Arduino Board.
### Running Simulation
- Go to workspace directory and source several tabs.
- Run the following command

`roscore`
- Run the following command. Also you have different launch file options.

`roslaunch hokuyo_plus small_house_turtle.launch`

- Open Rviz.

`rosrun rviz rviz`

- Do the followings in rviz;

Change "Global Options->Fixed Frame" to "taban".

Add "PointCloud2".

Change "PointCloud2->Topic" to "/output".

- Run the following commands. These will start the movement and data should be visible at Rviz.

`rosrun hokuyo_go laser_to_pointcloud`

`rosrun hokuyo_go vel_publisher`


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
