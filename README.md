[![License](https://img.shields.io/badge/License-BSD%203--Clause-blue.svg)](https://opensource.org/licenses/BSD-3-Clause)

<h1>ROS Beginner Tutorial - Publisher and Subscriber </h1>

</p>
<p align="center">
<img src="/Images/ROSlogo.png">
</p>

</p>
<p align="center">
Reference for image: <a href='http://www.ros.org/'>link</a>
</p>

## Project Overview
The project covers creating publisher and subscriber in ROS. It has two following nodes:
* Publisher node - talker.cpp
* Subscriber node - listener.cpp

Publisher node publishes a custom string message on the chatter topic. The Subscriber node subscribes the topic and display the string message published by the talker. Both publisher and subscriber nodes are written in C++ language.

## Dependencies

These ROS nodes are made to be used on systems which have:
* ROS Kinetic
* Ubuntu 16.04

To install ROS, follow the instructions on this [link](http://wiki.ros.org/kinetic/Installation)

## Build Instructions
#### Build Catkin Workspace
Open terminal and run the following command to clone this repository in it
```
$ git clone -b Week10_HW https://github.com/krawal19/beginner_tutorials.git catkin_ws/src/beginner_tutorials
$ cd catkin_ws
```
#### Build the package
Use the below command to build the ROS package
```
$ catkin_make
```
## Running Instructions

To run code using launch command, open a new terminal window and run following command
```
$ cd <path to catkin_ws>
$ source devel/setup.bash
$ rosrun beginner_tutorials launchFile.launch
```
To run the launch command using arguments
```
$ cd <path to catkin_ws>
$ source devel/setup.bash
$ rosrun beginner_tutorials launchFile.launch result:=passed
```
To run each node separately using roscore , open a new terminal window and run following command
```
$ source /opt/ros/kinetic/setup.bash
$ roscore
```
Then to run talker node, open a new terminal window and run following command
```
$ cd <path to catkin_ws>
$ source devel/setup.bash
$ rosrun beginner_tutorials talker <Enter passed or failed>
```
Then to run listener node, open a new terminal window and run following command
```
$ cd <path to catkin_ws>
$ source devel/setup.bash
$ rosrun beginner_tutorials listener
```
## Service
To run talker node service, type the following in a new terminal after starting roscore and talker node from the methods mentioned above.
After running the command the publisher message will change to "Text for simple" as mentioned in input string.
```
rosservice call /changeString Igotpassed
```

## Logging
To visualise logger messages in Qt-based framework, run the commands below after running roscore and nodes as mentioned above
```
rosrun rqt_console rqt_console
```
## Graphical visualization of ROS Nodes
Open a new terminal and run following command
```
$ rqt_graph
```
<p align="center">
<img src="Images/rqt_graph.jpg" width="70%" height="70%">
</p>
