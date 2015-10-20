# giraff_ros_driver
ROS Catkin package for the Giraff telepresence platform.

The package is composed by two programs:

* *giraff_driver* is a driver for controlling the Giraff telepresence platform by using ROS.
* *giraff_teleop_joy* is a program for teleoperating the Giraff telepresence platform with a joystick by using ROS. 

The giraff_teleop_joy program is based on the following tutorial: 
*Writing a Teleoperation Node for a Linux-Supported Joystick* 
http://wiki.ros.org/joy/Tutorials/WritingTeleopNode 
under license http://creativecommons.org/licenses/by/3.0/

- See http://www.ros.org for more information about the ROS framework.
- See http://www.giraff.org for more information about the Giraff telepresence platform. 
- See http://robotics.upo.es for more information about the authors of this package.


## Requirements

* Giraff Telepresence platform.
* Updated version of the Giraff microcontroller firmware.
  Please ask Giraff (http://www.giraff.org) for the appropriate version for your platform.
  **If you install an incorrect version, you could damage the platform**
  
* Computer to be located on the platform running Ubuntu Linux 14.04 and ROS Indigo

* Serial cables


## Hardware configuration

The original hardware scheme is as follows:

Giraff PC <--- serial cable ---> Giraff microcontroller

It is needed to remove the serial cable linking the Giraff PC with the Giraff AVR and
substitute it by two custom cables as explained below. 

The new hardware scheme is the following:

Giraff PC <---custom cable1--->  Linux PC  <---custom cable2---> Giraff microcontroller 

The links for *custom cable1* are:
* Rx-----------------Tx
* Tx-----------------Rx
* GND----------------GND
* DTR----------------RTS


The links for *custom cable2* are:

* Rx-----------------Tx
* Tx-----------------Rx
* GND----------------GND
* RTS----------------DSR
 
## Compilation
In order to build the package, download it to the *src* directory of your Catkin workspace and compile it by using *catkin_make* as normal.

## Parameters
