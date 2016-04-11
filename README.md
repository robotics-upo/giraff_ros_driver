# giraff_ros_driver
ROS Catkin package for the Giraff telepresence platform.

The package is composed by three programs:

* *giraff_driver* is a driver for controlling the Giraff telepresence platform by using ROS.
* *giraff_teleop_joy* is a program for teleoperating the Giraff telepresence platform with a joystick by using ROS (*optional feature*).
* *TERESA firmware* is a firmware to be installed in the Giraff microcontroller board (AVR) in order to replace the original PID developed by Giraff with an improved version which is able to manage instant velocities.
 **This version of the firmware is not currently compatible with the Giraff Pilot software interface**.

The giraff_teleop_joy program is based on the following tutorial: 
*Writing a Teleoperation Node for a Linux-Supported Joystick* 
http://wiki.ros.org/joy/Tutorials/WritingTeleopNode 
under license http://creativecommons.org/licenses/by/3.0/

- See http://www.ros.org for more information about the ROS framework.
- See http://www.giraff.org for more information about the Giraff telepresence platform. 


## General requirements

* Giraff Telepresence platform.
  
* Computer to be located on the platform running Ubuntu Linux 14.04 and ROS Indigo. It is possible to use the inner GiraffPC replacing the Operative System (see *Hardware configuration using one PC*), it is also possible to use two PCs: The original inner GiraffPC running the Pilot Software under MS Windows and one extra PC running Ubuntu Linux 14.04 and ROS Indigo. (see *Hardware configuration using two PCs*)

* Two custom FTDI serial cables if you are using two PCs (see *Hardware configuration using two PCs*)

* An Inertial Measurement Unit (IMU) compatible with ROS (*optional*). The system has been tested with the *Xsens Mti 30* IMU. If you are not using an IMU, the odometry of the robot will be calculated by using the wheel encoders.

* A wireless joystick or gamepad with at least 8 buttons and 1 axis compatible with ROS (*optional*). The system has been tested with the *Logitech Wireless F710* gamepad. 


## Hardware configuration using one PC

The original hardware scheme is as follows:

Giraff PC <--- serial cable ---> Giraff microcontroller

The new hardware scheme is the following:

Linux PC <--- serial cable ---> Giraff microcontroller

Please, check the *no_giraff_pc* parameter in the parameters section below.

You can replace the MS Windows Operative System in the Giraff PC by Ubuntu 14.04 and ROS Indigo. 

If you want to use a different motherboard, please, be sure the board includes a serial port socket. 

If you want to use the original serial cable, the link is as follows:

* Rx-----------------Tx
* Tx-----------------Rx
* GND----------------GND
* DTR----------------DSR

If you want to use a custom FTDI cable, the link is as follows:

* Rx-----------------Tx
* Tx-----------------Rx
* GND----------------GND
* RTS----------------DSR

Please, check the *using_ftdi* parameter in the parameters section below.

If you are using an IMU, it should be located on the robot and connected to the Linux PC. Please, check the *using_imu* parameter in the parameters section below.

If you are using a joystick/gamepad, the receiver stick should be connected to the Linux PC.


Please, check the *no_giraff_pc* and *using_ftdi* parameters in the parameters section below.

## Hardware configuration using two PCs

The original hardware scheme is as follows:

Giraff PC <--- serial cable ---> Giraff microcontroller

It is needed to remove the serial cable linking the Giraff PC with the Giraff AVR and
substitute it by two custom FTDI serial cables as explained below. 

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
 
If you are using an IMU, it should be located on the robot and connected to the Linux PC. Please, check the *using_imu* parameter in the parameters section below.

If you are using a joystick/gamepad, the receiver stick should be connected to the Linux PC.

Please, check the *no_giraff_pc* and *using_ftdi* parameters in the parameters section below.

## Firware Installation

This package includes a firmware in order to replace the original PID developed by Giraff with an improved version which is able to manage instant velocities. This version of the firmware is not compatible with the Giraff Pilot software. It can be used with the hardware configuration of one or two PCs.
**If you are going to replace the MS Windows Operative System in the Giraff PC, please, install first the firmware**

In order to install the firmware, follow the next instructions:

* Be sure that the GiraffPC is directly connected to the AVR by the original serial cable included in the platform.

* Do a backup copy of the controller.upl file in the directory *C:/Program Files/cygwin/home/giraffe/telbot/software/avr/controller* in the GiraffPC.

* Replace the previous controller.upl file with the one included in this package.

* Start a cygwin application on the Giraff.

* Type "bin/runterm" to start the terminal application.

* Type "upload" to start the re-programming of the microcontroller.

* When the runterm application restarts, check that the version now is changed to 2.1.213u


Please, check the *using_teresa_pid* parameter in the parameters section below.

## How to command the robot by using ROS

If you are using the original Giraff firmware, the robot can be commanded by using the Giraff Pilot software or by using ROS commands. If you are using the new firmware, the robot can only be commanded by using ROS commands. Please, check the *using_teresa_pid* parameter in the parameters section below.

The Pilot sofware always has priority in order to control the robot, only after a configurable time without receiving motion commands from the Pilot software, the driver begins to accept motion commands from the next ROS topics:

* **/cmd_vel** of type **geometry_msgs::Twist** in order to get instant angular and linear velocities.

* **/stalk** of type **ros_giraff_driver::stalk** in order to get the commands for the heigth and tilt of the head. This topic is built-in with the package.

The format of the */stalk* topic is as follows:

* bool head_up
* bool head_down
* bool tilt_up
* bool tilt_down

If the value of *head_up* is *true*, the heigth of the head is increased at a constant rate. If the value of *head_down* is *true*, the heigth of the head is decreased at a constant rate. The behavior of *tilt_up* and *tilt_down* is analogous for the angle of the head.

## Required ROS topics

The next topic is required in order to run the *giraff_driver* program if you are using an IMU:

* **/imu/data** of type **sensor_msgs::Imu** in order to get the information of the IMU.

The next topic is required in order to run the *giraff_teleop_joy* program:

* **/joy** of type **sensor_msgs::Joy** in order get the input of the joystick/gamepad.


## Published ROS topics

The next topics are published by the *giraff_driver*: 

* **/odom** of type **nav_msgs::Odometry**

* **/batteries** of type **giraff_ros_driver::batteries** in order to publish the status of the batteries. This topic is built-in with the package.

* **/avr_comms** of type **giraff_ros_driver::avr_comms** in order to publish all the low-level communications with the AVR. This topic is built-in with the package.

* **/pilot** of type **giraff_ros_driver::pilot** in order to publish the low-level communications from the GiraffPC to the AVR. This topic is built-in with the package.

* **/cmd_vel_avr** of type **giraff_ros_driver::cmd_vel_avr** in order to publish the parameters sent to the AVR for each motion command received.

The next topics are published by the *giraff_teleop_joy*:

* **/cmd_vel** of type **geometry_msgs::Twist** in order to command the robot by reading the status of the joystick.

* **/stalk** of type **giraff_ros_driver::cmd_vel_avr** in order to command the height and tilt of the head of the robot by reading the status of the joystick. 

## Compilation
In order to build the package, download it to the *src* directory of your Catkin workspace and compile it by using *catkin_make* as normal.


## ROS parameters

Parameters of the *giraff_driver* program:

* **freq**: Frequency in hertzs of the main loop.

* **giraff_avr_port**: Path for the USB port to connect the AVR.

* **giraff_pc_port**: Path for the USB port to connect the Giraff PC.

* **max_linear_vel**: Maximun linear velocity in m/s accepted by the driver.

* **max_angular_vel**: Maximun angular velocity in rad/s accepted by the driver.

* **giraff_pc_timeout**: Timeout in seconds without receiving motion commands from the Pilot software to begin accepting motion commands from ROS.

* **tilt_bias**: Initial angle of the head.

* **using_imu**: Set to *1* if you are using an IMU, set to *0* if you are not using an IMU (the odometry will be calculated by using the wheel encoders).

* **no_giraff_pc**: Set to *1* if you are only using the LinuxPC, set to *0* if you are using both the Linux and the Giraff PC.

* **using_ftdi**: Set to *1* if you are using custom FTDI serial cables, set to *0* if you are using the original serial cable included in the platform in order to connect the LinuxPC to the AVR.

* **using_teresa_pid**: Set to *1* if you have installed the firmware included in this package, set to *0* if you are using the original firmware included in the Giraff platform.



Parameters of the *giraff_teleop_joy* program:

* **freq**: Frequency in hertzs of the main loop.

* **panic_freq**: Frequency in hertzs of the main loop in case of pushing the panic button in the joystick.

* **linear_velocity_axis**: Id of the axis to control the linear velocity.

* **angular_velocity_axis**: Id of the axis to control the angular velocity.

* **max_velocity_button**: Id of the button to allow the maximun velocity.

* **head_up_button**: Id of the button to move up the head of the robot.

* **head_down_button**: Id of the button to move down the head of the robot.

* **panic_button**: Id of the panic button in the joystick.

* **move_primary_button**: Id of the primary button to allow movements with the axis.

* **move_secundary_button**: Id of the secundary button to allow movements with the axis.

* **tilt_up_button**: Id of the button to increase the tilt angle of the head.

* **tilt_down_button**: Id of the button to decrease the tilt angle of the head.

* **max_linear_velocity**: Maximun allowed linear velocity in m/s.

* **max_angular_velocity**: Maximun allowd angular velocity in rad/s.



