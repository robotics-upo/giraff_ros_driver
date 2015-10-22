# giraff_ros_driver
ROS Catkin package for the Giraff telepresence platform.

The package is composed by two programs:

* *giraff_driver* is a driver for controlling the Giraff telepresence platform by using ROS.
* *giraff_teleop_joy* is a program for teleoperating the Giraff telepresence platform with a joystick by using ROS (*optional feature*).

The giraff_teleop_joy program is based on the following tutorial: 
*Writing a Teleoperation Node for a Linux-Supported Joystick* 
http://wiki.ros.org/joy/Tutorials/WritingTeleopNode 
under license http://creativecommons.org/licenses/by/3.0/

- See http://www.ros.org for more information about the ROS framework.
- See http://www.giraff.org for more information about the Giraff telepresence platform. 


## General requirements

* Giraff Telepresence platform.
* Updated version of the Giraff microcontroller firmware.
  Please ask Giraff (http://www.giraff.org) for the appropriate version for your platform.
  **If you install an incorrect version, you could damage the platform**
  
* Computer to be located on the platform running Ubuntu Linux 14.04 and ROS Indigo.

* Serial cables.

* An Inertial Measurement Unit (IMU) compatible with ROS. The system has been tested with the *Xsens Mti 30* IMU.

* A wireless joystick or gamepad with at least 8 buttons and 1 axis compatible with ROS (*optional*). The system has been tested with the *Logitech Wireless F710* gamepad. 


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
 
The IMU should be located on the robot and connected to the Linux PC.

The receiver stick of the joystick/gamepad should be connected to the Linux PC.


## Compatibility with the Giraff Pilot software

The original Giraff platform includes a teleoperation software called Pilot to control the robot. 

The driver is compatible with the Pilot software in a transparent fashion.


## How to command the robot by using ROS

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

The next topic is required in order to run the *giraff_driver* program:

* **/imu/data** of type **sensor_msgs::Imu** in order to get the information of the IMU.

The next topics is required in order to run the *giraff_teleop_joy* program:

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


## Parameters

Parameters of the *giraff_driver* program:

* **freq**: Frequency in hertzs of the main loop.

* **giraff_avr_port**: Path for the USB port to connect the AVR.

* **giraff_pc_port**: Path for the USB port to connect the Giraff PC.

* **max_linear_vel**: Maximun linear velocity in m/s accepted by the driver.

* **max_angular_vel**: Maximun angular velocity in rad/s accepted by the driver.

* **giraff_pc_timeout**: Timeout in seconds without receiving motion commands from the Pilot software to begin accepting motion commands from ROS.

* **tilt_bias**: Initial angle of the head.



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


## TODO

- Make optional the connection with the IMU
- Make optional the connection with the GiraffPC
