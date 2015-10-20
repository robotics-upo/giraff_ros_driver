/***********************************************************************/
/**                                                                    */
/** giraff_teleop_joy.h                                                */
/**                                                                    */
/** Copyright (c) 2015, Service Robotics Lab.                          */ 
/**                     http://robotics.upo.es                         */
/**                                                                    */
/** All rights reserved.                                               */
/**                                                                    */
/** Authors:                                                           */
/** Ignacio Perez-Hurtado (maintainer)                                 */
/** Noe Perez                                                          */
/** Rafael Ramon                                                       */
/** Fernando Caballero                                                 */
/** Jesus Capitan                                                      */
/** Luis Merino                                                        */
/**                                                                    */   
/** This software may be modified and distributed under the terms      */
/** of the BSD license. See the LICENSE file for details.              */
/**                                                                    */
/** http://www.opensource.org/licenses/BSD-3-Clause                    */
/**                                                                    */
/***********************************************************************/


#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>
#include <giraff_ros_driver/Stalk.h>

/////////////////////////////////

#define FREQ                 15 
#define PANIC_FREQ          200
#define LINEAR_VELOCITY_AXIS  1
#define ANGULAR_VELOCITY_AXIS 0
#define MAX_VELOCITY_BUTTON   0
#define HEAD_UP_BUTTON        3
#define HEAD_DOWN_BUTTON      1
#define PANIC_BUTTON          2
#define MOVE_PRIMARY_BUTTON   4
#define MOVE_SECUNDARY_BUTTON 5
#define TILT_UP_BUTTON        7
#define TILT_DOWN_BUTTON      6
#define MAX_LINEAR_VELOCITY   0.6
#define MAX_ANGULAR_VELOCITY  1.5707963

/////////////////////////////////

int linearVelocityAxis ;
int angularVelocityAxis;
int maxVelocityButton;
int headUpButton;
int headDownButton;
int panicButton;
int movePrimaryButton;
int moveSecundaryButton;
int tiltUpButton;
int tiltDownButton;
double maxLinearVelocity;
double maxAngularVelocity;

//////////////////////////////////

double currentLinearVelocity  = 0;
double currentAngularVelocity = 0;

//////////////////////////////////

bool publishCmdVel = false;
bool panic         = false;

ros::Publisher *stalk_pub_ptr=NULL;

void joyReceived(const sensor_msgs::Joy::ConstPtr& joy)
{
	bool prev_panic = panic;
	panic = joy->buttons[panicButton]==1;
	
	if (!prev_panic && panic && stalk_pub_ptr!=NULL) {
		giraff_ros_driver::Stalk stalk;
		stalk.head_up = false;
		stalk.head_down = false;
		stalk.tilt_up = false;
		stalk.tilt_down = false;
		stalk_pub_ptr->publish(stalk);	
	}

	if (panic) {
		currentLinearVelocity=0;
		currentAngularVelocity=0;
		return;
	}

	publishCmdVel = joy->buttons[movePrimaryButton]==1 || joy->buttons[moveSecundaryButton]==1;
	
	double multiplier = (joy->buttons[maxVelocityButton]==0)?0.5:1.0;
	currentAngularVelocity = maxAngularVelocity*multiplier*joy->axes[angularVelocityAxis];
	currentLinearVelocity = maxLinearVelocity*multiplier*joy->axes[linearVelocityAxis];
	
	if (stalk_pub_ptr==NULL) {
		return;
	}

	giraff_ros_driver::Stalk stalk;
	
	if (joy->buttons[headUpButton]==1) {
		stalk.head_up = true;
		stalk.head_down = false;
	}
	else
	if (joy->buttons[headDownButton]==1) {
		stalk.head_up = false;
		stalk.head_down = true;
	}
	else {
		stalk.head_up = false;
		stalk.head_down = false;
	}
	
	if (joy->buttons[tiltUpButton]==1) {
		stalk.tilt_up = true;
		stalk.tilt_down = false;
	}
	else
	if (joy->buttons[tiltDownButton]==1) {
		stalk.tilt_up = false;
		stalk.tilt_down = true;
	}
	else {
		stalk.tilt_up = false;
		stalk.tilt_down = false;
	}
		
	stalk_pub_ptr->publish(stalk);	



}


void sendCmdVel(double linearVelocity, double angularVelocity, ros::Publisher& vel_pub)
{
	geometry_msgs::Twist vel;
	vel.angular.z = angularVelocity;
	vel.linear.x = linearVelocity;
	vel.linear.z = 0;
	vel.linear.y = 0;
	vel_pub.publish(vel);		

}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "GiraffTeleopJoy");
	ros::NodeHandle n;
	ros::NodeHandle pn("~");
	
	double freq;
	pn.param<double>("freq",freq,FREQ);
	double panicFreq;
	pn.param<double>("panic_freq",panicFreq,PANIC_FREQ);
	pn.param<int>("linear_velocity_axis",linearVelocityAxis,LINEAR_VELOCITY_AXIS);
	pn.param<int>("angular_velocity_axis",angularVelocityAxis,ANGULAR_VELOCITY_AXIS);
	pn.param<int>("max_velocity_button",maxVelocityButton,MAX_VELOCITY_BUTTON);
	pn.param<int>("head_up_button",headUpButton,HEAD_UP_BUTTON);	
	pn.param<int>("head_down_button",headDownButton,HEAD_DOWN_BUTTON);
	pn.param<int>("panic_button",panicButton,PANIC_BUTTON);
	pn.param<int>("move_primary_button",movePrimaryButton,MOVE_PRIMARY_BUTTON);
	pn.param<int>("move_secundary_button",moveSecundaryButton,MOVE_SECUNDARY_BUTTON);
	pn.param<int>("tilt_up_button",tiltUpButton,TILT_UP_BUTTON);
	pn.param<int>("tilt_down_button",tiltDownButton,TILT_DOWN_BUTTON);
	pn.param<double>("max_linear_velocity",maxLinearVelocity,MAX_LINEAR_VELOCITY);
	pn.param<double>("max_angular_velocity",maxAngularVelocity,MAX_ANGULAR_VELOCITY);

	ros::Publisher vel_pub = pn.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
	ros::Publisher stalk_pub = pn.advertise<giraff_ros_driver::Stalk>("/stalk",1);
	stalk_pub_ptr = &stalk_pub;
	ros::Subscriber joy_sub = n.subscribe<sensor_msgs::Joy>("/joy", 5, joyReceived);		

	ros::Rate rate(freq);
	ros::Rate panicRate(panicFreq);
	bool prevPublishCmdVel=false;
	while (n.ok()) {
		if (panic) {
			sendCmdVel(0,0,vel_pub);
			panicRate.sleep();
		}
		else {
			if (publishCmdVel) {
				sendCmdVel(currentLinearVelocity, currentAngularVelocity, vel_pub);	
			}
			else
			if (prevPublishCmdVel) {
				sendCmdVel(0,0,vel_pub);
			}
			prevPublishCmdVel = publishCmdVel;
			rate.sleep();
		}
		ros::spinOnce();
	}
	sendCmdVel(0,0,vel_pub);
	return 0;
}



