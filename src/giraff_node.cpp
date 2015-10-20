/***********************************************************************/
/**                                                                    */
/** giraff_node.h                                                      */
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
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>			
#include <geometry_msgs/Twist.h>		
#include <sensor_msgs/Imu.h>
#include <giraff_ros_driver/Pilot.h>
#include <giraff_ros_driver/Stalk.h>
#include <giraff_ros_driver/StalkRef.h>
#include <giraff_ros_driver/cmd_vel_avr.h>
#include <giraff_ros_driver/giraff_manager.h>
#include <giraff_ros_driver/batteries.h>
#include <giraff_ros_driver/avr_comms.h>
#include <boost/bind.hpp>

#define GIRAFF_RADIUS 0.235

#define NODE_VERSION 1.2
#define FREQ	100.0

#define GIRAFF_AVR_PORT "/dev/ttyUSB0"  
#define GIRAFF_PC_PORT  "/dev/ttyUSB1"

#define MAX_LIN_VEL 0.6
#define MAX_ANG_VEL 1.5707963

#define GIRAFF_PC_TIMEOUT 5


GiraffManager *giraff = NULL;

ros::Time imu_time;
ros::Time cmd_vel_time;
bool imu_error=false;
bool head_up=false;
bool head_down=false;
bool tilt_up=false;
bool tilt_down=false;

double yaw=0.0;
double roll=0.0;
double pitch=0.0;
double ang_vel = 0.0;
double lin_vel = 0.0;
double pos_x = 0.0;
double pos_y = 0.0;

ros::Publisher *cmd_vel_avr_pub_ptr=NULL;

class GiraffAVRMonitorPublisher : public GiraffAVRMonitor
{
public:
	GiraffAVRMonitorPublisher(ros::Publisher& publisher) : publisher(publisher) {}
	virtual ~GiraffAVRMonitorPublisher() {}
	virtual void notifyCommand(const std::string& command)
	{
		this->command = command;
		init = ros::Time::now();
	}
	virtual void notifyResponse(const std::string& response)
	{
		giraff_ros_driver::avr_comms msg;
		msg.header.stamp = ros::Time::now();
		msg.init = init;
		msg.end = ros::Time::now();
		msg.command = command;
		msg.response = response;
		publisher.publish(msg);			
	} 

private:
	ros::Publisher& publisher; 
	std::string command;
	ros::Time init;
};


void cmdVelReceived(const geometry_msgs::Twist::ConstPtr& cmd_vel)
{
	cmd_vel_time = ros::Time::now();
	if (!imu_error) {
		GiraffState state = giraff->setVelocity( cmd_vel->linear.x,cmd_vel->angular.z);
		if (!state.giraffPC_control && cmd_vel_avr_pub_ptr!=NULL) {
			giraff_ros_driver::cmd_vel_avr cmd_vel_avr_msg;
			cmd_vel_avr_msg.header.stamp = ros::Time::now();
			cmd_vel_avr_msg.cmd_vel.linear.x = cmd_vel->linear.x;
			cmd_vel_avr_msg.cmd_vel.angular.z = cmd_vel->angular.z;
			cmd_vel_avr_msg.mode = state.mode;
			cmd_vel_avr_msg.a = state.a;
			cmd_vel_avr_msg.v = state.v;
			cmd_vel_avr_msg.vg = state.vg;
			cmd_vel_avr_msg.p = state.p;
			cmd_vel_avr_pub_ptr->publish(cmd_vel_avr_msg);
		}
	}
}

void imuReceived(const sensor_msgs::Imu::ConstPtr& imu)
{
	imu_time = ros::Time::now();
	static bool first_time=true;
	static ros::Time past_time = ros::Time::now();
	double duration = (imu->header.stamp - past_time).toSec();
	past_time=imu->header.stamp; 
	if(first_time){
		first_time=false;	
	 	return;
        }
		
	tf::Quaternion imu_quaternion;
    geometry_msgs::Quaternion imu_quaternion_msg;
    double yaw_aux;

    //Roll and Pitch come from imu_data
    //imu_quaternion_msg=imu->orientation;
    //tf::quaternionMsgToTF(imu_quaternion_msg, imu_quaternion);
    //tf::Matrix3x3(imu_quaternion).getRPY(roll,pitch,yaw_aux);

	if (giraff->isStopped() || fabs(imu->angular_velocity.z) < 0.04) {
		ang_vel = 0.0;
		return;
	}
	ang_vel = imu->angular_velocity.z;
	yaw += ang_vel * duration;
}


void stalkReceived(const giraff_ros_driver::Stalk::ConstPtr& stalk)
{
	if (stalk->head_up) {
		head_up=true;
		head_down=false;
	}
	else
	if (stalk->head_down) {
		head_up=false;
		head_down=true;
	}
	else {
		head_up=false;
		head_down=false;
	}
	
	if (stalk->tilt_up) {
		tilt_up=true;
		tilt_down=false;
	}
	else
	if (stalk->tilt_down) {
		tilt_up=false;
		tilt_down=true;
	}
	else {
		tilt_up=false;
		tilt_down=false;
	}
	
}

void stalkRefReceived(const giraff_ros_driver::StalkRef::ConstPtr& stalk_ref,
                      GiraffManager* giraff,
                      double tilt_bias)
{
        giraff->setStalk(stalk_ref->head_height);
        giraff->setTilt(stalk_ref->head_tilt + tilt_bias);
}



int main(int argc, char** argv)
{
	try
	{
	ros::init(argc, argv, "GiraffNode");
	ros::NodeHandle n;
	ros::NodeHandle pn("~");
	
	std::string giraff_avr_port;
	std::string giraff_pc_port;

	pn.param<std::string>("giraff_avr_port",giraff_avr_port,GIRAFF_AVR_PORT);
	pn.param<std::string>("giraff_pc_port",giraff_pc_port,GIRAFF_PC_PORT);

	std::string base_frame_id;
	std::string odom_frame_id;
	std::string head_frame_id;
    std::string stalk_frame_id;

	pn.param<std::string>("base_frame_id", base_frame_id, "/base_link");
	pn.param<std::string>("odom_frame_id", odom_frame_id, "/odom");
	pn.param<std::string>("head_frame_id", head_frame_id, "/giraff_head");	
    pn.param<std::string>("stalk_frame_id", stalk_frame_id, "/giraff_stalk");

	double max_lv;
	double max_av;
  	pn.param<double>("max_linear_vel", max_lv, MAX_LIN_VEL);
	pn.param<double>("max_angular_vel", max_av, MAX_ANG_VEL);

	int timeout;
	pn.param<int>("giraff_pc_timeout", timeout, GIRAFF_PC_TIMEOUT);
	
	

	double freq;
	pn.param<double>("freq",freq,FREQ);
	
	double tilt_bias;
	pn.param<double>("tilt_bias",tilt_bias, 0);
	///There's something weird with the tilt, which is not centered. A bias term may be needed

	ros::Publisher avr_comms_pub = pn.advertise<giraff_ros_driver::avr_comms>("/avr_comms",5);
	GiraffAVRMonitorPublisher monitor(avr_comms_pub);
	giraff = new GiraffManager(giraff_avr_port,giraff_pc_port,max_lv,max_av,timeout,monitor);

	ros::Publisher odom_pub = pn.advertise<nav_msgs::Odometry>(odom_frame_id, 5);
	ros::Subscriber cmd_vel_sub = n.subscribe<geometry_msgs::Twist>("/cmd_vel",1,cmdVelReceived);
	ros::Subscriber imu_sub = n.subscribe<sensor_msgs::Imu>("/imu/data",1,imuReceived);	
	ros::Subscriber stalk_sub=n.subscribe<giraff_ros_driver::Stalk>("/stalk",1,stalkReceived);
	ros::Subscriber stalk_ref_sub =
                n.subscribe<giraff_ros_driver::StalkRef>("/stalk_ref",1,boost::bind(&stalkRefReceived,_1, giraff, tilt_bias));
	ros::Publisher pilot_pub = pn.advertise<giraff_ros_driver::Pilot>("/pilot", 5);
	
	ros::Publisher cmd_vel_avr_pub = pn.advertise<giraff_ros_driver::cmd_vel_avr>("/cmd_vel_avr", 5);
	ros::Publisher batteries_pub = pn.advertise<giraff_ros_driver::batteries>("/batteries",5);
	cmd_vel_avr_pub_ptr = &cmd_vel_avr_pub;
	ros::Time current_time,last_time;
	imu_time = ros::Time::now();
	last_time = ros::Time::now();
	cmd_vel_time = ros::Time::now();
	ros::Rate r(freq);
	bool pilot_communication;
	std::string command;
	std::string response;
	tf::TransformBroadcaster tf_broadcaster;
	float imdl,imdr;
	double dt;
	bool first_time=true;
	float giraff_battery_level;	
	while (n.ok()) {
		current_time = ros::Time::now();		
		double imu_sec = (current_time - imu_time).toSec();
		if(imu_sec >= 0.25){
			giraff->setVelocity(0,0);
			ang_vel = 0;
			imu_error = true;
			ROS_WARN("-_-_-_-_-_- IMU STOP -_-_-_-_-_- imu_sec=%.3f sec",imu_sec);
		} else {
			imu_error = false;
		}
		double cmd_vel_sec = (current_time - cmd_vel_time).toSec();
		if (cmd_vel_sec >= 0.5) {
			giraff->setVelocity(0,0);
		}

		giraff->getIMD(imdl,imdr);
		dt = (current_time - last_time).toSec();
		last_time = current_time;
		if (!first_time) {
			double imd = (imdl+imdr)/2;
			lin_vel = imd / dt;
			pos_x += imd*cos(yaw + ang_vel*dt/2);
			pos_y += imd*sin(yaw + ang_vel*dt/2);
		} else {
			first_time = false;
		}
		
		pilot_communication = giraff->update(command,response);
		
		if (head_up) {
			giraff->incStalk();
		} else if (head_down) {
			giraff->decStalk();
		}
		if (tilt_up) {
			giraff->incTilt();
		} else if (tilt_down) {
			giraff->decTilt();
		}
		// ******************************************************************************************
		//first, we'll publish the transforms over tf
		geometry_msgs::TransformStamped odom_trans;
		odom_trans.header.stamp = current_time;
		odom_trans.header.frame_id = odom_frame_id;
		odom_trans.child_frame_id = base_frame_id;
		odom_trans.transform.translation.x = pos_x;
		odom_trans.transform.translation.y = pos_y;
		odom_trans.transform.translation.z = 0.0;
		odom_trans.transform.rotation = tf::createQuaternionMsgFromRollPitchYaw(roll, pitch, yaw);
		tf_broadcaster.sendTransform(odom_trans);
		
		float altura_en_metros = (116 + 0.025 * giraff->getStalk()) * 0.01;
        float tilt_en_rad = giraff->getTilt() - tilt_bias;

        geometry_msgs::TransformStamped stalk_trans;
        stalk_trans.header.stamp = current_time;
        stalk_trans.header.frame_id = base_frame_id;
        stalk_trans.child_frame_id = stalk_frame_id;
        stalk_trans.transform.translation.x = 0.0;
        stalk_trans.transform.translation.y = 0.0;
        stalk_trans.transform.translation.z = altura_en_metros;
        stalk_trans.transform.rotation = tf::createQuaternionMsgFromRollPitchYaw(0.0, 0.0, 0.0);
        tf_broadcaster.sendTransform(stalk_trans);

		geometry_msgs::TransformStamped head_trans;
		head_trans.header.stamp = current_time;
		head_trans.header.frame_id = stalk_frame_id;
		head_trans.child_frame_id = head_frame_id;
		head_trans.transform.translation.x = 0.0;
		head_trans.transform.translation.y = 0.0;
		head_trans.transform.translation.z = 0.0;
		head_trans.transform.rotation = tf::createQuaternionMsgFromRollPitchYaw(0.0, -tilt_en_rad, 0.0);
		tf_broadcaster.sendTransform(head_trans);

		// ******************************************************************************************
		//next, we'll publish the odometry message over ROS
		nav_msgs::Odometry odom;
		odom.header.stamp = current_time;
		odom.header.frame_id = odom_frame_id;
		
		//set the position
		odom.pose.pose.position.x = pos_x;
		odom.pose.pose.position.y = pos_y;
		odom.pose.pose.position.z = 0.0;
		odom.pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(roll, pitch, yaw);
		
		//set the velocity
		odom.child_frame_id = base_frame_id;
		odom.twist.twist.linear.x = lin_vel;
		odom.twist.twist.linear.y = 0.0; 
		odom.twist.twist.angular.z = ang_vel;
		
		//publish the odometry
		odom_pub.publish(odom);

		//publish Pilot command/response
		if (pilot_communication) {
			giraff_ros_driver::Pilot pilotmsg;
			pilotmsg.header.stamp = current_time;
			pilotmsg.command = command;
			pilotmsg.response = response;
			pilot_pub.publish(pilotmsg);			
		}
		
		

		//publish the state of the batteries
		if (giraff->getBattery(giraff_battery_level)) {
			giraff_ros_driver::batteries battmsg;
			battmsg.header.stamp = current_time;
			battmsg.giraff_battery = giraff_battery_level;
			batteries_pub.publish(battmsg);	
		}

				
		r.sleep();	
		ros::spinOnce();
	
	}
	
	}
	catch(GiraffManagerException e) {
		ROS_WARN("%s",e.what());
		if (giraff!=NULL) {
			delete giraff;		
		}
		ROS_BREAK();
	}
	delete giraff;
	return 0;

}

