/***********************************************************************/
/**                                                                    */
/** giraff_manager.h                                                   */
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


#ifndef _GIRAFF_MANAGER_H_
#define _GIRAFF_MANAGER_H_

#define   PI 3.14159265

#define ACELERATION 0.6
#define MAX_STALK_HEIGHT 1.410
#define MIN_STALK_HEIGHT 1.160

// Activate this to show some debug information
//#define _GIRAFF_MANAGER_DEBUG_


// Activate if you want to filter the "set tilt_angle_from_home" command from the Pilot
#define _FILTER_TILT_FROM_PILOT_

// Activate if you want to monitor the actual parameters of the AVR, instead of the sent parameters
//#define _MONITOR_ACTUAL_AVR_PARAMETERS_

// Activate if you want to use the new PID software in the AVR
//#define _UPO_PID_


#include <iostream>
#include <string>
#include <stdint.h>
#include <time.h>
#include <math.h>
#include "giraff_serial.h"

//-- HEADERS ----------------------------------------------------------------------------

/*********************************************************************************/
/**                                                                              */
/** class GiraffManagerException                                                 */
/**                                                                              */
/** An exception to throw in case of fatal error                                 */
/**                                                                              */
/*********************************************************************************/

class GiraffManagerException
{
	public:
	GiraffManagerException(const std::string& message) : message(message) {}
	~GiraffManagerException() throw () {}
	virtual const char* what() const throw() {return message.c_str();}
		
	private:
	std::string message;
};

/*********************************************************************************/
/**                                                                              */
/** class StopAction                                                             */
/**                                                                              */
/** This implements an action to stop the Giraff. It is used when the            */
/** Giraff PC activates its DTR flag to begin communication, we stop the Giraff  */
/** for security and send the welcome message                                    */
/**                                                                              */
/*********************************************************************************/

class StopGiraffAction : public Action
{
	public:
	StopGiraffAction(GiraffAVR& giraffAVR) : giraffAVR(giraffAVR) {}
	virtual ~StopGiraffAction() {}
	virtual bool doAction()
	{
		#ifdef _UPO_PID_
		return giraffAVR.stopPID();
		#else
		return giraffAVR.setInt32("mode",0) &&
					giraffAVR.setFloat("a",0) &&
					giraffAVR.setFloat("v",0) && 
					giraffAVR.setFloat("vg",0) &&
					giraffAVR.setFloat("p",0); 
		#endif
	}
	private:
	GiraffAVR& giraffAVR;
};


typedef struct
{
	bool giraffPC_control;
	bool is_stopped;
	int32_t mode;
	float a;
	float v;	
	float vg;
	float p;
} GiraffState;

/*********************************************************************************/
/**                                                                              */
/** class GiraffManager                                                          */
/**                                                                              */
/** This manages the communication Giraff PC <--> Linux PC <--> Giraff AVR       */
/** It connects with both sides by RS232. It is possible to set the velocity     */
/** of the Giraff by linear velocity and angular velocity. The commands of       */
/** the Giraff PC have priority, only after a timeout without receiving motion   */
/** commands, the manager begins to send motion commands based on the last set   */
/** of linear velocity and angular velocity.                                     */
/**                                                                              */
/*********************************************************************************/


class GiraffManager
{
	public:
	/*------------------------------------------------- Constructor -----
	|  Constructor
	|
	|  Parameters:
	|      giraffAVR_device (IN) -- The device name of the Giraff AVR, 
	|                               such as "/dev/ttyUSB0"
	|      giraffPC_device (IN) -- The device name of the Giraff PC,
	|                              such as "/dev/ttyUSB1"
	|      maxLinearVelocity (IN) -- Maximum linear velocity in m/s
	|      maxAngularVelocity (IN) -- Maximum angular velocity in rad/s
	|      timeout (IN) -- Timeout in seconds without receiving motion commands
	|                      from giraffPC to begin controlling the giraff by
	|                      linear velocity and angular velocity.
	|   
	|  Comments:
	|     Both connections will be stablished in the constructor, if some
	|     error happens, a GiraffManagerException will be throw.
	|  
	*-------------------------------------------------------------------*/
	GiraffManager(const std::string& giraffAVR_device, const std::string& giraffPC_device,
		 	float maxLinearVelocity,float maxAngularVelocity,int timeout, GiraffAVRMonitor& monitor = defaultMonitor);
	/*------------------------------------------------- Destructor -----
	|  Destructor
	|
	|  Comments: The connections will be closed
	*-------------------------------------------------------------------*/
	~GiraffManager();
	/*------------------------------------------------- update -----
	|  update
	|
	|  Purpose: 
	|	This function first attends the commands of the Giraff PC, if
	|       the received command is a motion command, the Giraff PC takes the
	|       control of the Giraff until a timeout without sending motion commands.
	|       If the Giraff PC doesn't have the control, then the manager send commands
	|       to move the Giraff following the last set of linear and angular velocity. 	
	|
	|   
	|  Comments:
	|	This function should be called in a loop
	|  
	*-------------------------------------------------------------------*/
	bool update(std::string& pilot_command, std::string& pilot_response);
	/*------------------------------------------------- setVelocity -----
	|  setVelocity
	|
	|  Purpose: 
	|	This function sets the linear and angular velocity to be used
	|	when the Giraff PC doesn't have the control.
	|
	|  Parameters:
	|	linear (IN) -- linear velocity in m/s
	|	angular (IN) -- angular velocity in rad/s
	|
	|  Return:
	|	The internal AVR state (mode,a,v,vg,p) for this command 
	|
	|  
	*-------------------------------------------------------------------*/
	GiraffState setVelocity(float linear, float angular);

	/*------------------------------------------------- getIMD -----
	|  getIMD
	|
	|  Purpose:
	|       This function gets the incremental move distance (IMDL,IMDR)
	|       directly from the AVR
	|
	|  Parameters:
	|	IMDL (OUT) -- Incremental move distance for left wheel in meters
	|	IMDR (OUT) -- Incremental move distance for right wheel in meters
	|  
	|  
	*-------------------------------------------------------------------*/
	void getIMD(float& imdl, float& imdr);

	/*------------------------------------------------- setStalk -----
	|  setStalk
	|
	|  Purpose:
	|       This function sets the stalk height position to the specified value
	|
	|  Parameters:
	|       height -- the desired height, in meters, of the robot head w.r.t. its
        |                 initiial position.
        |
	|  Returns:
	|
	*-------------------------------------------------------------------*/	
	void setStalk(float height);
	/*------------------------------------------------- incStalk -----
	|  incStalk
	|
	|  Purpose:
	|       This function increments the stalk height position 10 steps
	|
	|  Parameters:
	|
	|  Returns:
	|      The current stalk height position (0 - 1000)
	|  
	*-------------------------------------------------------------------*/	
	int incStalk();
	/*------------------------------------------------- decStalk -----
	|  decStalk
	|
	|  Purpose:
	|       This function decrements the stalk height position 10 steps
	|
	|  Parameters:
	|
	|  Returns:
	|      The current stalk height position (0 - 1000)
	|  
	*-------------------------------------------------------------------*/	
	int decStalk();
	/*------------------------------------------------- setTilt -----
	|  setTilt
	|
	|  Purpose:
	|       This function sets the head tilt to the specified value.
	|
	|  Parameters:
	|       tilt -- the desired tilt angle, in radians.
    	|
	|  Returns:
    	|
	*-------------------------------------------------------------------*/
	void setTilt(float tilt);
	/*------------------------------------------------- setTilt -----
	|  setTilt
	|
   	|  Purpose:
   	|       This function sets the head tilt to the specified value.
   	|
   	|  Parameters:
   	|
   	|  Returns:
   	|       The current tilt angle, in radians.
	|
	*-------------------------------------------------------------------*/	
	float getTilt();
	/*------------------------------------------------- incTilt -----
	|  incTilt
	|
	|  Purpose:
	|       This function increments the head tilt angle 0.02 radians
	|
	|  Parameters:
	|
	|  Returns:
	|      The current head tilt angle
	|  
	*-------------------------------------------------------------------*/	
	float incTilt();
	/*------------------------------------------------- decTilt -----
	|  decTilt
	|
	|  Purpose:
	|       This function decrements the head tilt angle 0.02 radians
	|
	|  Parameters:
	|
	|  Returns:
	|      The current head tilt angle
	|  
	*-------------------------------------------------------------------*/	
	float decTilt();
	/*------------------------------------------------- isStopped -----
	|  isStopped
	|
	|  Purpose:
	|       This function determines if the robot is stopped
	|
	|  Parameters:
	|
	|  Returns:
	|      True -> The robot is stopped
	|      False -> The robot is moving	
	|  
	*-------------------------------------------------------------------*/
	bool isStopped();
	/*------------------------------------------------- getStalk-----
	|  getStalk
	|
	|  Purpose:
	|       get the current h value from AVR
	|
	|  Parameters:
	|
	|  Returns:
	|     h value (0 - 1000)
	|      	
	|  
	*-------------------------------------------------------------------*/
	int getStalk();

	/*------------------------------------------------- getBattery-----
	|  getBattery
	|
	|  Purpose:
	|       get the battery level of the robot 
	|
	|  Parameters:
	|      batteryLevel (OUT): The battery level 
	|
	|  Returns:
	|     true -> battery level was obtained
	|     false -> battery level cannot be obtained yet 
	|	
	|
	|      	
	|  
	*-------------------------------------------------------------------*/
	bool getBattery(float& batteryLevel);
	
	private:

	GiraffAVR giraffAVR;
	GiraffPC  giraffPC;
	
	float maxLinearVelocity;
	float maxAngularVelocity;
	
	int timeout;

	GiraffState state;

	void updateState(int32_t mode, float a, float v, float vg, float p);

	int32_t calculateEncoderIncrement(int32_t previous, int32_t current);

};

//-- END OF HEADERS ----------------------------------------------------------------------------

//-- INLINE FUNCTIONS ----------------------------------------------------------------------------
// All the neccesary code is here :-)

/***********************************/
/** GiraffManager implementation   */
/***********************************/

inline GiraffManager::GiraffManager(const std::string& giraffAVR_device, 
					const std::string& giraffPC_device, 
					float maxLinearVelocity,
					float maxAngularVelocity,
					int timeout, 
					GiraffAVRMonitor& monitor) :
giraffAVR(giraffAVR_device,monitor),
giraffPC(giraffPC_device),
maxLinearVelocity(maxLinearVelocity),
maxAngularVelocity(maxAngularVelocity),
timeout(timeout)
{
	#ifdef _UPO_PID_
	state.giraffPC_control=false;
	#else
	state.giraffPC_control=true;
	#endif	
	state.is_stopped = true;
	state.mode = 0;
	state.a = 0;
	state.v = 0;
	state.vg = 0;
	state.p = 0;

	#ifdef _GIRAFF_MANAGER_DEBUG_
	_printTime();
	std::cout << "Connecting Giraff AVR"<<std::endl;
	#endif
	if (!giraffAVR.open()) {
		throw GiraffManagerException(giraffAVR.getLastError());
	}

	// Set wheel encoders to 0 and stop robot
	#ifdef _UPO_PID_
	if (!giraffAVR.setInt32("enc0",0) ||
		!giraffAVR.setInt32("enc1",0) ||
		!giraffAVR.stopPID()) {
		throw GiraffManagerException(giraffAVR.getLastError());	
	} 
	#else
	if (!giraffAVR.setInt32("enc0",0) ||
		!giraffAVR.setInt32("enc1",0) ||
		!giraffAVR.setInt32("mode",0) ||
		!giraffAVR.setFloat("a",0) ||
		!giraffAVR.setFloat("v",0) ||
		!giraffAVR.setFloat("vg",0) ||
		!giraffAVR.setFloat("vgr",0) ||
		!giraffAVR.setFloat("p",0)) {
		throw GiraffManagerException(giraffAVR.getLastError());	
	}
	#endif 


	#ifdef _GIRAFF_MANAGER_DEBUG_
	_printTime();
	std::cout << "Connecting Giraff PC"<<std::endl;
	#endif
	if (!giraffPC.open()) {
		throw GiraffManagerException(giraffPC.getLastError());
	}

	
}

inline GiraffManager::~GiraffManager() {
	if (giraffAVR.isOpen()) {
		#ifdef _UPO_PID_
		giraffAVR.stopPID();
		#else
		giraffAVR.setInt32("mode",0);
		giraffAVR.setFloat("a",0);
		giraffAVR.setFloat("v",0);
		giraffAVR.setFloat("vg",0);
		giraffAVR.setFloat("vgr",0);
		giraffAVR.setFloat("p",0);
		#endif
	}
}


inline int32_t GiraffManager::calculateEncoderIncrement(int32_t previous, int32_t current)
{
	// Encoder range: -32768 to 32767

	int32_t a = previous + 32768; // From 0 to 65535
	int32_t b = current  + 32768; // From 0 to 65535

	int32_t option1 = b-a; 
	int32_t option2 = (option1>0)?(-65536 + option1):(65536 + option1);

	return abs(option1)<abs(option2) ? option1:option2;
}


inline void GiraffManager::getIMD(float& imdl, float& imdr)
{
	static int32_t prev_enc0=0,prev_enc1=0;
	int32_t enc0,enc1,inc_enc0,inc_enc1;

	if (!giraffAVR.writeCommand("get enc0\r",enc0) ||
		!giraffAVR.writeCommand("get enc1\r",enc1)) {
		throw GiraffManagerException(giraffAVR.getLastError());
	}
	
	inc_enc0 = calculateEncoderIncrement(prev_enc0,enc0);
	inc_enc1 = calculateEncoderIncrement(prev_enc1,enc1);
	prev_enc0 = enc0;
	prev_enc1 = enc1;
	
	if (inc_enc0==0 && inc_enc1==0) {
		state.is_stopped=true;
		imdl = 0.0;
		imdr = 0.0;
	}
	else{
		state.is_stopped=false;
		imdl = (float)inc_enc0/2002.0594;
		imdr = (float)inc_enc1/2002.0594;
	}
 
}

inline void GiraffManager::setStalk(float height)
{
        height = std::max(height, (float) MIN_STALK_HEIGHT);
        height = std::min(height, (float) MAX_STALK_HEIGHT);
        
        int32_t h_cmd = round((height - MIN_STALK_HEIGHT)/(MAX_STALK_HEIGHT-MIN_STALK_HEIGHT)*1000);

	if (!giraffAVR.setInt32("h", h_cmd)) {
		throw GiraffManagerException(giraffAVR.getLastError());
      	}
}


inline int GiraffManager::incStalk()
{
	int32_t stalk;
	if (!giraffAVR.writeCommand("get h\r",stalk)) {
		throw GiraffManagerException(giraffAVR.getLastError());
	}
	
	int32_t oldStalk=stalk;
	stalk+=5;
	
	if (stalk>1000) {
		stalk=1000;
	}
	if (stalk!=oldStalk) {
		if (!giraffAVR.setInt32("h",stalk)) {
			throw GiraffManagerException(giraffAVR.getLastError());
		}
	}
	return stalk;
}

inline int GiraffManager::getStalk()
{
	int32_t stalk;
	if (!giraffAVR.writeCommand("get h\r",stalk)) {
		throw GiraffManagerException(giraffAVR.getLastError());
	}
	return stalk;

}

inline bool GiraffManager::getBattery(float &batteryLevel)
{
	const static float c = -327360.0f;
	std::string response;
	if (!giraffAVR.writeCommand("get charger_data\r",response)) {
		throw GiraffManagerException(giraffAVR.getLastError());
	}
	
	size_t p1 = response.find_first_of(',');
	if (p1	== std::string::npos)  {
		return false;
	}
	p1++;
	if (p1 == response.size()) {
		return false;
	}
	if (response[p1]!='W') {
		return false;
	}
	p1+=2;	

	size_t p2 = response.find_first_of(' ',p1);
	if (p2	== std::string::npos)  {
		return false;
	}
	float ws;
	if (!GiraffAVR::toFloat(response.substr(p1,p2-p1),ws)) {
		return false;
	}
	
	batteryLevel = std::abs((c-ws)/c); 
	
	return true;

}

inline bool GiraffManager::isStopped()
{
	return state.is_stopped;
}

inline int GiraffManager::decStalk()
{
	int32_t stalk;
	if (!giraffAVR.writeCommand("get h\r",stalk)) {
		throw GiraffManagerException(giraffAVR.getLastError());
	}
	int32_t oldStalk = stalk;
	stalk-=5;
	if (stalk<0) {
		stalk=0;
	}
	if (stalk!=oldStalk) {
		if (!giraffAVR.setInt32("h",stalk)) {
			throw GiraffManagerException(giraffAVR.getLastError());
		}
	}
	
	return stalk;
}

inline void GiraffManager::setTilt(float tilt)
{	
        if (fabs(tilt) > 2*M_PI/3.0) {
          tilt = (1-2*std::signbit(tilt))*2*M_PI/3.0;
        }
	
	if (!giraffAVR.setFloat("tilt_angle_from_home",tilt)) {
		throw GiraffManagerException(giraffAVR.getLastError());
	}
}

inline float GiraffManager::getTilt()
{
	float tilt;
	if (!giraffAVR.writeCommand("get tilt_angle_from_home\r",tilt)) {
		throw GiraffManagerException(giraffAVR.getLastError());
	}
	return tilt;
}


inline float GiraffManager::incTilt()
{
	float tilt;
	if (!giraffAVR.writeCommand("get tilt_angle_from_home\r",tilt)) {
		throw GiraffManagerException(giraffAVR.getLastError());
	}
	
	float oldTilt=tilt;
	tilt = tilt + 0.02;
	
	if (tilt>2.0943935) {
		tilt = 2.0943935;
	}
	if (tilt!=oldTilt) {
		if (!giraffAVR.setFloat("tilt_angle_from_home",tilt)) {
			throw GiraffManagerException(giraffAVR.getLastError());
		}
	}
	return tilt;
}

inline float GiraffManager::decTilt()
{
	float tilt;
	if (!giraffAVR.writeCommand("get tilt_angle_from_home\r",tilt)) {
		throw GiraffManagerException(giraffAVR.getLastError());
	}
	
	float oldTilt=tilt;
	tilt = tilt - 0.02;
	
	if (tilt<0.0872664) {
		tilt = 0.0872664;
	}
	if (tilt!=oldTilt) {
		if (!giraffAVR.setFloat("tilt_angle_from_home",tilt)) {
			throw GiraffManagerException(giraffAVR.getLastError());
		}
	}
	return tilt;
}


inline void GiraffManager::updateState(int32_t mode, float a, float v, float vg, float p)
{
	#ifndef _UPO_PID_
	#ifdef _MONITOR_ACTUAL_AVR_PARAMETERS_
	giraffAVR.writeCommand("get mode\r",mode);
	giraffAVR.writeCommand("get a\r",a);
	giraffAVR.writeCommand("get v\r",v);
	giraffAVR.writeCommand("get vg\r",vg);
	giraffAVR.writeCommand("get p\r",p);
	#endif
	#endif
	
	state.mode = mode;
	state.a = a;	
	state.v = v;
	state.vg = vg;
	state.p = p;
}
#ifdef _UPO_PID_
inline bool GiraffManager::update(std::string& pilot_command, std::string& pilot_response)
{
	std::string command;
	std::string response;	
	static StopGiraffAction stopGiraffAction(giraffAVR);
	bool pilot_communication = false;

	#ifdef _OLD_USB_CONNECTIONS_
	if (!giraffPC.checkDSR(giraffAVR.getWelcomeMessage(),stopGiraffAction)) {
	#else
	if (!giraffPC.checkRTS(giraffAVR.getWelcomeMessage(),stopGiraffAction)) {
	#endif
		throw GiraffManagerException(giraffPC.getLastError());
	}

	if (!giraffPC.readCommand(command)) {
		throw GiraffManagerException(giraffPC.getLastError());
	}
	
	if (command.size()>0) {
		
		bool takeControl = command.find("get ")!=0 && 
		                   command.compare("get\r")!=0 &&
		                   command.compare("home\r")!=0 &&
	                       command.find("set tilt_angle_from_home ")!=0 && 
	                       command.find("set h ")!=0;
		if (takeControl) {
			response.assign("OK >\r\n");
		} else               		
		#ifdef _FILTER_TILT_FROM_PILOT_
		if (command.find("set tilt_angle_from_home ")==0) {
			response.assign("F*00000000\r\nOK >\r\n");
		} else		
		#endif
	  	if (!giraffAVR.writeCommand(command,response)) {
			throw GiraffManagerException(giraffAVR.getLastError());
		}
		
		if (!giraffPC.writeResponse(response)) {
			throw GiraffManagerException(giraffPC.getLastError());
		}
	
		pilot_command = command;
		pilot_response = response;		
		pilot_communication = true;
	
	}
	
	return pilot_communication;

}

#else

inline bool GiraffManager::update(std::string& pilot_command, std::string& pilot_response)
{
	static float a_bk=0,v_bk=0,vg_bk=0;
	static int32_t mode_bk=0;
	static	time_t giraffPC_time = time(NULL);
	static StopGiraffAction stopGiraffAction(giraffAVR);
			
	std::string command;
	std::string response;
	bool pilot_communication = false;
	bool giraffPC_initiated0 = giraffPC.isInitiated();
	
	#ifdef _OLD_USB_CONNECTIONS_
	if (!giraffPC.checkDSR(giraffAVR.getWelcomeMessage(),stopGiraffAction)) {
	#else
	if (!giraffPC.checkRTS(giraffAVR.getWelcomeMessage(),stopGiraffAction)) {
	#endif
		throw GiraffManagerException(giraffPC.getLastError());
	}
	
	bool giraffPC_initiated1 = giraffPC.isInitiated();
	
	if (!giraffPC_initiated0 && giraffPC_initiated1) {
		state.giraffPC_control = true;
		giraffPC_time = time(NULL);
	}
	else if (state.giraffPC_control && giraffPC_initiated0 && !giraffPC_initiated1) {
		if (!giraffAVR.setInt32("mode",2) ||
			!giraffAVR.setFloat("a",ACELERATION) ||
			!giraffAVR.setFloat("v",0) ||
			!giraffAVR.setFloat("vg",0) ||
			!giraffAVR.setFloat("p",0)) {
			throw GiraffManagerException(giraffAVR.getLastError());	
		}
		updateState(2,ACELERATION,0,0,0);
		state.giraffPC_control = false;
	}
	
	if (!giraffPC.readCommand(command)) {
		throw GiraffManagerException(giraffPC.getLastError());
	}
	
	
	if (command.size()>0) {
		
		bool takeControl = command.find("get ")!=0 && 
		                   command.compare("get\r")!=0 &&
		                   command.compare("home\r")!=0 &&
	                       command.find("set tilt_angle_from_home ")!=0 && 
	                       command.find("set h ")!=0;
	                       
	    if (!state.giraffPC_control && takeControl) {
			if (!giraffAVR.setInt32("mode",mode_bk) ||
				!giraffAVR.setFloat("a",a_bk) ||
				!giraffAVR.setFloat("v",v_bk) ||
				!giraffAVR.setFloat("vg",vg_bk) ||
				!giraffAVR.setFloat("p",0)) {
				throw GiraffManagerException(giraffAVR.getLastError());	
			}
			updateState(mode_bk,a_bk,v_bk,vg_bk,0);
		}
		
		#ifdef _FILTER_TILT_FROM_PILOT_
		if (command.find("set tilt_angle_from_home ")==0) {
			response.assign("F*00000000\r\nOK >\r\n");
		} else		
		#endif
	  	if (!giraffAVR.writeCommand(command,response)) {
			throw GiraffManagerException(giraffAVR.getLastError());
		}
		
		if (!giraffPC.writeResponse(response)) {
			throw GiraffManagerException(giraffPC.getLastError());
		}

		
		pilot_command = command;
		pilot_response = response;		
		pilot_communication = true;

		if (takeControl) {
			state.giraffPC_control = true;
			giraffPC_time = time(NULL);
		}
	}
	
	
	if (state.giraffPC_control && difftime(time(NULL),giraffPC_time)>timeout) {
		if (!giraffAVR.writeCommand("get mode\r",mode_bk) ||
			!giraffAVR.writeCommand("get a\r",a_bk) ||
			!giraffAVR.writeCommand("get v\r",v_bk) ||
			!giraffAVR.writeCommand("get vg\r",vg_bk)) {
			throw GiraffManagerException(giraffAVR.getLastError());
		}
		if (!giraffAVR.setInt32("mode",2) ||
			!giraffAVR.setFloat("a",ACELERATION) ||
			!giraffAVR.setFloat("v",0) ||
			!giraffAVR.setFloat("vg",0) ||
			!giraffAVR.setFloat("p",0)) {
			throw GiraffManagerException(giraffAVR.getLastError());	
		}
		updateState(2,ACELERATION,0,0,0);
		state.giraffPC_control=false;
	}
	
	
	return pilot_communication;
}

#endif


inline GiraffState GiraffManager::setVelocity(float linearVelocity, float angularVelocity)
{

	#ifndef _UPO_PID_
	if (state.giraffPC_control) {
		return state;
	}
	#endif

	if (linearVelocity<0.001 && linearVelocity>-0.001) {
		linearVelocity = 0.0;
	}	

	if (angularVelocity<0.001 && angularVelocity>-0.001) {
		angularVelocity = 0.0;
	}

	if (linearVelocity>maxLinearVelocity) {
		linearVelocity = maxLinearVelocity;	
	}
	else if (linearVelocity< -maxLinearVelocity) {
		linearVelocity = -maxLinearVelocity;
	}

	if (angularVelocity>maxAngularVelocity) {
		angularVelocity = maxAngularVelocity;	
	}
	else if (angularVelocity< -maxAngularVelocity) {
		angularVelocity = -maxAngularVelocity;
	}


	if (linearVelocity>-0.001 && linearVelocity<0.001 && angularVelocity>-0.001 && angularVelocity<0.001) {
		#ifdef _UPO_PID_
		if (!giraffAVR.setPIDVelocities(0,0)) {
			throw GiraffManagerException(giraffAVR.getLastError());	
		}
		#else
		if (!giraffAVR.setFloat("v",0) ||
			!giraffAVR.setFloat("vg",0) ||
			!giraffAVR.setFloat("p",0)) {
			throw GiraffManagerException(giraffAVR.getLastError());	
		}
		#endif
		updateState(state.mode,state.a,0,0,0);
	}
	else {
		#ifdef _UPO_PID_
		if (!giraffAVR.setPIDVelocities(linearVelocity,angularVelocity)) {
			throw GiraffManagerException(giraffAVR.getLastError());	
		}
		updateState(state.mode,state.a,linearVelocity,angularVelocity,0);
		#else
		float v,vg,p;
		if (linearVelocity>=0.001) {
			v = linearVelocity;
			p = 1;
		}
		else if (linearVelocity<=-0.001) {
			v = -linearVelocity;
			p = -1;
		}
		else {
			v = 0;
			p = 1;
		}
		vg = angularVelocity*180/PI;
		if (!giraffAVR.setFloat("v",v) ||
			!giraffAVR.setFloat("vg",vg) ||
			!giraffAVR.setFloat("p",p)) {
			throw GiraffManagerException(giraffAVR.getLastError());	
		}
		updateState(state.mode,state.a,v,vg,p);		
		#endif
		

	}
	return state;
	
}

//-- END OF INLINE FUNCTIONS ---------------------------------------

#endif
