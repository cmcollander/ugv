/*
Chris Collander, cmcollander@gmail.com, December 2018

This class extends hardware_interface:Robot_HW handle communication between  to a Roboteq motor controller. 
This code should be called by UGVinterface.cpp
*/

#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>

#include "ugv/RoboteqDevice.h"
#include "ugv/ErrorCodes.h"
#include "ugv/Constants.h"
#include <unistd.h>

class UGV : public hardware_interface::RobotHW {
private:
	RoboteqDevice device;
	hardware_interface::JointStateInterface jnt_state;
	hardware_interface::VelocityJointInterface jnt_vel;
	
	double cmd[2];
	double pos[2];
	double vel[2];
	double eff[2];
	
	double vel_in_value;
	double vel_out_value;
	double odom_value;
		
	int ret, initial_encoder[2];
public:
	UGV(std::string mcusb, double vel_in_scale, double vel_out_scale, double odom_scale) {
		// Pull in arguments
		vel_in_value = vel_in_scale;
		vel_out_value = vel_out_scale;
		odom_value = odom_scale;
		
		// Initialize all values to zero
		cmd[0] = 0; cmd[1] = 0;
		pos[0] = 0; pos[1] = 0;
		vel[0] = 0; vel[1] = 0;
		eff[0] = 0; eff[1] = 0;

		// Create our joints and register them
		// TODO: Why did I set these to these indices???? They work in my case though...
		hardware_interface::JointStateHandle stateA("Right",&pos[1],&vel[0],&eff[0]);
		hardware_interface::JointStateHandle stateB("Left",&pos[0],&vel[1],&eff[1]);
		jnt_state.registerHandle(stateA);
		jnt_state.registerHandle(stateB);
		registerInterface(&jnt_state);
		hardware_interface::JointHandle velA(jnt_state.getHandle("Right"),&cmd[1]);
		hardware_interface::JointHandle velB(jnt_state.getHandle("Left"),&cmd[0]);
		jnt_vel.registerHandle(velA);
		jnt_vel.registerHandle(velB);
		registerInterface(&jnt_vel);
		
		// Prepare our motor controller
		int status = device.Connect(mcusb);
		device.GetValue(_ABCNTR,1,initial_encoder[0]);
		device.GetValue(_ABCNTR,2,initial_encoder[1]);

	}

	// Functions to assist with ROS time-management
	ros::Time getTime() const {return ros::Time::now();}
        ros::Duration getPeriod() const {return ros::Duration(0.05);}

	void read() {
		// Determine pos, vel, and eff and place them into variables
		
		// _ABSPEED returns RPM, we need Rads/Sec
		vel[0] = device.GetValue(_ABSPEED,1,ret);
		vel[0] = ret*vel_in_value;
		vel[1] = device.GetValue(_ABSPEED,2,ret);
		vel[1] = ret*vel_in_value;
		
		pos[0] = device.GetValue(_ABCNTR,1,ret);
		pos[0] = ret-initial_encoder[1]*odom_value;
		pos[1] = device.GetValue(_ABCNTR,2,ret);
		pos[1] = ret-initial_encoder[1]*odom_value;
		
		eff[0] = device.GetValue(_MOTPWR,1,ret);
		eff[0] = ret;
		eff[1] = device.GetValue(_MOTPWR,2,ret);
		eff[1] = ret;
	}

	void write() {
		// Write 'cmd' out to the motor driver
		// cmd[] is in rads per second
		// _GO takes in a value from -1000 to 1000 linearly from -60RPM to 60RPM
		device.SetCommand(_GO,1,(int)(cmd[0]*vel_out_value));
		device.SetCommand(_GO,2,(int)(cmd[1]*vel_out_value));
	}
};
