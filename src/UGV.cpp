#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>

#include "ugv/motorvels.h"
#include "ugv/RoboteqDevice.h"
#include "ugv/ErrorCodes.h"
#include "ugv/Constants.h"

class UGV : public hardware_interface::RobotHW {
private:
	RoboteqDevice device;
	hardware_interface::JointStateInterface jnt_state;
	hardware_interface::VelocityJointInterface jnt_vel;
	double cmd[2];
	double pos[2];
	double vel[2];
	double eff[2];
	int* ret;
public:
	UGV() {
		// Initialize all values to zero
		cmd[0] = 0; cmd[1] = 0;
		pos[0] = 0; pos[1] = 0;
		vel[0] = 0; vel[0] = 0;
		eff[0] = 0; eff[0] = 0;

		// Create our joints and register them
		hardware_interface::JointStateHandle stateA("Left",&pos[0],&vel[0],&eff[0]);
		hardware_interface::JointStateHandle stateB("Right",&pos[1],&vel[1],&eff[1]);
		jnt_state.registerHandle(stateA);
		jnt_state.registerHandle(stateB);
		registerInterface(&jnt_state);
		hardware_interface::JointHandle velA(jnt_state.getHandle("Left"),&cmd[0]);
		hardware_interface::JointHandle velB(jnt_state.getHandle("Right"),&cmd[1]);
		jnt_vel.registerHandle(velA);
		jnt_vel.registerHandle(velB);
		registerInterface(&jnt_vel);

		// Prepare our motor controller (Loop until connection made)
		int status = device.Connect("/dev/ttyACM0");
		while(status != RQ_SUCCESS) {
			status = device.Connect("/dev/ttyACM0");
		}
	}

	// Functions to assist with ROS time-management
	ros::Time getTime() const {return ros::Time::now();}
        ros::Duration getPeriod() const {return ros::Duration(0.01);}

	void read() {
		// Determine pos, vel, and eff and place them into variables
		// _ABSPEED returns RPM, we need Rads/Sec
		vel[0] = device.GetValue(_ABSPEED,1,ret)/9.5493;
		vel[1] = device.GetValue(_ABSPEED,2,ret)/9.5493;
		pos[0] = 0; // I don't know if we need this...
		pos[1] = 0; // I don't know if we need this...
		eff[0] = device.GetValue(_MOTPWR,1,ret);
		eff[1] = device.GetValue(_MOTPWR,2,ret);
	}

	void write() {
		// Write 'cmd' out to the motor driver
		// cmd[] is in rads per second
		// _MOTVEL takes in RPM
		device.SetCommand(_MOTVEL,1,(int)(cmd[0]*9.5493));
		sleepms(10);
		device.SetCommand(_MOTVEL,2,(int)(cmd[1]*9.5493));
		sleepms(10);
	}
};
