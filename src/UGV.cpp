#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>

#include "ugv/motorvels.h"
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
	int ret;
public:
	UGV() {
		// Initialize all values to zero
		cmd[0] = 0; cmd[1] = 0;
		pos[0] = 0; pos[1] = 0;
		vel[0] = 0; vel[1] = 0;
		eff[0] = 0; eff[1] = 0;

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

		// Prepare our motor controller
		int status = device.Connect("/dev/ttyACM0");
	}

	// Functions to assist with ROS time-management
	ros::Time getTime() const {return ros::Time::now();}
        ros::Duration getPeriod() const {return ros::Duration(0.05);}

	void read() {
		// Determine pos, vel, and eff and place them into variables
		// _ABSPEED returns RPM, we need Rads/Sec
		vel[0] = device.GetValue(_ABSPEED,1,ret)/9.5493;
<<<<<<< HEAD
		//usleep(10000);
		vel[1] = device.GetValue(_ABSPEED,2,ret)/9.5493;
		//usleep(10000);
		pos[0] = 3.141592653589793 * device.GetValue(_ABCNTR,1,ret)/6000;
		//usleep(10000);
		pos[1] = 3.141592653589793 * device.GetValue(_ABCNTR,2,ret)/6000;
		//usleep(10000);
		eff[0] = device.GetValue(_MOTPWR,1,ret);
		//usleep(10000);
		eff[1] = device.GetValue(_MOTPWR,2,ret);
		//usleep(10000);
=======
		vel[1] = device.GetValue(_ABSPEED,2,ret)/9.5493;
		pos[0] = 3.141592653589793 * device.GetValue(_ABCNTR,1,ret)/6000;
		pos[1] = 3.141592653589793 * device.GetValue(_ABCNTR,2,ret)/6000;
		eff[0] = device.GetValue(_MOTPWR,1,ret);
		eff[1] = device.GetValue(_MOTPWR,2,ret);
>>>>>>> 3bb681ba4474986eac0f869ea9e784d0d8291be4
	}

	void write() {
		// Write 'cmd' out to the motor driver
		// cmd[] is in rads per second
		// _GO takes in a value from -1000 to 1000 linearly from -60RPM to 60RPM
		device.SetCommand(_GO,1,(int)(cmd[0]*159.155));
<<<<<<< HEAD
		//usleep(10000);
		device.SetCommand(_GO,2,(int)(cmd[1]*159.155));
		//usleep(10000);
=======
		device.SetCommand(_GO,2,(int)(cmd[1]*159.155));
>>>>>>> 3bb681ba4474986eac0f869ea9e784d0d8291be4
	}
};
