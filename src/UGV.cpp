/*
Chris Collander, cmcollander@gmail.com, December 2018

This ROS node is used for communication between /cmd_vel and /odom to a Roboteq motor controller.

Please ensure your controller has the following settings:
* Left wheel connected to motor 1
* Right wheel connected to motor 2
* Encoders connected, tested, and set to tuned PID coefficients in closed-loop speed control
* Encoder ticks per revolution properly calculated through gear ratio and encoder specs
* Max velocity for each wheel set to 60 RPM
* Some parameters may need to be tuned and changed based on your build. Trial and error.

After wheels respond to correct direction, speed can be adjusted through proper calculation (or trial and error)
After wheels respond to correct speed, odom can be adjusted through proper calculation (or trial and error)

My current method for tuning odom:
* Visualize in Rviz with /odom as global frame.
* View lidar and transform tree
* Rotate robot clockwise.
* * If lidar scan rotates to the right: Decrease ODOM_NUM and/or Increase ODOM_DEN
* * If lidar scan rotates to the left:  Increase ODOM_NUM and/or Decrease ODOM_DEN
* When robot is able to rotate N-rotations without rotation of the laser scan, odom is properly tuned (my N=100)
*/

#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>

#include "ugv/RoboteqDevice.h"
#include "ugv/ErrorCodes.h"
#include "ugv/Constants.h"
#include <unistd.h>

#define PI 3.14159265359

// These two values represent the numerator and denominator of the scalar values for Velocity input
#define VEL_IN_NUM PI
#define VEL_IN_DEN 3.0

// These two values represent the numerator and denominator of the scalar values for Velocity output
#define VEL_OUT_NUM 15.9155*0.33333333
#define VEL_OUT_DEN 1.0

// These two values represent the numerator and denominator of the scalar value from Odom.
#define ODOM_NUM PI
#define ODOM_DEN 6000*4.20

class UGV : public hardware_interface::RobotHW {
private:
	RoboteqDevice device;
	hardware_interface::JointStateInterface jnt_state;
	hardware_interface::VelocityJointInterface jnt_vel;
	double cmd[2];
	double pos[2];
	double vel[2];
	double eff[2];
	int ret, initial_encoder[2];
public:
	UGV() {
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
		int status = device.Connect("/dev/ttyACM0");
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
		vel[0] = ret*VEL_IN_NUM/VEL_IN_DEN;
		vel[1] = device.GetValue(_ABSPEED,2,ret);
		vel[1] = ret*VEL_IN_NUM/VEL_IN_DEN;
		
		pos[0] = device.GetValue(_ABCNTR,1,ret);
		pos[0] = ret-initial_encoder[1]*ODOM_NUM/ODOM_DEN;
		pos[1] = device.GetValue(_ABCNTR,2,ret);
		pos[1] = ret-initial_encoder[1]*ODOM_NUM/ODOM_DEN;
		
		eff[0] = device.GetValue(_MOTPWR,1,ret);
		eff[0] = ret;
		eff[1] = device.GetValue(_MOTPWR,2,ret);
		eff[1] = ret;
	}

	void write() {
		// Write 'cmd' out to the motor driver
		// cmd[] is in rads per second
		// _GO takes in a value from -1000 to 1000 linearly from -60RPM to 60RPM
		device.SetCommand(_GO,1,(int)(cmd[0]*VEL_OUT_NUM/VEL_OUT_DEN));
		device.SetCommand(_GO,2,(int)(cmd[1]*VEL_OUT_NUM/VEL_OUT_DEN));
	}
};
