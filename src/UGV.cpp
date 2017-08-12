#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>

class UGV : public hardware_interface::RobotHW {
private:
	hardware_interface::JointStateInterface jnt_state;
	hardware_interface::VelocityJointInterface jnt_vel;
	double cmd[2];
	double pos[2];
	double vel[2];
	double eff[2];
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
	}

	// Functions to assist with ROS time-management
	ros::Time getTime() const {return ros::Time::now();}
    ros::Duration getPeriod() const {return ros::Duration(0.01);}

	void read() {
		// TODO: Determine pos, vel, and eff and place them into variables

	}

	void write() {
		// TODO: Write 'cmd' out to the motor driver
		pos[0] += vel[0]*getPeriod().toSec();
		pos[1] += vel[1]*getPeriod().toSec();
		vel[0] = cmd[0];
		vel[1] = cmd[1];
	}
};
