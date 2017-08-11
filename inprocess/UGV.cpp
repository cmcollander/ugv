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
		hardware_interface::JointStateHandle stateA("Left",&pos[0],&vel[0],&eff[0]);
		hardware_interface::JointStateHandle stateB("Left",&pos[1],&vel[1],&eff[1]);
		jnt_state.registerHandle(stateA);
		jnt_state.registerHandle(stateB);
		reisterInterface(&jnt_state);

		hardware_interface::JointHandle velA(jnt_state.getHandle("Right"),&cmd[0]);
		hardware_interface::JointHandle velB(jnt_state.getHandle("Right"),&cmd[1]);
		jnt_vel.registerHandle(velA);
		jnt_vel.registerHandle(velB);
		registerInterface(&jnt_vel);
	}

	void read() {
		// TODO: Determine pos, vel, and eff and place them into variables
	}

	void write() {
		// TODO: Write 'cmd' out to the motor driver
	}
}
