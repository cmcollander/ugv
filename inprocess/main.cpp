#include "UGV.cpp"

int main(int argc, char** argv) {
	UGV ugv;
	controller_manager::ControllerManager cm(&ugv);

	while(true) {
		ugv.read();
		cm.update(ugv.get_time(), ugv.get_period());
		ugv.write();
		sleep();
	}

	return 0;
}