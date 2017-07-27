#include "ros/ros.h"
#include "ugv/motorvels.h"
#include <stdlib.h>

int main(int argc, char** argv) {
	ros::init(argc, argv, "randomcontrol");
	ros::NodeHandle n;
	ros::Publisher pub = n.advertise<ugv::motorvels>("motors",1000);
	ros::Rate loop_rate(1);

	while(ros::ok()) {
		ugv::motorvels msg;
		
		msg.left = rand()%2001 - 1000;
		msg.right = rand()%2001 - 1000;
		
		pub.publish(msg);
		ros::spinOnce();
		loop_rate.sleep();
	}
	return 0;
}

