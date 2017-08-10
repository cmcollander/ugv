#include "ros/ros.h" // For ROS control
#include "ugv/motorvels.h" // The description of the ugv/motorvels message type
#include <stdlib.h>

int main(int argc, char** argv) {
	ros::init(argc, argv, "randomcontrol"); // Initialize our application 'randomcontrol' (same as file name)
	ros::NodeHandle n; // Create a new ROS node for this application
	ros::Publisher pub = n.advertise<ugv::motorvels>("motors",1000);  // Create a new publisher to the /motors topic, sending ugv/motorvels messages. 1000 messages can be queued at a time
	ros::Rate loop_rate(1); // How many Hz does this signal publish at? for this, 1 cycle per second.

	while(ros::ok()) { // Loop as long as ROS is running, or broken out of otherwise
		ugv::motorvels msg; // Blank ugv/motorvels message
		
		msg.left = rand()%500 - 250; // Set the left motor value between -1000 and 1000
		msg.right = rand()%500 - 250; // Set the right motor value between -1000 and 1000
		
		pub.publish(msg); // Publish our message
		ros::spinOnce(); // Handle ROS
		loop_rate.sleep(); // Pause to allow the above loop_rate
	}
	return 0;
}

