
#include "ros/ros.h"
#include "ugv/motorvels.h"
#include <sensor_msgs/Joy.h>
#include <stdlib.h>

ros::Publisher pub;
ros::Subscriber sub;

int speed;
int count;

long map(float x, float in_min, float in_max, float out_min, float out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void joystickCallback(sensor_msgs::Joy in) {
	// Only repeat every 7th message
	count++;
	if(count>=7) {
		count=0;
		return;
	}

	// Create our new message (initially empty)
	ugv::motorvels msg;
	// Translate Joy variables to msg variables
	std::vector<float> axes2 = std::vector<float>(in.axes);
	float x = axes2[3];
	float y = axes2[4];
	int     nJoyX = map(x,-1,1,128,-127);
	int     nJoyY = map(y,-1,1,-128,127);
	int     nMotMixL;
	int     nMotMixR;
	float fPivYLimit = 32.0;
	float   nMotPremixL;    // Motor (left)  premixed output        (-128..+127)
	float   nMotPremixR;    // Motor (right) premixed output        (-128..+127)
	int     nPivSpeed;      // Pivot Speed                          (-128..+127)
	float   fPivScale;      // Balance scale b/w drive and pivot    (   0..1   )
	if (nJoyY >= 0) {
	  // Forward
	  nMotPremixL = (nJoyX>=0)? 127.0 : (127.0 + nJoyX);
	  nMotPremixR = (nJoyX>=0)? (127.0 - nJoyX) : 127.0;
	} else {
	  // Reverse
	  nMotPremixL = (nJoyX>=0)? (127.0 - nJoyX) : 127.0;
	  nMotPremixR = (nJoyX>=0)? 127.0 : (127.0 + nJoyX);
	}
	// Scale Drive output due to Joystick Y input (throttle)
	nMotPremixL = nMotPremixL * nJoyY/128.0;
	nMotPremixR = nMotPremixR * nJoyY/128.0;

	nPivSpeed = nJoyX;
	fPivScale = (abs(nJoyY)>fPivYLimit)? 0.0 : (1.0 - abs(nJoyY)/fPivYLimit);

	nMotMixL = (1.0-fPivScale)*nMotPremixL + fPivScale*( nPivSpeed);
	nMotMixR = (1.0-fPivScale)*nMotPremixR + fPivScale*(-nPivSpeed);
	msg.left = map(nMotMixL,-128,127,-1*speed,speed);
	msg.right = map(nMotMixR,-128,127,-1*speed,speed);
	pub.publish(msg);
}

int main(int argc, char** argv) {
	ros::init(argc,argv,"joystickcontrol");
	ros::NodeHandle n;
	count = 0;	

	// Determine our max speed from the "ugvmaxspeed" rosparam, default value of 250
	n.param<int>("ugvmaxspeed",speed,250);

	pub = n.advertise<ugv::motorvels>("motors",1000);
	sub = n.subscribe("joy",1000,joystickCallback);

	ros::spin();
	return 0;
}



