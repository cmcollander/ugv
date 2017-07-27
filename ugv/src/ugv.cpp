#include "ros/ros.h"
#include "ugv/motorvels.h"
#include "ugv/RoboteqDevice.h"
#include "ugv/ErrorCodes.h"
#include "ugv/Constants.h"

#include <stdio.h>
#include <string.h>


RoboteqDevice device;

void motorCallback(const ugv::motorvels& msg)
{
  device.SetCommand(_GO, 2, (int)msg.left);
  sleepms(10); // Allow proper time between commands
  device.SetCommand(_GO, 1, (int)msg.right);
  sleepms(10); // Allow proper time between commands
}

int main(int argc, char **argv)
{
  // Prepare our motor controller (Loop until connection made)
  int status = device.Connect("/dev/ttyACM0");
  while(status != RQ_SUCCESS)
  {
    cout<<"Couldn't connect, reconnecting"<<endl;
    status = device.Connect("/dev/ttyACM0");
  }


  ros::init(argc, argv, "ugv");

  ros::NodeHandle n;

  ros::Subscriber sub = n.subscribe("motors", 1000, motorCallback);

  ros::spin();

  device.Disconnect();
  return 0;
}
