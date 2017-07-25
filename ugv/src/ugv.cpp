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
  device.SetCommand(_GO, 1, (int)msg.left);
  sleepms(10); // Allow proper time between commands
  device.SetCommand(_GO, 2, (int)msg.right);
}

int main(int argc, char **argv)
{
  // Prepare our motor controller
  int status = device.Connect("/dev/ttyACM0");
  if(status != RQ_SUCCESS)
  {
    cout<<"Error connecting to device: "<<status<<"."<<endl;
    return 1;
  }


  ros::init(argc, argv, "ugv");

  ros::NodeHandle n;

  ros::Subscriber sub = n.subscribe("motors", 1000, motorCallback);

  ros::spin();

  device.Disconnect();
  return 0;
}
