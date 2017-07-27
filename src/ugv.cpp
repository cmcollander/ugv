#include "ros/ros.h"
#include "ugv/motorvels.h"
#include "ugv/RoboteqDevice.h"
#include "ugv/ErrorCodes.h"
#include "ugv/Constants.h"

#include <stdio.h>
#include <string.h>


RoboteqDevice device;

// Callback function for the /motors topic (accepts a ugv/motorvels message)
void motorCallback(const ugv::motorvels& msg)
{
  device.SetCommand(_GO, 2, (int)msg.left); // Set our left motor
  sleepms(10); // Allow proper time between commands
  device.SetCommand(_GO, 1, (int)msg.right); // Set our right motor
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

  // Prepare ROS
  ros::init(argc, argv, "ugv"); // Start a new ROS application called ugv

  ros::NodeHandle n; // Create a new node

  ros::Subscriber sub = n.subscribe("motors", 1000, motorCallback); // Create a subscriber to the /motors topic, that has a 1000-count message queue, and upon receiving a message calls the above callback function

  ros::spin(); // continously loop ROS (obtaining and processing messages in the process)

  device.Disconnect(); // Once broken from ROS, disconnect our motor driver.
  return 0;
}
