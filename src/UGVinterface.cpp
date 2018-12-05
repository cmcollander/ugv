/*
Chris Collander, cmcollander@gmail.com, December 2018

This ROS node is used with UGV.cpp for communication between /cmd_vel and /odom to a Roboteq motor controller.

ROS Parameters
	/motor_controller_usb, which Linux device the Roboteq controller is registered as. Default: /dev/ttyACM0
        /ugv_vel_in_scalar, Default: (PI/3.0)
        /ugv_vel_out_scalar, Default: (15.9155*0.33333333)
        /ugv_odom_scalar, Default: (PI/(6000*4.20))

Please ensure your controller has the following settings:
* Left wheel connected to motor 1
* Right wheel connected to motor 2
* Encoders connected, tested, and set to tuned PID coefficients in closed-loop speed control
* Encoder ticks per revolution properly calculated through gear ratio and encoder specs
* Max velocity for each wheel set to 60 RPM
* Some parameters may need to be tuned and changed based on your build. Trial and error.
TODO: At a later date, I will add the configuration file for my motor driver (MDC2460)

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

#include "UGV.cpp"
#include <ros/ros.h>
#include <controller_manager/controller_manager.h>

#define PI 3.14159265359

// Default input/output scalar values
#define USB_DEFAULT "/dev/ttyACM0"
#define VEL_IN_DEFAULT (PI/3.0)
#define VEL_OUT_DEFAULT (15.9155*0.33333333)
#define ODOM_DEFAULT (PI/(6000*4.20))

int main(int argc, char** argv) {
        ros::init(argc, argv, "UGV");
        ros::NodeHandle nh;
        
        // Parameter Variables
        std::string MotorControllerUSB;
        double velInScalar;
        double velOutScalar;
        double odomScalar;
        
        n.param<std::string>("/motor_controller_usb", MotorControllerUSB, USB_DEFAULT);
        n.param<double>("/ugv_vel_in_scalar", velInScalar, VEL_IN_DEFAULT);
        n.param<double>("/ugv_vel_out_scalar", velOutScalar, VEL_OUT_DEFAULT);
        n.param<double>("/ugv_odom_scalar", odomScalar, ODOM_DEFAULT);

        // Create our hardware_interface and connect it to the controller manager
        UGV ugv(MotorControllerUSB,velInScalar,velOutScalar,odomScalar);
        controller_manager::ControllerManager cm(&ugv,nh);

        ROS_WARN_STREAM("period: " << ugv.getPeriod().toSec());
        ros::Rate rate(1.0 / ugv.getPeriod().toSec());
        ros::AsyncSpinner spinner(1);
        spinner.start();

        while(ros::ok()) {
                ugv.read();
                cm.update(ugv.getTime(), ugv.getPeriod());
                ugv.write();
                rate.sleep();
        }
        spinner.stop();
        return 0;
}

