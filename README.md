This is a ROS package used for an autonomous UGV for research at the University of Texas at Arlington.

This package works on differential drive robots using a Roboteq motor controller. (Tested on HDC2460 and MDC2460)
This package has been tested successfully with ROS Kinetic and Melodic

TODO: At a later date, I will add the configuration file for my motor driver (MDC2460)

ROS Parameters
* /motor_controller_usb, which Linux device the Roboteq controller is registered as. Default: /dev/ttyACM0
* /ugv_vel_in_scalar, Default: (PI/3.0)
* /ugv_vel_out_scalar, Default: (5.30516666)
* /ugv_odom_scalar, Default: (PI/25200.0)
        
Please ensure your controller has the following settings:
* Left wheel connected to motor 1
* Right wheel connected to motor 2
* Encoders connected, tested, and set to tuned PID coefficients in closed-loop speed control
* Encoder ticks per revolution properly calculated through gear ratio and encoder specs
* Max velocity for each wheel set to 60 RPM
* Some parameters may need to be tuned and changed based on your build. Trial and error.

Edit UGV.urdf to fit the dimensions of your robot
Edit ugv.launch to have the correct wheel_separation and wheel_radius for your robot

After wheels respond to correct direction, speed can be adjusted through proper calculation (or trial and error) of /ugv_vel_in_scalar or /ugv_vel_out_scalar
After wheels respond to correct speed, odom can be adjusted through proper calculation (or trial and error, listed below) of /ugv_odom_scalar

My current method for tuning odom:
* Visualize in Rviz with /odom as global frame.
* View lidar and transform tree
* Rotate robot clockwise.
* * If lidar scan rotates to the right: Decrease the /ugv_odom_scalar parameter
* * If lidar scan rotates to the left:  Increase the /ugv_odom_scalar parameter
* When robot is able to rotate N-rotations without rotation of the laser scan, odom is properly tuned (my N=100)
