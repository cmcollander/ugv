This is a ROS package used to control the autonomous UGV for CSE Senior Design 1 at the University of Texas at Arlington.

This was made for ROS-Kinetic (has not been tested for other versions)

Full descriptions of important executables are listed at the bottom

TO CONTROL

First, set up the PC
* Ensure the PC is properly connected to the network with the IP Address of 192.168.0.100
* Ensure the PC has access to the internet
* Ensure the router's WiFi is enabled and within range for the UGV
* In a terminal on the PC, run 'roscore'

Second, set up the UGV
* Ensure the PC is wired into the network with the IP address 192.168.0.100
* Ensure the router is properly connected and functioning, within Wifi rane of the UGV
* Ensure the batteries for the UGV are properly connected and charged.
* Flip the power switch
* Allow 2 minutes for the components to start up

Third, back to the PC
* In a terminal, run 'ssh pi@192.168.0.102'
* Enter the password 'raspberry'
* In this new SSH session to the UGV, run 'roslaunch ugv ugv.launch'
* In a new terminal on the PC, run 'roslaunch ugv controller_hector.launch'
* Flip the power switch on the custom controller
* DRIVE!

LIST OF EXECUTABLES

launch/ugv.launch
* connects to the SICK LM100 LiDAR
* Starts a node that subscribes to the /motors topic, and translates values into motor speeds (2 valued vector, values between -1000 and 1000)

launch/controller_hector.launch
* Starts up the full Hector SLAM algorithm with parameters for both localization detection and environment mapping
* Starts up connection to the custom controller, publishing values to the /motors topic

src/ugv.cpp
* main node for the /motors subscriber, including translation out to the motors

src/randomcontrol.cpp
* Randomly sends values between -1000 and 1000 to /motors at a 1 Hz frequency. FOR TESTING ONLY! DO NOT RUN WHEN ON GROUND!

src/keyboardcontrol.cpp
* Translates arrow key values to /motors
* Speed is based on the ROS parameter 'ugvmaxspeed'
* Any button that is not an arrow key (such as space) stops all motion

src/joystickcontrol.cpp
* Translates values from the /joy node to /motors, using single joystick to differential calculation
* Requires 'rosrun /joy joy-node'
* Uses the XBOX 360 controller
* Speed is baesd on the ROS parameter 'ugvmaxspeed'
