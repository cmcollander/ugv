This is a ROS package used to control the autonomous UGV for CSE Senior Design 1 at the University of Texas at Arlington.

This was made for ROS-Kinetic (has not been tested for other versions)

Full descriptions of important executables are listed at the bottom

TO SETUP AND CONTROL

First, set up the PC
* Ensure the PC is properly connected to the network with the IP Address of 192.168.0.100
* Ensure the PC has access to the internet
* Ensure the router's WiFi is enabled and within range for the Server
* Ensure the PC has ROS-Kinetic installed
* Clone this repository so that the ugv folder is located at ~/catkin_ws/src/ugv
* Ensure that, in ugv/CMakeLists.txt, the sections at the bottom of the file for both UGVInterface and LaserBroadcaster are commented out, while the section for controller is enabled
* Run catkin_make, resolving any prompted dependencies
* In a terminal on the PC, run 'roscore'

Second, set up the UGV
* Ensure the Pi is wired into the network with the IP address 192.168.0.102
* Ensure the router is properly connected and functioning, within Wifi range of the UGV
* Ensure the batteries for the UGV are properly connected and charged.
* Flip the power switch and ensure the emergency button is depressed
* Allow 2 minutes for the components to start up
* SSH into the Pi from the Server
* Ensure the Pi has ROS-Kinetic installed
* Clone this repository so that the ugv folder is located at ~/catkin_ws/src/ugv
* Ensure that, in ugv/CMakeLists.txt, the sections at the bottom of the file for both UGVInterface and LaserBroadcaster are enabled, while the section for controller is commented out
* Run catkin_make, resolving any prompted dependencies

Third, back to the PC
* In a terminal on the PC, run 'roscore'
* In a separate PC terminal, SSH into the Pi, and type the command 'roslaunch ugv ugv.launch'
* In a separate PC terminal, type the command 'roslaunch ugv keyboard.launch'
* Use the keyboard arrow keys to ensure proper movement of the UGV
* In a separate PC terminal, type the command 'roslaunch ugv config2.launch' (this example uses the SLAM config2, but feel free to experiment with others)
* Using the 'Goal Placement' feature of RViz, send goal commands to the UGV.
