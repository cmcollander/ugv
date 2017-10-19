// Partially based off example code at 
// https://github.com/ros-controls/ros_controllers/blob/kinetic-devel/diff_drive_controller/test/diffbot.cpp

#include "UGV.cpp"
#include <ros/ros.h>
#include <controller_manager/controller_manager.h>


int main(int argc, char** argv) {
        ros::init(argc, argv, "UGV");
        ros::NodeHandle nh;

        UGV ugv;
        ROS_WARN_STREAM("period: " << ugv.getPeriod().toSec());
        controller_manager::ControllerManager cm(&ugv,nh);

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

