#include <iostream>

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

#include <boost/asio/serial_port.hpp>
#include <boost/asio.hpp>

#include <unistd.h>

using namespace boost;

class RobotDriver
{
private:
  //! The node handle we'll be using
  ros::NodeHandle nh_;
  //! We will be publishing to the "/base_controller/command" topic to issue commands
  ros::Publisher cmd_vel_pub_;



  double map(int x, int in_min, int in_max, double out_min, double out_max)
  {
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
  }
public:
  //! ROS node initialization
  RobotDriver(ros::NodeHandle &nh)
  {
    nh_ = nh;
    //set up the publisher for the cmd_vel topic
    cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/diff_drive_controller/cmd_vel", 1);
  }

  //! Loop forever while sending drive commands based on keyboard input
  bool driveController()
  {
    //we will be sending commands of type "twist"
    geometry_msgs::Twist base_cmd;
    base_cmd.linear.x = base_cmd.linear.y = base_cmd.angular.z = 0;

    char c, x_c, y_c;
    asio::io_service io;
    asio::serial_port port(io);
    system::error_code ec;
    port.open("/dev/ttyACM0");
    port.set_option(asio::serial_port_base::baud_rate(115200));

    while(nh_.ok()){

      
      asio::read(port, asio::buffer(&c,1), ec);
      std::cout<<c<<","<<ec<<std::endl;
        if(!ec||ec==asio::error::eof){c=0;} // Do nothing

      std::cout << "* Read in: " << c << std::endl;
      
      if(c=='K') {
        usleep(2000);
        asio::read(port, asio::buffer(&x_c,1), ec);
        if(!ec||ec==asio::error::eof){x_c=110;} // Do nothing
        usleep(2000);
        asio::read(port, asio::buffer(&y_c,1), ec);
        if(!ec||ec==asio::error::eof){y_c=110;} // Do nothing




        double x = map((int)x_c,97,122,-1.0,1.0);
        double y = map((int)y_c,97,122,0.3,-0.3);
        if(x<0.3 && x>-0.3) x=0;
        if(y<0.1 && y>-0.1) y=0;
        if(x>1.0 || x<-1.0) x=0;
        if(y>0.3 || y<-0.3) y=0;
        if(y>0.275 && y<0.277) y=0;
        base_cmd.angular.z = x;
        base_cmd.linear.x = y;

        std::cout<<"Command: "<<x_c<<","<<y_c<<":"<<x<<","<<y<<std::endl;

        //publish the assembled command
        cmd_vel_pub_.publish(base_cmd);
      }

      

      // 100ms delay
      usleep(1000);
    }
    return true;
  }

};

int main(int argc, char** argv)
{
  //init the ROS node
  ros::init(argc, argv, "controller");
  ros::NodeHandle nh;

  RobotDriver driver(nh);
  // driver.driveKeyboard();
  driver.driveController();
}
