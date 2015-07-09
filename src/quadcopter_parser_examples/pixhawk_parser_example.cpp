#include <ros/ros.h>
#include <multirotor_manipulation_and_control_framework/pixhawk_parser.h>

using namespace std;

boost::shared_ptr<QuadcopterParser> parser_instance_;

inline void printMenu()
{
  cout<<"Menu: \n"<<"p:\t Print "<<"a:\t Arm "<<"d:\t Disarm "<<endl;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "pixhawk_parser");

  ros::NodeHandle nh("~");

  std::string url = "/dev/ttyACM0:115200";

  if(nh.hasParam("fcu_url"))
    nh.getParam("fcu_url", url);

  ROS_INFO("URL: %s",url.c_str());
  //Create the pixhawk parser
  parser_instance_.reset(new PixhawkParser(url));
  parser_instance_->printSensorData();
  //assert(!parser_instance_->open_error);//Check there is no open error
  ros::Rate rate(20);
  //DEBUG_TEST RADIO:
  QuadcopterParser::ControlCommand ctrl_cmd;///Command to send after arming (#TEST)
  ctrl_cmd.command_pitch = ctrl_cmd.command_roll = ctrl_cmd.command_rate_yaw = 0;
  ctrl_cmd.command_thrust = 5;//Send some nominal thrust
  while(ros::ok())
  {
    printMenu();
    char c = getchar();
    switch(c)
    {
      case 'p':
        parser_instance_->printSensorData();//PRINT DATA
        break;
      case 'a':
        parser_instance_->takeoff();//ARM
        //For 30 seconds keep sending nominal control to check the if radio can interfere:
        for(int i = 0; i < 60; i++)
        {
          parser_instance_->command(ctrl_cmd);
          parser_instance_->printSensorData();//PRINT DATA
          usleep(500000);//0.5sec
        }

        break;
      case 'd':
        parser_instance_->disarm();//Disarm
    }
    ros::spinOnce();
    rate.sleep();
  }
}
