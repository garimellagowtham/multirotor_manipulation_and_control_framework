#include <ros/ros.h>
#include <quadcopter_parsers/pixhawk_parser.h>

using namespace std;

boost::shared_ptr<QuadcopterParser> parser_instance_;

inline void printMenu()
{
  cout<<"Menu: \n"<<"a:\t Arm "<<"d:\t Disarm "<<endl;
}

void receiveSensorData(const QuadcopterParser::SensorData &sensor_data, uint16_t mask)
{
  if(mask & 1)//If mask has first bit then we received status
  {
    static ros::Time t1 = ros::Time::now();//Initialize
    ros::Time t2 = ros::Time::now();
    ROS_INFO("Time taken from previous msg: %f",(t2 - t1).toSec());
    parser_instance_->printSensorData();//Print the data; We can also use sensor data here somehow
    t1 = t2;
  }
}

int main(int argc, char **argv)
{
  //Initialize ROS
  ros::init(argc, argv, "pixhawk_parser");

  ros::NodeHandle nh("~");

  //Obtain URL to open Quadcopter
  std::string url = "/dev/ttyACM0:115200";

  if(nh.hasParam("fcu_url"))
    nh.getParam("fcu_url", url);

  ROS_INFO("URL: %s",url.c_str());

  //Create the pixhawk parser to receive and send quadcopter commands
  parser_instance_.reset(new PixhawkParser(url));
  parser_instance_->printSensorData();
  assert(!parser_instance_->open_error);//Check there is no opening error

  //Connect Boost signal to receive and print sensor data
  parser_instance_->signal_sensor_update_.connect(receiveSensorData);//Connect the signal to receive function

  //Loop at 20 Hz to receive user feedback
  ros::Rate rate(20);
  while(ros::ok())
  {
    printMenu();
    char c = getchar();
    switch(c)
    {
      case 'a':
        parser_instance_->takeoff();//ARM
        //For 30 seconds keep sending nominal control to check the if radio can interfere:
        for(int i = 0; i < 60; i++)
        {
          QuadcopterParser::ControlCommand ctrl_cmd;///Command to send after arming (#TEST)
          ctrl_cmd.command_pitch = ctrl_cmd.command_roll = ctrl_cmd.command_rate_yaw = 0;
          ctrl_cmd.command_thrust = 5;//Send some nominal thrust
          parser_instance_->command(ctrl_cmd);
          usleep(500000);//0.5sec
        }

        break;
      case 'd':
        parser_instance_->disarm();//Disarm
        break;
      case 'q':
        parser_instance_.reset();//Close Parser
        ros::shutdown();//Quit
        break;
    }
    ros::spinOnce();
    rate.sleep();
  }
}
