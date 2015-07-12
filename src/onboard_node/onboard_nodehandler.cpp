#include "onboard_nodehandler.h"

using namespace std;

OnboardNodeHandler::OnboardNodeHandler(ros::NodeHandle &nh):nh_(nh)
{
  //Create Publishers and Subscribers
  //Create Parsers and Controllers
  {
    std::string url;
    nh.getParam("url", url);
    try
    {
      quad_parser_.reset(new PixhawkParser(url));
    }
    catch(const std::invalid_argument& ia)
    {
      ROS_ERROR("Invalid argument: %s",ia.what());
      return;
    }
  }

  {
    int device_index = 0;
    int baudrate = 57600;
    if(nh.hasParam("arm_devid"))
      nh.getParam("arm_devid",device_index);
    if(nh.hasParam("arm_baudrate"))
      nh.getParam("arm_baudrate",baudrate);
    try
    {
      arm_parser_.reset(new SimpleArm(device_index, baudrate));
    }
    catch(const std::invalid_argument& ia)
    {
      ROS_ERROR("Invalid argument: %s",ia.what());
      return;
    }
  }

  state_estimator_.reset(new Point3DEstimator(nh, *quad_parser_));

  quad_controller_.reset(new TrajectoryTrackingController(*quad_parser_));

  arm_controller_.reset(new ArmController(*arm_parser_));

  //Initialize State Machine

}
