#include "onboard_nodehandler.h"

using namespace std;
using namespace multirotor_manipulation_and_control_framework;

OnboardNodeHandler::OnboardNodeHandler(ros::NodeHandle &nh):nh_(nh), quad_handler_(*this), arm_handler_(*this)
{
  //Create Publishers and Subscribers
  quad_state_publisher = nh.advertise<multirotor_quad_state>("quad_state", 1);
  arm_state_publisher = nh.advertise<multirotor_manipulator_state>("arm_state", 1);
  barometer_publisher = nh.advertise<Barometer>("barometer", 1);
  motorpwm_publisher = nh.advertise<MotorPwm>("motor_out", 1);
  imu_publisher = nh.advertise<sensor_msgs::Imu>("imu", 1);
  rpy_publisher = nh.advertise<geometry_msgs::Vector3>("rpy", 1);
  gps_publisher = nh.advertise<sensor_msgs::NavSatFix>("gps", 1);
  linvel_publisher = nh.advertise<geometry_msgs::Vector3>("linvel", 1);
  windspeed_publisher = nh.advertise<WindSpeed>("windspeed", 1);

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

  //Connect signals:
  quad_parser_->signal_sensor_update_.connect(boost::bind(&OnboardNodeHandler::quadDataHandler, this, _1, _2));//Sends data to GUI

  arm_parser_->signal_feedback_received_.connect(boost::bind(&OnboardNodeHandler::armDataHandler, this, _1));//Sends arm data to GUI

}

void OnboardNodeHandler::quadDataHandler(const QuadcopterParser::SensorData &sensor_data, uint16_t mask)
{
  if(mask& 1)
  {
    multirotor_quad_state quad_state;
    quad_state.battery_voltage = sensor_data.battery_volts;
    quad_state.position_controller_state = quad_controller_->state;
    quad_state.quad_time = ros::Time(sensor_data.timestamp);
    quad_state.quad_state = sensor_data.quadstate;
    quad_state_publisher.publish(quad_state);
  }

  if(mask & (1<<1))
  {
    rpy_publisher.publish(sensor_data.rpydata);
  }

  if(mask & (1<<2))
  {
    Barometer barometer_msg;
    barometer_msg.pressure.fluid_pressure = sensor_data.pressure;
    barometer_msg.temperature.temperature = sensor_data.temperature;
    barometer_msg.altitude = sensor_data.altitude;
    barometer_msg.pressure.header.stamp = ros::Time(sensor_data.timestamp);
    barometer_publisher.publish(barometer_msg);
  }

  if(mask & (1<<3))
  {
    WindSpeed windspeed_msg;
    windspeed_msg.wind_angle = sensor_data.wind_angle;
    windspeed_msg.wind_speed = sensor_data.wind_speed;
    windspeed_msg.header.stamp = ros::Time(sensor_data.timestamp);
    windspeed_publisher.publish(windspeed_msg);
  }

  if(mask & (1<<4))
  {
    MotorPwm pwm_msg;
    for(int count = 0; count < 8; count++)
      pwm_msg.motorpwm[count] = sensor_data.motorpwm[count];
    pwm_msg.header.stamp = ros::Time(sensor_data.timestamp);
    motorpwm_publisher.publish(pwm_msg);
  }

  if(mask & (1<<5))
  {
    sensor_msgs::Imu imu_msg;
    imu_msg.linear_acceleration = sensor_data.linacc;
    imu_msg.angular_velocity = sensor_data.angvel;
    imu_publisher.publish(imu_msg);
  }

  if(mask & (1<<6))
  {
    sensor_msgs::NavSatFix gps_msg;
    gps_msg.latitude = double(sensor_data.gps_raw[0]);
    gps_msg.longitude = double(sensor_data.gps_raw[1]);
    gps_msg.altitude = double(sensor_data.gps_raw[2]);
    gps_msg.header.stamp = ros::Time(sensor_data.timestamp);
    gps_publisher.publish(gps_msg);
  }

  if(mask & (1<<7))
  {
    linvel_publisher.publish(sensor_data.linvel);
  }
}

void OnboardNodeHandler::armDataHandler(const ArmParser::ArmSensorData &sensor_data)
{
}
void OnboardNodeHandler::guiMsgHandler(const multirotor_manipulator_commands::ConstPtr &gui_command)
{
  //Handle the input from user
  if(gui_command->quad_arm)
  {
    //CHECK CURRENT STATE
    //DO PRECHECKS
    if(!quad_parser_->takeoff())
    {
      //CHANGE STATES NEEDED
      ROS_INFO("Arming Failed");
      return;
    }
  }
  if(gui_command->quad_disarm)
  {
    if(!quad_parser_->disarm())
    {
      //CHANGE STATES NEEDED
      ROS_INFO("Disarming Failed");
      return;
    }
  }
  if(gui_command->enable_armcontrol)
  {
    //Check Current State
    //Do Prechecks for arm control
    //Enable arm control
  }
  if(gui_command->clear_rcoverride)
  {
    //Clear rc override
  }
  if(gui_command->enable_positioncontrol)
  {
    //Check Current State
    //Do Prechecks for position control
  }
}

/****************** QuadcopterControlHandler Implementation **************/
OnboardNodeHandler::QuadcopterControlHandler::QuadcopterControlHandler(OnboardNodeHandler &node_handler):node_handler_(node_handler)
{
}

bool OnboardNodeHandler::QuadcopterControlHandler::preChecks()
{
  //Check if you can enable Position Controller
}
void OnboardNodeHandler::QuadcopterControlHandler::quadDataHandler(const QuadcopterParser::SensorData &sensor_data, uint16_t mask)
{
  //Check for any critical changes etc
}


/****************** ArmControlHandler Implementation ********************/
OnboardNodeHandler::ArmControlHandler::ArmControlHandler(OnboardNodeHandler &node_handler):node_handler_(node_handler)
{
}

bool OnboardNodeHandler::ArmControlHandler::preChecks()
{
  //Check if you can enable position Controller
}

void OnboardNodeHandler::ArmControlHandler::armDataHandler(const ArmParser::ArmSensorData &sensor_data)
{
  //Check if arm is healthy and change the state of handler accordingly
}
