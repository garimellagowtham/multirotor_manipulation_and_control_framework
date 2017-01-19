#include "onboard_nodehandler.h"

using namespace std;
using namespace multirotor_manipulation_and_control_framework;

OnboardNodeHandler::OnboardNodeHandler(ros::NodeHandle &nh):nh_(nh)
                                                            , quad_controller_state_(DISABLED)
                                                            , arm_controller_state_(DISABLED)
                                                            , check_armstate(false)
                                                            , control_loop_period_(0.01)
                                                            , se3_(gcop::SE3::Instance())
{
  //Create Publishers and Subscribers
  quad_state_publisher = nh.advertise<multirotor_quad_state>("/quad_state", 1);
  arm_state_publisher = nh.advertise<multirotor_manipulator_state>("/arm_state", 1);
  barometer_publisher = nh.advertise<Barometer>("/barometer", 1);
  motorpwm_publisher = nh.advertise<MotorPwm>("/motor_out", 1);
  imu_publisher = nh.advertise<sensor_msgs::Imu>("/imu", 1);
  rpy_publisher = nh.advertise<geometry_msgs::Vector3>("/rpy", 1);
  gps_publisher = nh.advertise<sensor_msgs::NavSatFix>("/gps", 1);
  linvel_publisher = nh.advertise<geometry_msgs::Vector3>("/linvel", 1);
  windspeed_publisher = nh.advertise<WindSpeed>("/windspeed", 1);

  //Get Params:
  nh.getParam("/check_arm", check_armstate);
  nh.getParam("/control_loop_period", control_loop_period_);
  if(control_loop_period_ < 0.001)   {
    control_loop_period_ = 0.001;
    ROS_WARN("Controller period set too high binding to 1Khz");
  }
  else if(control_loop_period_ > 0.05)
  {
    control_loop_period_ = 0.05;
    ROS_WARN("Controller period set too low binding to 20Hz");
  }


  //Create Parsers and Controllers
  {
    std::string url = "";
    nh.getParam("/quad_url", url);
    try
    {
      quad_parser_.reset(new PixhawkParser(url));
    }
    catch(const std::invalid_argument& ia)
    {
      ROS_ERROR("Failed to open Quadcopter: %s",ia.what());
      quad_controller_state_ = CRITICAL;
    }
  }

  {
    int device_index = 0;
    int baudrate = 57600;
    if(nh.hasParam("arm_devid"))
      nh.getParam("/arm_devid",device_index);
    if(nh.hasParam("arm_baudrate"))
      nh.getParam("/arm_baudrate",baudrate);
    try
    {
      arm_parser_.reset(new SimpleArm(device_index, baudrate));
    }
    catch(const std::invalid_argument& ia)
    {
      ROS_ERROR("Failed to open arm: %s",ia.what());
      arm_controller_state_ = CRITICAL;
    }
  }

  state_estimator_.reset(new Point3DEstimator(nh, *quad_parser_));

  quad_controller_.reset(new TrajectoryTrackingController(*quad_parser_));

  arm_controller_.reset(new ArmController(*arm_parser_));

  //Connect signals:
  quad_parser_->signal_sensor_update_.connect(boost::bind(&OnboardNodeHandler::quadDataHandler, this, _1, _2));//Sends data to GUI

  arm_parser_->signal_feedback_received_.connect(boost::bind(&OnboardNodeHandler::armDataHandler, this, _1));//Sends arm data to GUI

  //State Handling:
  quad_parser_->signal_state_update_.connect(boost::bind(&OnboardNodeHandler::stateChanged, this, _1, _2));

  arm_parser_->signal_state_update_.connect(boost::bind(&OnboardNodeHandler::stateChanged, this, _1, _2));

  state_estimator_->signal_state_update_.connect(boost::bind(&OnboardNodeHandler::stateChanged, this, _1, _2));

  //Create a timer which does not start by itself
  quad_control_timer_ = nh.createTimer(ros::Duration(control_loop_period_), &OnboardNodeHandler::quadControlTimer, this, false, false);

  //General Subscribers:
  quad_gains_subscriber = nh.subscribe("/quad_gains", 2, &OnboardNodeHandler::quadGainsCallback, this);
}


void OnboardNodeHandler::quadDataHandler(const QuadcopterParser::SensorData &sensor_data, uint16_t mask)
{
  if(bitcheck(mask,0)&&(quad_state_publisher.getNumSubscribers() > 0))
  {
    multirotor_quad_state quad_state;
    quad_state.battery_voltage = sensor_data.battery_volts;
    quad_state.position_controller_state = quad_controller_state_;
    quad_state.quad_time = ros::Time(sensor_data.timestamp);
    quad_state.quad_state = sensor_data.quadstate;
    quad_state_publisher.publish(quad_state);
  }

  if(bitcheck(mask,1)&&(rpy_publisher.getNumSubscribers() > 0))
  {
    rpy_publisher.publish(sensor_data.rpydata);
  }

  if(bitcheck(mask,2)&&(barometer_publisher.getNumSubscribers() > 0))
  {
    Barometer barometer_msg;
    barometer_msg.pressure.fluid_pressure = sensor_data.pressure;
    barometer_msg.temperature.temperature = sensor_data.temperature;
    barometer_msg.altitude = sensor_data.altitude;
    barometer_msg.pressure.header.stamp = ros::Time(sensor_data.timestamp);
    barometer_publisher.publish(barometer_msg);
  }

  if(bitcheck(mask,3)&&(windspeed_publisher.getNumSubscribers() > 0))
  {
    WindSpeed windspeed_msg;
    windspeed_msg.wind_angle = sensor_data.wind_angle;
    windspeed_msg.wind_speed = sensor_data.wind_speed;
    windspeed_msg.header.stamp = ros::Time(sensor_data.timestamp);
    windspeed_publisher.publish(windspeed_msg);
  }

  if(bitcheck(mask,4)&&(motorpwm_publisher.getNumSubscribers() > 0))
  {
    MotorPwm pwm_msg;
    for(int count = 0; count < 8; count++)
      pwm_msg.motorpwm[count] = sensor_data.motorpwm[count];
    pwm_msg.header.stamp = ros::Time(sensor_data.timestamp);
    motorpwm_publisher.publish(pwm_msg);
  }

  if(bitcheck(mask,5)&&(imu_publisher.getNumSubscribers() > 0))
  {
    sensor_msgs::Imu imu_msg;
    imu_msg.linear_acceleration = sensor_data.linacc;
    imu_msg.angular_velocity = sensor_data.angvel;
    imu_publisher.publish(imu_msg);
  }

  if(bitcheck(mask,6)&&(gps_publisher.getNumSubscribers() > 0))
  {
    sensor_msgs::NavSatFix gps_msg;
    gps_msg.latitude = double(sensor_data.gps_raw[0]);
    gps_msg.longitude = double(sensor_data.gps_raw[1]);
    gps_msg.altitude = double(sensor_data.gps_raw[2]);
    gps_msg.header.stamp = ros::Time(sensor_data.timestamp);
    gps_publisher.publish(gps_msg);
  }

  if(bitcheck(mask,7)&&(linvel_publisher.getNumSubscribers() > 0))
  {
    linvel_publisher.publish(sensor_data.linvel);
  }
}

void OnboardNodeHandler::armDataHandler(const ArmParser::ArmSensorData &sensor_data)
{
  if(arm_state_publisher.getNumSubscribers() > 0)
  {
    sensor_msgs::JointState arm_msg;
    arm_msg.position = sensor_data.joint_angles_;
    arm_msg.velocity = sensor_data.joint_velocities_;
    arm_msg.header.stamp = ros::Time(sensor_data.timestamp);

    //Publish the arm state msg also:
    multirotor_manipulator_state state_msg;
    state_msg.arm_controller_state = arm_controller_state_;
    state_msg.arm_state = sensor_data.arm_state;
    arm_state_publisher.publish(state_msg);
  }
}

void OnboardNodeHandler::guiMsgHandler(const multirotor_manipulator_commands::ConstPtr &gui_command)
{
  //Handle the input from user
  if(bitcheck(gui_command->mask, 0))
  {
    if(gui_command->quad_arm)
    {
      if(quad_parser_->state == QuadcopterParser::DISABLED && quad_controller_state_ == DISABLED)
      {
        quad_parser_->takeoff();
      }
      else
      {
        ROS_INFO("Quad Parser state [DISABLED]: %d, Quad Controller State[DISABLED]: %d",(quad_parser_->state), quad_controller_state_);
      }//PRECHECKS
    }//ARM_QUAD
    else
    {
      if(quad_parser_->state == QuadcopterParser::ENABLED)
      {
        quad_parser_->disarm();
      }
      else
      {
        ROS_INFO("Quad Parser state [ENABLED]:%d", quad_parser_->state);
      }
    }//DISARM_QUAD
  }//CHECK_MASK

  if(bitcheck(gui_command->mask, 1))
  {
    if(gui_command->manipulator_arm)
    {
      if(arm_parser_->state == ArmParser::DISABLED)
      {
        arm_parser_->powerMotors(true);
      }
    }
    else
    {
      if(arm_parser_->state == ArmParser::ENABLED)
      {
        arm_parser_->powerMotors(false);
      }
    }
  }//CHECK_MASK

  if(bitcheck(gui_command->mask, 2))
  {
    if(gui_command->enable_quadcontrol)
    {
      //Check Current State
      if(quad_parser_->state == QuadcopterParser::ENABLED && quad_controller_state_ == DISABLED && state_estimator_->state != StateEstimator::CRITICAL)
      {
        //Additional check to see arm is enabled if arm is used
        if(!check_armstate || (arm_parser_->state == ArmParser::ENABLED))
        {
          if(gui_command->quad_controlmode == gui_command->POSITION_CONTROL)
          {
            //#TODO Disable Optimal control etc
            quad_goal_subscriber = nh_.subscribe("/quad_goal", 2, &OnboardNodeHandler::quadGoalCallback, this);
            quad_control_timer_.start();
          }
          else if(gui_command->quad_controlmode == gui_command->OPTIMAL_CONTROL)
          {
            //quad_control_timer.stop();//Stop the timer first
            //#TODO Start a timer for Optimal control
          }
          quad_controller_state_ = ENABLED;
        }//ADDITION_PRECHECKS
      }//PRE_CHECKS
    }//ENABLE_QUADCONTROL
    else
    {
      quad_goal_subscriber.shutdown();
      quad_control_timer_.stop();
    }
  }

  if(bitcheck(gui_command->mask, 3))
  {
    if(gui_command->enable_armcontrol)
    {
      //Do Prechecks for arm control
      if(arm_parser_->state == ArmParser::ENABLED && arm_controller_state_ == DISABLED)
      {
        //Subscribe to external pose
        if(gui_command->arm_controlmode == gui_command->MARKER_CONTROL)
        {
          //Close other subscribers:
          arm_endeffector_subscriber.shutdown();
          arm_joint_subscriber.shutdown();
          marker_subscriber = nh_.subscribe("/cam_marker_pose", 1, &OnboardNodeHandler::markerCallback, this);
        }
        else if(gui_command->arm_controlmode == gui_command->JOINT_CONTROL)
        {
          //Close other subscribers:
          marker_subscriber.shutdown();
          arm_endeffector_subscriber.shutdown();
          arm_joint_subscriber = nh_.subscribe("/joint_commands", 2, &OnboardNodeHandler::jointCallback, this);
        }
        else if(gui_command->arm_controlmode == gui_command->ENDEFFECTOR_CONTROL)
        {
          //Close other subscribers:
          marker_subscriber.shutdown();
          arm_joint_subscriber.shutdown();
          arm_endeffector_subscriber = nh_.subscribe("/end_effector_commands", 2, &OnboardNodeHandler::endeffectorCallback, this);
        }
        //Call the setCtrl function
        arm_controller_state_ = ENABLED;
      }//PRECHECKS
    }//ENABLE_ARMCONTROL
    else
    {
      //Disable arm controller
      marker_subscriber.shutdown();
      arm_joint_subscriber.shutdown();
      arm_endeffector_subscriber.shutdown();
      arm_controller_state_ = DISABLED;
    }
  }//CHECK_MASK



}

void OnboardNodeHandler::stateChanged(const uint8_t &state, int id)
{
  //Check who sent the message
  //Take action based on the current state
  switch(id)
  {
  case 0://Quad Parser
    if(state == CRITICAL || state == DISABLED)
    {
      //Switch off Quad Controller:
      quad_goal_subscriber.shutdown();
      quad_control_timer_.stop();

      //Close arm controller
      marker_subscriber.shutdown();
      arm_joint_subscriber.shutdown();
      arm_endeffector_subscriber.shutdown();
      arm_controller_state_ = DISABLED;

      //Reset arm parser:
      if(arm_parser_->state == ArmParser::ENABLED)
      {
        arm_parser_->powerMotors(true);//Resets the Arm
      }
    }
    break;
  case 1://Arm Parser
    if(state == CRITICAL || state == DISABLED)
    {
      //Close arm controller:
      marker_subscriber.shutdown();
      arm_joint_subscriber.shutdown();
      arm_endeffector_subscriber.shutdown();
      arm_controller_state_ = DISABLED;
    }
  case 2://State Estimator
    if(state ==CRITICAL || state == DISABLED)
    {
      //Close arm controller
      marker_subscriber.shutdown();
      arm_joint_subscriber.shutdown();
      arm_endeffector_subscriber.shutdown();
      arm_controller_state_ = DISABLED;

      //Close Quad controller
      quad_goal_subscriber.shutdown();
      quad_control_timer_.stop();
    }
  }

}

void OnboardNodeHandler::quadControlTimer(const ros::TimerEvent &event_data)
{
  //#TODO Check even data to specify bad times
  gcop::Body3dState current_state;
  state_estimator_->predictState(current_state);
  quad_controller_->setCtrl(current_state);
}

void OnboardNodeHandler::markerCallback(const geometry_msgs::Pose::ConstPtr &pose)
{
  //Convert pose to Eigen
  Eigen::Matrix4d cam_object_transform;
  Vector7d pose_vec;
  pose_vec<< pose->orientation.w, pose->orientation.x, pose->orientation.y, pose->orientation.x, pose->orientation.y, pose->orientation.z
            ,pose->position.x, pose->position.y, pose->position.z;
  se3_.quatxyz2g(cam_object_transform,pose_vec);//Convert to Eigen
  arm_controller_->setCtrl(cam_object_transform);
}

void OnboardNodeHandler::jointCallback(const sensor_msgs::JointState::ConstPtr &command_state)
{
  if(command_state->velocity.size() != 0)
    arm_parser_->setAngles(command_state->position, &command_state->velocity);
  else
    arm_parser_->setAngles(command_state->position);
}

void OnboardNodeHandler::quadGoalCallback(const BodyState::ConstPtr &state)
{
  gcop::Body3dState goal_state;

  //Convert pose to Eigen
  Eigen::Vector4d quat;
  quat<< state->pose.orientation.w, state->pose.orientation.x, state->pose.orientation.y, state->pose.orientation.x
         , state->pose.orientation.y, state->pose.orientation.z;
  se3_.so3.quat2g(goal_state.R, quat);
  goal_state.p<<state->pose.position.x, state->pose.position.y, state->pose.position.z;
  goal_state.w<<state->twist.angular.x, state->twist.angular.y, state->twist.angular.z;
  goal_state.v<<state->twist.linear.x, state->twist.linear.y, state->twist.linear.z;

  quad_controller_->setGoal(goal_state);
}

void OnboardNodeHandler::quadGainsCallback(const QuadControllerGains::ConstPtr &gains)
{
  TrajectoryTrackingController::ControllerGains gains_;// Local copy
  gains_.kpr = gains->kpr;
  gains_.kpy = gains->kpy;
  gains_.kpt = gains->kpt;
  gains_.kdr = gains->kdr;
  gains_.kdt = gains->kdt;
  gains_.kiy = gains->kiy;
  gains_.kit = gains->kit;

  quad_controller_->setGains(gains_);
}
