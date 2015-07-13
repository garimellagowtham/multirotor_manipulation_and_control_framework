/* Copyright (C)
* 2015 - Gowtham Garimella
* This program is free software; you can redistribute it and/or
* modify it under the terms of the GNU General Public License
* as published by the Free Software Foundation; either version 2
* of the License, or (at your option) any later version.
* 
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
* GNU General Public License for more details.
* 
* You should have received a copy of the GNU General Public License
* along with this program; if not, write to the Free Software
* Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.
* 
*/
#ifndef ONBOARD_NODEHANDLER_H
#define ONBOARD_NODEHANDLER_H

#include <ros/ros.h>
#include <quadcopter_parsers/pixhawk_parser.h>

#include <arm_parsers/simple_arm.h>

#include <filters/point3d_estimator.h>

#include <arm_controllers/arm_controller.h>

#include <quadcopter_controllers/trajectory_tracking_control.h>

#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>

#include <multirotor_manipulation_and_control_framework/multirotor_manipulator_state.h>
#include <multirotor_manipulation_and_control_framework/multirotor_manipulator_commands.h>
#include <multirotor_manipulation_and_control_framework/multirotor_quad_state.h>
#include <multirotor_manipulation_and_control_framework/WindSpeed.h>
#include <multirotor_manipulation_and_control_framework/MotorPwm.h>
#include <multirotor_manipulation_and_control_framework/Barometer.h>

using namespace std;
using namespace multirotor_manipulation_and_control_framework;

/**
* @brief This class provides state machine to combine different components:
* > Arm Controller
* > Quadcopter Controller
* > State Estimator
* This class combines the data from these different classes and sends to the GUI. It also runs these
* classes based on inputs from GUI
*/
class OnboardNodeHandler
{
public:
    OnboardNodeHandler(ros::NodeHandle &nh);
protected:
    void quadDataHandler(const QuadcopterParser::SensorData &sensor_data, uint16_t mask);
    void armDataHandler(const ArmParser::ArmSensorData &sensor_data);
    void guiMsgHandler(const multirotor_manipulator_commands::ConstPtr &gui_command);
protected:
    class QuadcopterControlHandler
    {
      public:
        enum State{
          Enabled,///< Controller enabled
          DISABLED,///< Controller disabled
          CRITICAL///< Controller critical
        };
        State state;///< Current State
      public:
        QuadcopterControlHandler(OnboardNodeHandler &node_handler);
      protected:
        bool preChecks();
        void quadDataHandler(const QuadcopterParser::SensorData &sensor_data, uint16_t mask);
      protected:
        OnboardNodeHandler &node_handler_;///< Parent Class to access other states

    };
    class ArmControlHandler
    {
      public:
        enum State{
          ENABLED,///< Controller enabled
          DISABLED,///< Controller disabled
          CRITICAL///< Controller critical
        };
        State state;///< Current State
      public:
        ArmControlHandler(OnboardNodeHandler &node_handler);
      protected:
        bool preChecks();
        void armDataHandler(const ArmParser::ArmSensorData &sensor_data);
      protected:
        OnboardNodeHandler &node_handler_;///< Parent Class to access other states
    };
protected:
    ros::NodeHandle nh_;///< Internal nodehandle to publish and subscribe
    QuadcopterControlHandler quad_handler_;///< State Machine for Quadcopter Control
    ArmControlHandler arm_handler_;///< StateMachine for Arm Control
    boost::shared_ptr<ArmParser> arm_parser_;///< Parser to talk to the manipulator
    boost::shared_ptr<QuadcopterParser> quad_parser_;///< Parser to talk to the quadcopter
    boost::shared_ptr<StateEstimator> state_estimator_;///< Estimator to get the state of the quadcopter
    boost::shared_ptr<TrajectoryTrackingController> quad_controller_;///< Controller to track trajectories
    boost::shared_ptr<ArmController> arm_controller_;///< Controller to grab objects
    ros::Publisher quad_state_publisher;///< Publish Quadcopter State
    ros::Publisher arm_state_publisher;///< Publish arm state
    ros::Publisher barometer_publisher;///< Publish Pressure, temperature and Altitude
    ros::Publisher motorpwm_publisher;///< Publishes output motor pwm
    ros::Publisher imu_publisher;///< Publishes raw imu
    ros::Publisher rpy_publisher;///< Publishes the quadcopter orientation from AHRS
    ros::Publisher gps_publisher;///< Publishes the gps data
    ros::Publisher linvel_publisher;///< Publishes the linear velocity from optical flow
    ros::Publisher windspeed_publisher;///< Publish windspeed and angle
};

#endif // ONBOARD_NODEHANDLER_H
