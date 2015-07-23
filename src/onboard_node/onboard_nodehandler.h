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
#include <sensor_msgs/JointState.h>

#include <multirotor_manipulation_and_control_framework/multirotor_manipulator_state.h>
#include <multirotor_manipulation_and_control_framework/multirotor_manipulator_commands.h>
#include <multirotor_manipulation_and_control_framework/multirotor_quad_state.h>
#include <multirotor_manipulation_and_control_framework/WindSpeed.h>
#include <multirotor_manipulation_and_control_framework/MotorPwm.h>
#include <multirotor_manipulation_and_control_framework/Barometer.h>
#include <multirotor_manipulation_and_control_framework/BodyState.h>
#include <multirotor_manipulation_and_control_framework/QuadControllerGains.h>


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

typedef Eigen::Matrix<double, 7, 1> Vector7d;

public:
    OnboardNodeHandler(ros::NodeHandle &nh);
protected:
    void quadDataHandler(const QuadcopterParser::SensorData &sensor_data, uint16_t mask);
    void armDataHandler(const ArmParser::ArmSensorData &sensor_data);
    void guiMsgHandler(const multirotor_manipulator_commands::ConstPtr &gui_command);
    void stateChanged(const uint8_t &state, int id);
    void quadControlTimer(const ros::TimerEvent& event_data);
    void markerCallback(const geometry_msgs::Pose::ConstPtr &pose);
    void jointCallback(const sensor_msgs::JointState::ConstPtr &command_state);
    void endeffectorCallback(const geometry_msgs::Pose::ConstPtr &pose);
    void quadGoalCallback(const BodyState::ConstPtr &state);
    void quadGainsCallback(const QuadControllerGains::ConstPtr &gains);

protected:
    enum State{
      ENABLED=0,///< Enabled
      DISABLED=1,///< Disabled
      CRITICAL=2///< Critical
    };
    inline bool bitcheck(uint16_t input, int shift)
    {
      return (input & (1<<shift));
    }
protected:
    ros::NodeHandle nh_;///< Internal nodehandle to publish and subscribe

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

    ros::Subscriber quad_goal_subscriber;///< Subscribe to target pose of quadcopter
    ros::Subscriber arm_joint_subscriber;///< Subscribe to arm joint commands
    ros::Subscriber arm_endeffector_subscriber;///< Subscribe to arm end effector commands
    ros::Subscriber marker_subscriber;///< Subscribe to marker/object pose for arm controller
    ros::Subscriber quad_gains_subscriber;///< Subscribe to the gains for quad controller

    ros::Timer quad_control_timer_;///< Timer to control the quadcopter

    State quad_controller_state_;///< State of the positioncontroller
    State arm_controller_state_;///< State of the arm controller

    /**
      * @brief If arm state is coupled with quad state.
      *  For eg. if quadcopter controller should not be enable unless arm
      *  arm is powered on
      */
    bool check_armstate;

    double control_loop_period_;///< Loop timer period in sec

    gcop::SE3 &se3_;///< SE3 instance
};

#endif // ONBOARD_NODEHANDLER_H
