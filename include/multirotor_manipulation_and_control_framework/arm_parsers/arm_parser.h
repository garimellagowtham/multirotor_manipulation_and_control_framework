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
#ifndef ARM_PARSER_H
#define ARM_PARSER_H

#include <iostream>
#include <fstream>
#include <vector>
#include <Eigen/Dense>
#include <utils/utils.h>
#include <boost/signal.hpp>
#include <stdexcept>

using namespace std;

/**
* @brief This class provides an abstract interface to control multi DOF arms. 
* Subclasses should provide functions for setting and getting joint angles/ velocities.
*/
class ArmParser{
public:
  /**
    * @brief Stores arm data. Add more entities like voltage etc later
    */
  struct ArmSensorData{
    std::vector<double> joint_angles_;///< Feedback Joint Angles
    std::vector<double> joint_velocities_;///< Feedback Joint Velocities
  };
public:
  const ArmSensorData &sensor_data;///< Arm feedback data for reading
  boost::signal<void (const ArmSensorData&)> signal_feedback_received_;///< Signal that arm feedback has been updated
public:
  ArmParser(): log_enable_(false), sensor_data(sensor_data_)
  {
  }

  /**
    * @brief Virtual Destructor
    */
  virtual ~ArmParser()
  {
  }
  /**
  * @brief Sets the joint angles and velocities of the arm
  *
  * @param joint_angles       Desired Joint Angles (unit: rad) should be between [-pi to pi]
  * @param joint_velocities   Desired Joint Velocities (If not provided will leave at previous/default values) (unit: rad/s)
  *
  * @return true if successfully set false otherwise
  */
  virtual bool setAngles(const vector<double> &joint_angles, const vector<double> *joint_velocities=0)=0;
  /**
  * @brief Sets the End effector at Desired Pose. For lower DOF arms, it sets the projection of the desired pose
  *
  * @param end_effector_pose End Effector Pose in the local base frame
  *
  * @return  true if successfully set
  */
  virtual bool setEndEffectorPose(const Eigen::Matrix4d &end_effector_pose)=0;
  /**
  * @brief Powers Motors on an arm on/off
  *
  * @param state Input to switch the motors on(true)/off(false)
  *
  * @return  true if successful
  */
  virtual bool powerMotors(bool state)=0;

  /**
  * @brief Creates files for logging and enables log status
  *
  * @param log_directory Input Directory where  log files are created
  */
  virtual void enableLog(string log_directory)//Default does not create a log directory etc
  {
    //Default is empty no log
  }
  /**
  * @brief Set the state of the gripper
  *
  * @param value Gripper value Depends on the arm
  *
  * @return true if success
  */
  virtual bool grip(double value)=0;
  /**
  * @brief Disable the log status and close all log files
  */
  void disableLog()
  {
    log_enable_ = false;
    command_log_file_.close();
    feedback_log_file_.close();
  }

protected:
  bool log_enable_;///< Enable Log
  ofstream command_log_file_;///< Command Log file
  ofstream feedback_log_file_;///< Feedback of actual angles log file
  ArmSensorData sensor_data_;///< Arm feedback
};
#endif
