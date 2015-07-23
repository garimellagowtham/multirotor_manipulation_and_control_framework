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

#ifndef TRAJECTORYTRACKINGCONTROLLER_H
#define TRAJECTORYTRACKINGCONTROLLER_H

#include "tf/transform_datatypes.h"
#include <quadcopter_parsers/quadcopter_parser.h>
#include "fstream"
#include "iostream"
#include <gcop/se3.h>
#include <gcop/body3dmanifold.h>
#include <Eigen/Geometry>

#define PI 3.14159

using namespace std;

/**
* @brief This class provides a nonlinear controller using feedback linearization on angles of the quadcopter
* The controller integrates the thrust to find the external forces. So start with smaller mass of the quadcopter
*/
class TrajectoryTrackingController 
{
public:
  struct ControllerGains {
    double kpr;///< Proportional gain on roll and pitch
    double kpy;///< Proportional gain on yaw
    double kpt;///< Proportional gain on thrust
    double kdr;///< Derivative gain on roll and pitch
    double kdt;///< Derivative gain on thrust
    double kit;///< Integral gain on thrust
    double kiy;///< Integral gain on yaw
  };
  struct ControllerBias {
    Eigen::Vector3d force_bias;///< Force bias in x, y, z directions
    double rateyaw_bias;///<Torque due to mismatch of motor velocities
  };
protected:
  gcop::Body3dState goal_state_;///< Desired goal state
  Eigen::Vector3d goal_rpy_orientation_;///< Desired orientation computed from goal state
  ControllerGains controller_gains_; ///< Gains for the controller
  ControllerBias controller_bias_;///< Bias estimated by the controller
  QuadcopterParser::ControlCommand controller_command_bound_;///< Bound on the control command that can be produced
  ofstream log_file_;///< File for logging controller data
  bool log_enable_;///< Enable/Disable logging
  gcop::SO3 &so3;///< SE3 Instance
  const double bound_velocity_difference_;///< Bound on the velocity difference in the controller in m/s
  bool integrator_enable_;///< Integrator enable
  QuadcopterParser &parser_;///< Const reference to parser for sending commands to quadcopter

private:
  /**
  * @brief Bind the input value symmetrically
  *
  * @param x      Input value x
  * @param xbound Bound x
  *
  * @return Output value x
  */
  inline double bind(double x, double xbound)
  {
    if( x > xbound)
      return xbound;
    else if(x < -xbound)
      return -xbound;
    else
      return x;
    // just as precaution
    return x;
  }

public:
  /**
  * @brief Constructor
  * @param parser   Reference of a quadcopter parser is needed to send quadcopter commands
  */
  TrajectoryTrackingController(QuadcopterParser &parser);
  /**
  * @brief Set the rc override command
  *
  * @param result_command   Output roll pitch yaw and thrust command
  * @param filtered_state   Input State from the estimator
  *
  * @return  False if input state has bad values
  */
  virtual bool setCtrl(const gcop::Body3dState &filtered_state);
  /**
  * @brief Sets the gains for the controller after some prechecks
  *
  * @param gains Input gains for the controller
  */
  void setGains(const ControllerGains &gains);
  /**
  * @brief Set the goal of the quadcopter
  *
  * @param goal_state Input goal state of the quadcopter
  */
  void setGoal(const gcop::Body3dState &goal_state);
  /**
  * @brief Get the current goal of the quadcopter
  *
  * @param goal_state Output Goal state of the quadcopter
  */
  void getGoal(gcop::Body3dState &goal_state);
  /**
  * @brief Set the bound on the commanded roll, pitch, yaw, thrust
  *
  * @param command_bound  Input Command bound
  *
  * @return True if it is set
  */
  bool setBound(const QuadcopterParser::ControlCommand &command_bound);
  /**
  * @brief Set the initial controller bias (external force and yaw bias)
  *
  * @param controller_bias Input controller bias
  *
  * @return True if it is set
  */
  bool setBias(const ControllerBias &controller_bias);
  /**
  * @brief Enable the integration to find controller bias
  */
  void enableIntegrator()
  {
    integrator_enable_ = true;
  }
  /**
  * @brief Disable integration to find controller bias
  */
  void disableIntegrator()
  {
    integrator_enable_ = false;
  }
  /**
  * @brief Create Log files in the log directory
  *
  * @param log_directory  Input Log directory Should be writable
  */
  void enableLog(string log_directory)
  {
    log_file_.open((log_directory+"/ctrl.dat").c_str());
    log_file_.precision(10);
    log_file_<<"#curr_Time\t dx[0]\t dx[1]\t dx[2]\t ddx[0]\t ddx[1]\t ddx[2]\t Fext[0]\t Fext[1]\t Fext[2]\t rpycmd[0]\t rpycmd[1]\t rpycmd[2]"<<endl;
    log_enable_ = true;
  }

  /**
  * @brief Disable the log status and close the log files
  */
  void disableLog()
  {
    log_enable_ = false;
    log_file_.close();
  }
};
#endif

