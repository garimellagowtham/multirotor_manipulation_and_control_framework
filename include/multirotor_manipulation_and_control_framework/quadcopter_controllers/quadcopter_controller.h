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

#ifndef QUADCOPTERCONTROLLER_H
#define QUADCOPTERCONTROLLER_H

#include <quadcopter_parsers/quadcopter_parser.h>
#include "fstream"
#include "iostream"
#include <gcop/se3.h>
#include <gcop/body3dmanifold.h>
#include <Eigen/Geometry>

class QuadcopterController 
{
  public:
    const gcop::Body3dState &goal_state;///< Desired goal state only for reading
  public: 
    /**
     * @brief Constructor
     * @param parser   Reference of a quadcopter parser is needed to send quadcopter commands
     */
    virtual void initialize(QuadcopterParser &parser) = 0;
    /**
     * @brief Set the control command to quadcopter
     *
     * @param result_command   Output roll pitch yaw and thrust command
     * @param filtered_state   Input State from the estimator
     *
     * @return  False if input state has bad values
     */
    virtual bool setCtrl(const gcop::Body3dState &filtered_state)=0;
    /**
    * @brief Set the parameter for the controller
    *
    * @param name Name of the parameter
    * @param value Value taken by the parameter
    *
    * @return  True if parameter is found and set correctly
    */
    virtual bool setParam(const char *name, double value)=0;
    /**
     * @brief Create Log files in the log directory
     *
     * @param log_directory  Input Log directory Should be writable
     */
    virtual void enableLog(string log_directory)=0;
  protected:
    gcop::Body3dState goal_state_;///< Desired goal state
    ofstream log_file_;///< File for logging controller data
    bool log_enable_;///< Enable/Disable logging
  protected:
    /**
    * @brief Constructor
    *
    */
    QuadcopterController(): goal_state(goal_state_), log_enable_(false){}
  public:
    /**
     * @brief Set the goal of the quadcopter
     *
     * @param goal_state Input goal state of the quadcopter
     */
    virtual void setGoal(const gcop::Body3dState &goal_state)
    {
      goal_state_ = goal_state;//Copy goal state
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
