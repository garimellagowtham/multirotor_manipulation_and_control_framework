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

#ifndef STATE_ESTIMATOR_H
#define STATE_ESTIMATOR_H

#include <gcop/predictor.h>
#include <gcop/corrector.h>
#include <gcop/body3d.h>
#include <Eigen/Dense>
#include <iostream>
#include <quadcopter_parsers/quadcopter_parser.h>

using namespace std;

/**
* @brief This class provides an abstract interface for writing different state estimators for quadcopter
* The subsclasses should implement the the predictState function and are free to use any sensor combinations 
* 
* For the controllers to work, the output state should have observable position, velocity and yaw
*/
class StateEstimator
{
public:
 enum State{
    ENABLED=0,///< Armed and Enabled
    DISABLED=1,///< Disarmed and Disabled
    CRITICAL=2///< Crash or some emergency
  };

public:
  const State &state;///< Basic state used in statemachine 0: ENABLED, 1: DISABLED, 2: CRITICAL
  boost::signal<void (const uint8_t &, int id)> signal_state_update_;///< state updated
public:
  /**
   * Constructor creates a body3d system.
   *
   * @param body_dimensions Dimensions of the body
   */
  StateEstimator(QuadcopterParser &quad_parser): quad_parser_(quad_parser)
                                                 , state(state_)
                                                 , state_(DISABLED)
  {
  }

  //Virtual Functions:
  /**
  * @brief Predict the state of the system
  *
  * @param state This is filled with the current state at the time of calling
  */
  virtual void predictState(gcop::Body3dState &state)=0;

protected:
  QuadcopterParser &quad_parser_;///< Reference to the quadcopter parser to obtain sensor data
  State state_;///< Personal modifiable reference to state
};

#endif // STATE_ESTIMATOR_H
