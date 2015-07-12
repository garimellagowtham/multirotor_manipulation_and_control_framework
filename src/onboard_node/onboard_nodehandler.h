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
    ros::NodeHandle nh_;///< Internal nodehandle to publish and subscribe
    boost::shared_ptr<ArmParser> arm_parser_;///< Parser to talk to the manipulator
    boost::shared_ptr<QuadcopterParser> quad_parser_;///< Parser to talk to the quadcopter
    boost::shared_ptr<StateEstimator> state_estimator_;///< Estimator to get the state of the quadcopter
    boost::shared_ptr<TrajectoryTrackingController> quad_controller_;///< Controller to track trajectories
    boost::shared_ptr<ArmController> arm_controller_;///< Controller to grab objects
};

#endif // ONBOARD_NODEHANDLER_H
