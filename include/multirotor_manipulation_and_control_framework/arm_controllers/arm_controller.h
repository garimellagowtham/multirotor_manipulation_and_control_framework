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
#ifndef ARM_CONTROLLER_H
#define ARM_CONTROLLER_H

#include <iostream>
#include <Eigen/Dense>
#include <arm_parsers/arm_parser.h>
#include <gcop/se3.h>

using namespace std;

/**
 * @brief A controller to grab objects. Uses the arm parser to set the desired pose
 */
class ArmController
{
public:
    /**
    * @brief Constructor
    *
    * @param parser   Parser instance used for sending commands to arm from controller
    */
    ArmController(ArmParser &parser): armbase_cam_transform_(Eigen::Matrix4d::Identity())
                                      , object_offset_transform_(Eigen::Matrix4d::Identity())
                                      , parser_(parser)

    {
    }

    /**
    * @brief Set the transforms needed for controller
    *
    * @param cam_armbase_transform    The transform of arm base in camera's frame
    * @param object_offset_transform  The transform of desired end effector pose in object frame (Default Identity)
    */
    void setTransform(Eigen::Matrix4d cam_armbase_transform, Eigen::Matrix4d object_offset_transform = Eigen::Matrix4d::Identity())
    {
      //Copy transforms over
      gcop::SE3::Instance().inv(armbase_cam_transform_,cam_armbase_transform);
      object_offset_transform_ = object_offset_transform;
    }

    /**
    * @brief Set the control to the arm. Should be called whenever new information about object is known
    *
    * @param cam_object_transform Transform of the object in the camera frame
    */
    virtual void setCtrl(const Eigen::Matrix4d &cam_object_transform)
    {
      //Find the transform of desired end effector Pose
      Eigen::Matrix4d armbase_end_effector_transform = armbase_cam_transform_*cam_object_transform*object_offset_transform_;
      parser_.setEndEffectorPose(armbase_end_effector_transform);
      return;
    }

protected:
    Eigen::Matrix4d armbase_cam_transform_;///< Transform from arm base frame to camera frame
    Eigen::Matrix4d object_offset_transform_;///< Offset Transform to where gripper should be in object frame
    ArmParser &parser_;///< Arm parser to send commands to the arm

};

#endif // ARM_CONTROLLER_H
