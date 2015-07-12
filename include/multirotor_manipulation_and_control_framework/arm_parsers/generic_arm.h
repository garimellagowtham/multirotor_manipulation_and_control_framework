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

#ifndef GENERIC_ARM_H
#define GENERIC_ARM_H

#include <ros/ros.h>
#include "arm_parser.h"
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/Pose.h>
#include <gcop/so3.h>

using namespace std;

/**
 * @brief  This class provides ros interface for generic ndof arms through ros topics
 *
 * It produces sensor_msgs/Joint msg to the arm and receives sensor_msgs/Joint msg 
 * to receive the current position of the arm
 * The class publishes following topics:
 *  - command_joint_states (sensor_msgs/JointState): Joint states commanded
 *  - ee_pose (geometry_msgs/Pose): Commanded End Effector Pose of the arm
 *  - ee_gripper(std_msgs/Float64): Gripper value to be set
 *  - power_motors(std_msgs/Bool): Powers the motors on/off
 *  .
 * The class subscribes to following topics:
 *  - /joint_states (sensor_msgs/JointState): Actual Joint State msg coming from hardware etc
 *  
 */
class GenericArm : public ArmParser
{
public:
  /**
  * @brief Constructor for generic arm
  *
  * @param nh             NodeHandle used to create publishers and subscribers
  * @param njoints        number of joints on the arm
  */
  GenericArm(ros::NodeHandle &nh, int njoints = 7);
  virtual bool setAngles(const vector<double> &joint_angles, const vector<double> *joint_velocities = 0);
  virtual bool setEndEffectorPose(const Eigen::Matrix4d &end_effector_pose);
  virtual bool powerMotors(bool state);
  virtual void enableLog(string log_directory);
  virtual bool grip(double value);


protected:
  ros::Publisher joint_pub_;///< Desired Joint angle publisher
  ros::Publisher gripper_pub_;///< Commanded gripper position publisher
  ros::Publisher end_effector_pub_;///< Commanded end effector pose publisher
  ros::Publisher power_motors_pub_;///< Power motors on/off publisher
  ros::Subscriber joint_sub_;///< Joint feedback subscriber
  int number_of_joints_;///< Number of joints in the arm
  gcop::SO3 &so3;///< Instance of so3 to perform operations on rotation matrices

protected:
  /**
  * @brief Callback function for receiving Joint State msg
  *
  * @param joint_state
  */
  void stateReceived(const sensor_msgs::JointState::ConstPtr &joint_state);
};

#endif // GENERIC_ARM_H
