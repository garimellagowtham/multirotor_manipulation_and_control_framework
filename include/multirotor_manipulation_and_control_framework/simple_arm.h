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

#ifndef SIMPLE_ARM_H
#define SIMPLE_ARM_H

#include "arm_parser.h"
#include "dynamixelsdk/dynamixel.h"
#include <gcop/se3.h>

// Control table address
#define P_GOAL_POSITION_L	30
#define P_GOAL_POSITION_H	31
#define P_PRESENT_POSITION_L	36
#define P_PRESENT_POSITION_H	37
#define P_PRESENT_SPEED_L	38
#define P_PRESENT_SPEED_H	39
#define P_MOVING		46
#define P_ID 3
#define P_MOVING_SPEED_L 32
#define P_MOVING_SPEED_H 33
#define P_TORQUE_ENABLE 24

// Defult setting
#define DEFAULT_BAUDNUM 	 34
#define DEFAULT_ID		1

#define NOFJOINTS 2
#define ANGRES 0.088
#define ANGVELRES 0.011938

using namespace std;
using namespace dynamixelsdk;

/** @brief This is a specific implementation of arm parser for 2 Degree of
 * Freedom arm with a gripper. The motors are dynamixel MX-28 model
 *
 * This class directly talks to the motors through usb2dynamixel.
 * Coordinate System for arm:
 * Angle 0,0 corresponds to both links in z+ direction
 * The positive direction of rotation is counterclockwise
 * The first angle corresponds to shoulder and second angle corresponds to elbow
 * The base origin is z up and x forward
 * The maximum angles for arm are: [In Degrees]
 *
 * The bounds for arm are as follows:
 *  - angle[0] - -M_PI/2 , M_PI/2
 *  - angle[1] - -M_PI, M_PI (With limitations depending on angle[0] )
 *  - angle_vel[0,1] - 2 rad/s
 */
class SimpleArm : public ArmParser
{
public:
  /**
  * @brief Constructor
  *
  * @param deviceIndex   Opens ttyUSB[deviceIndex] or ttyACM[deviceIndex] whichever is available (tries ttyUSB first)
  * @param baudrate      Baudrate of device by default is 57600
  */
  SimpleArm(int deviceIndex, uint16_t baudrate = 57600);
  virtual bool setAngles(const vector<double> &joint_angles, const vector<double> *joint_velocities=0);
  virtual bool setEndEffectorPose(const Eigen::Matrix4d &end_effector_pose);
  virtual bool powerMotors(bool state);
  virtual bool getState(vector<double> &joint_angles, vector<double> *joint_velocities=0);
  virtual void enableLog(string log_directory);
  virtual bool grip(double value);
protected:
  /**
  * @brief Helper functions for communication
  *
  * @param CommStatus   status received from dynamixel library
  */
  inline void PrintCommStatus(int CommStatus)
  {
    switch(CommStatus) //Will change the #defines into enum TODO
    {
    case COMM_TXFAIL:
      printf("COMM_TXFAIL: Failed transmit instruction packet!\n");
      break;

    case COMM_TXERROR:
      printf("COMM_TXERROR: Incorrect instruction packet!\n");
      break;

    case COMM_RXFAIL:
      printf("COMM_RXFAIL: Failed get status packet from device!\n");
      break;

    case COMM_RXWAITING:
      printf("COMM_RXWAITING: Now recieving status packet!\n");
      break;

    case COMM_RXTIMEOUT:
      printf("COMM_RXTIMEOUT: There is no status packet!\n");
      break;

    case COMM_RXCORRUPT:
      printf("COMM_RXCORRUPT: Incorrect status packet!\n");
      break;

    default:
      printf("This is unknown error code!\n");
      break;
    }
  }

  /**
  * @brief Print error bit of status packet
  */
  inline void PrintErrorCode()
  {
    if(dynamixelsdk::dxl_get_rxpacket_error(ERRBIT_VOLTAGE) == 1)
      printf("Input voltage error!\n");

    if(dynamixelsdk::dxl_get_rxpacket_error(ERRBIT_ANGLE) == 1)
      printf("Angle limit error!\n");

    if(dynamixelsdk::dxl_get_rxpacket_error(ERRBIT_OVERHEAT) == 1)
      printf("Overheat error!\n");

    if(dynamixelsdk::dxl_get_rxpacket_error(ERRBIT_RANGE) == 1)
      printf("Out of range error!\n");

    if(dynamixelsdk::dxl_get_rxpacket_error(ERRBIT_CHECKSUM) == 1)
      printf("Checksum error!\n");

    if(dynamixelsdk::dxl_get_rxpacket_error(ERRBIT_OVERLOAD) == 1)
      printf("Overload error!\n");

    if(dynamixelsdk::dxl_get_rxpacket_error(ERRBIT_INSTRUCTION) == 1)
      printf("Instruction code error!\n");
  }
protected:
  //Arm Inverse Kinematics and Forward Kinematics
  /**
  * @brief Forward Kinematics
  *
  * @param end_effector_pose  End Effector Pose (only uses the x and z positions and pitch is updated)
  * @param joint_angles       Joint Angles input
  *
  * @return  True always
  */
  bool forwardKinematics(Eigen::Matrix4d &end_effector_pose, const vector<double> &joint_angles);

  /**
  * @brief  Arm Inverse Kinematics
  *
  * @param end_effector_pose  Input End effector pose (only uses x and z positions to find both lower elbow and upper elbow solutions)
  * @param joint_angles       Joint angles corresponding to end effector pose
  *
  * @return Positive value if in the workspace and negative if out of the workspace
  */
  double inverseKinematics(const Eigen::Matrix4d &end_effector_pose, vector<vector<double> > &joint_angles);

protected:
  uint16_t arm_id_[NOFJOINTS];///< Ids of Dynamixels to talk to
  uint16_t arm_max_angles_[NOFJOINTS];///< Maximum joint angles
  uint16_t arm_min_angles_[NOFJOINTS];///< Minimum joint angles
  uint16_t arm_offset_angles_[NOFJOINTS];///< Default values of arm joints
  uint16_t arm_default_velocities_[NOFJOINTS];///< Default velocities of the arm joints
  uint16_t arm_max_angle_velocities_[NOFJOINTS];///< Max angular velocites
  double l1, l2, x1;//Physical lengths of the links and offset between link1 and link2
  gcop::SE3 &se3;///< SE3 Instance
};

#endif // SIMPLE_ARM_H
