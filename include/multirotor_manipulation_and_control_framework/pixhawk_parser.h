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

#ifndef PIXHAWK_PARSER_H
#define PIXHAWK_PARSER_H

#include"quadcopter_parser.h"
#include <mavconn/interface.h>
#include "utils.h"

using namespace mavconn;

/**
* @brief This class implements the pixhawk mavlink interface to receive sensor data and send radio commands to the quadcopter
*/
class PixhawkParser : public QuadcopterParser
{
public:
  /**
   * @brief: Constructor
   *
   * @url: Pass the address to quadcopter. It can be a serial/udp or any other address
   */
  PixhawkParser(std::string url = "udp://@");

  bool takeoff();
  bool land();
  bool disarm();
  bool reset();
  bool calibrateImuBias();
  bool command(const ControlCommand &command);
  bool getSensorData(SensorData &data);

  void setThrustBounds(double thrust_min_newtons, double thrust_max_newtons);

protected:
  /**
  * @brief Receives Mavlink messages from serial/UDP ports
  *
  * @param message  Incoming mavlink msg
  * @param sysid    Incoming system's sysid
  * @param compid   Incoming system's compid
  */
  void receiveMavlinkMessage(const mavlink_message_t *message, uint8_t sysid, uint8_t compid);

  ////////////Helper Mavlink Publish Messages:
  /**
  * @brief Send parameter request to mavlink
  */
  inline void parameterRequest();
  /**
  * @brief Send data request to mavlink (imu/mag/gps etc)
  *
  * @param stream_id      ID of the data to be sent look into mavlink /pixhawk for definitions
  * @param status         start/stop will either start or stop the data stream
  * @param message_rate   Rate of the data in Hz
  */
  inline void dataRequest(uint8_t stream_id, bool status, uint16_t message_rate = 30);
  /**
  * @brief Send radio override commands. Zero radio command implies no override or clear override
  *
  * @param radio_commands Radio values to be sent
  */
  inline void sendRadio(uint16_t *radio_commands);

  /**
  * @brief Set the parameter on the target
  *
  * @param id                 ID is the name of the parameter should be 16 characters or less
  * @param parameter_value    Value of the parameter 
  * @param param_type         Type of the parameter as given by MAV_PARAM_TYPE in mavlink/.../common/common.h
  */
  inline void setParameter(std::string id, float parameter_value, MAV_PARAM_TYPE param_type = MAV_PARAM_TYPE_UINT8);


private:
  MAVConnInterface::Ptr gcs_link;///< Pointer to Mavlink Connector library
  bool heartbeat_received_;///< Checks if we are receiving heartbeat from the quadcopter continously
  uint8_t targetsys_id;///< Target System ID of the Quadcopter connecting
  uint8_t targetcomp_id;///< Target Comp ID of the Quadcopter
  uint16_t RC_TRIM[4]; ///< Trim values of the quadcopter
  uint16_t RC_MIN[4]; ///< Minimum values of the quadcopter
  uint16_t RC_MAX[4]; ///< Maximum values of the quadcopter
  double thrust_min_;///< Minimum thrust in Newtons can be provided by motors
  double thrust_max_;///< Maximum thrust in Newtons can be provided by motors
public:
  bool debug_enable_;///< Prints additional messages used for debugging
  // Auto Pilot Modes enumeration
  enum autopilot_modes {
      STABILIZE =     0,  // manual airframe angle with manual throttle
      ACRO =          1,  // manual body-frame angular rate with manual throttle
      ALT_HOLD =      2,  // manual airframe angle with automatic throttle
      AUTO =          3,  // fully automatic waypoint control using mission commands
      GUIDED =        4,  // fully automatic fly to coordinate or fly at velocity/direction using GCS immediate commands
      LOITER =        5,  // automatic horizontal acceleration with automatic throttle
      RTL =           6,  // automatic return to launching point
      CIRCLE =        7,  // automatic circular flight with automatic throttle
      LAND =          9,  // automatic landing with horizontal position control
      OF_LOITER =    10,  // deprecated
      DRIFT =        11,  // semi-automous position, yaw and throttle control
      SPORT =        13,  // manual earth-frame angular rate control with manual throttle
      FLIP =         14,  // automatically flip the vehicle on the roll axis
      AUTOTUNE =     15,  // automatically tune the vehicle's roll and pitch gains
      POSHOLD =      16,  // automatic position hold with manual override, with automatic throttle
      BRAKE =        17   // full-brake using inertial/GPS system, no pilot input
  };
};

#endif // PIXHAWK_PARSER_H
