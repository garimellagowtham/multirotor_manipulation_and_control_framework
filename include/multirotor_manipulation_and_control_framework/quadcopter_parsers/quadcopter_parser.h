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

#ifndef QUADCOPTER_PARSER_H
#define QUADCOPTER_PARSER_H
#include <fstream>
#include <iostream>

#include <utils/utils.h>

#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Vector3.h>

#include <std_msgs/Float32.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>

#include <boost/signal.hpp>

using namespace std;

/**
* @brief  This class is for receiving sensor data from various types of quadcopters and send radio override commands. 
* This is an abstract library and provides virtual functions to be implemented by subclasses
* 
* The orientation of the quadcopter should be in NWU format (forward x, left y, up z)
*
*/
class QuadcopterParser
{
public:
  struct ControlCommand{
    double command_roll;///< Commanded roll in radians
    double command_pitch;///< Commanded pitch in radians
    double command_rate_yaw;///< Commanded yaw in radians
    double command_thrust;///< Commanded thrust between 0 and command_max_thrust
  };
  /**
    * @brief Sensor data obtained from Quadcopter
    *  As the data is received a trigger in terms of boost signal is triggered
    *  To know what part of data is received, look at the mask in the triggered function:
    *  The mask is created with each bit representing different data types
    *  mask[0]: battery_volts, quadstate, timestamp, armed
    *  mask[1]: rpydata
    *  mask[2]: magdata
    *  mask[3]: pressure, temperature, altitude
    *  mask[4]: wind_speed, wind_angle
    *  mask[6]: motorpwm
    *  mask[7]: linvel, linacc
    *  mask[8]: gps_raw
    *
    *  Only the bits which are updated in the signal are set to 1
    */
  struct SensorData{
    double battery_volts;///< Battery volts in V
    std::string quadstate;///< Information about quadcopter state (Flying, Landed etc) in Human readable format
    geometry_msgs::Vector3 rpydata;///< Roll pitch yaw data in radians
    geometry_msgs::Vector3 magdata;///< Magnetometer data;
    double pressure;///< Pressure from barometer (KPa)
    double temperature;///< temperature from temp sensor (degree C)
    double  wind_speed;///< Wind speed (m/s)
    double  wind_angle;///< wind angle (degrees)
    double altitude; ///< estimated altitude
    double motorpwm[8];///< motorpwm values not all of them may be used
    geometry_msgs::Vector3 linvel;///< Linear velocity of quadcopter
    geometry_msgs::Vector3 linacc;///< Linear acceleration of quadcopter
    long gps_raw[3];///< GPS Position in Latitude, Longitude and Altitude
    double timestamp;///< timestamp from drone
    bool armed;///< Whether the quadcopter is ready to fly or not
    SensorData(): battery_volts(0)
    ,quadstate("NONE")
    ,pressure(0), temperature(0), wind_speed(0), wind_angle(0)
    ,altitude(0), timestamp(0)
    ,armed(false)
    ,motorpwm()//set to zero
    {
      rpydata.x = 0; rpydata.y = 0; rpydata.z = 0;
      magdata.x = 0; magdata.y = 0; magdata.z = 0;
      linvel.x = 0; linvel.y = 0; linvel.z = 0;
      linacc.x = 0; linacc.y = 0; linacc.z = 0;
    }
  };

public:
  double mass;///< Mass of the Quadcopter
  geometry_msgs::Vector3 body_dimensions;///< body dimensions of quadcopter along principal axis assuming its a box
  bool open_error;///< Error while opening the hardware
  const SensorData &sensor_data;///< Sensor data for reading purposes only Cannot write to it
  boost::signal<void (const SensorData &, uint16_t mask)> signal_sensor_update_;///< sensor data updated signal can be used to trigger callbacks etc

public: 
  /**
  * @brief Constructor
  *
  */
  QuadcopterParser():log_enable_(false), mass(1.0), open_error(false), sensor_data(sensor_data_){
    body_dimensions.x = 1.0;//Default values for body dimensions
    body_dimensions.y = 1.0;
    body_dimensions.z = 1.0;
  }//Constructor

  /**
  * @brief Takeoff quadcopter (Arms the motors and ready to receive motor override commands)
  *
  * @return  Success/Failure
  */
  virtual bool takeoff()=0;
  /**
  * @brief Land quadcopter from certain height
  *
  * @return  Success/Failure
  */
  virtual bool land()=0;
  /**
  * @brief Disarm (Stop Motors) Quadcopter
  *
  * @return  Success/Failure
  */
  virtual bool disarm()=0;
  /**
  * @brief Reset sensors and recalibrate imus etc
  *
  * @return  Success/Failure
  */
  virtual bool reset()=0;
  /**
  * @brief Calibrate gyro biases of the quadcopter based on sample data
  *
  * @return  Success/Failure
  */
  virtual bool calibrateImuBias()=0;
  /**
  * @brief Send roll pitch yaw and thrust commands to quadcopter
  *
  * @param command Control command to be sent to quadcopter
  *
  * @return  Success/Failure
  */
  virtual bool command(const ControlCommand &command)=0;

  /**
  * @brief Virtual Destructor
  */
  virtual ~QuadcopterParser()
  {
  }

  /**
  * @brief Creates files in the log directory and fills headers
  *
  * @param log_directory  Log directory where log files are created. Should be writable
  */
  void enableLog(string log_directory)
  {
    cmd_file_.open((log_directory+"/cmd.dat").c_str());
    servo_file_.open((log_directory+"/servo.dat").c_str());
    imu_file_.open((log_directory+"/imu.dat").c_str());

    cmd_file_.precision(10);
    servo_file_.precision(10);
    imu_file_.precision(10);

    cmd_file_<<"#Time\t Roll \t Pitch \t Yaw \t Thrust"<<endl;
    servo_file_<<"#Time\t SERVO_1\t SERVO_2\t SERVO_3\t SERVO_4"<<endl;
    imu_file_<<"#Time\t Roll \t Pitch \t Yaw"<<endl;
    log_enable_ = true;
  }

  /**
  * @brief Disable log status and close files
  */
  void disableLog()
  {
    log_enable_ = false;
    imu_file_.close();
    cmd_file_.close();
    servo_file_.close();
  }

  inline void printSensorData()
  {
    //Print all the Sensor Data:
    printf("Battery Voltage: %2.2f\tTemperature: %2.2f\t\nPressure: %2.4f\tWindspeed: "
           "%2.2f\tAltitude: %2.2f\t\nRoll: %2.2f\tPitch %2.2f\tYaw %2.2f\nMagx: %2.2f"
           "\tMagy %2.2f\tMagz %2.2f\naccx: %2.2f\taccy %2.2f\taccz %2.2f\nvelx: %2.2f\tvely %2.2f\tvelz "
           "%2.2f\t\nMass: %2.2f\tTimestamp: %2.2f\t\nServo:%f,%f,%f,%f,%f,%f,%f,%f\nQuadState: %s\n",
           sensor_data_.battery_volts
           ,sensor_data_.temperature,sensor_data_.pressure
           ,sensor_data_.wind_speed, sensor_data_.altitude
           ,sensor_data_.rpydata.x*(180/M_PI),sensor_data_.rpydata.y*(180/M_PI),sensor_data_.rpydata.z*(180/M_PI)//IMU rpy angles
           ,sensor_data_.magdata.x,sensor_data_.magdata.y,sensor_data_.magdata.z
           ,sensor_data_.linacc.x,sensor_data_.linacc.y,sensor_data_.linacc.z
           ,sensor_data_.linvel.x,sensor_data_.linvel.y,sensor_data_.linvel.z
           ,mass, sensor_data_.timestamp
           ,sensor_data_.motorpwm[0],sensor_data_.motorpwm[1],sensor_data_.motorpwm[2],sensor_data_.motorpwm[3]
           ,sensor_data_.motorpwm[4],sensor_data_.motorpwm[5],sensor_data_.motorpwm[6],sensor_data_.motorpwm[7]
           ,sensor_data_.quadstate.c_str());
           //"%2.2f\nposx: %2.2f\tposy: %2.2f\tposz: %2.2f\nvrpnr: %2.2f\tvrpnp: %2.2f\tvrpny: %2.2f\nErrorr: "
           //"%2.2f\tErrorrp: %2.2f\tErrory: %2.2f\nresr: %2.2f\tresp: %2.2f\tresy: %2.2f\trest: %2.2f\nbias_roll: "
           //"%2.2f\tbias_pitch: %2.2f\tbias_yaw: %2.2f\nObjx: %2.2f\tObjy: %2.2f\tObjz: %2.2f\t\nTipx: %2.2f\tTipy:"
           //"%2.2f\tTipz: %2.2f\t\nMass: %2.2f\tTimestamp: %2.2f\t\nQuadState: %s",
  }

protected:
  SensorData sensor_data_;///< Data from quadcopter
  ofstream cmd_file_;///< Command log file
  ofstream servo_file_;//Raw servo pwm log file
  ofstream imu_file_;//Imu data log file
  bool log_enable_;///< Enable/Disable logging
};
#endif //QuadcopterParser_H

