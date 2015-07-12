#include "pixhawk_parser.h"

using namespace mavconn;
using namespace std;

PixhawkParser::PixhawkParser(std::string url): QuadcopterParser()
                                              , thrust_min_(0)
                                              , thrust_max_(10)
                                              , targetsys_id(255)
                                              , targetcomp_id(MAV_COMP_ID_SYSTEM_CONTROL)
                                              , debug_enable_(true)
{
  try {
      gcs_link = MAVConnInterface::open_url(url, 255, 110);
  }
  catch (mavconn::DeviceError &ex) {
      cerr<<"GCS: "<<ex.what()<<endl;
      this->sensor_data_.quadstate = "ERROR";
      throw(std::invalid_argument("Invalid URL"));
  }

  //Initialize RC IDs        and RC Trims:
  RC_MIN[0] = 1104; RC_MIN[1] = 1101; RC_MIN[2] = 1046; RC_MIN[3] = 1101;
  RC_TRIM[0]= 1514; RC_TRIM[1]= 1514; RC_TRIM[2]= 1048; RC_TRIM[3]= 1512;
  RC_MAX[0] = 1928; RC_MAX[1] = 1929; RC_MAX[2] = 1924; RC_MAX[3] = 1925;


  //Initialize body dimensions:
  body_dimensions.x = 0.5;
  body_dimensions.y = 0.5;
  body_dimensions.z = 0.2;
  mass = 0.5;

  //Set a connect function to receive data when it is available from mavlink
  gcs_link->message_received.connect(boost::bind(&PixhawkParser::receiveMavlinkMessage, this, _1, _2, _3));
}

inline void PixhawkParser::sendRadio(uint16_t *radio_commands)
{
  mavlink_message_t mavlink_msg;
  //construct rc override command
  mavlink_rc_channels_override_t override_msg;
  override_msg.target_system = targetsys_id;
  override_msg.target_component = targetcomp_id;

  //Controller should ensure that the commanded angles are between -pi to pi
  override_msg.chan1_raw = radio_commands[0];
  override_msg.chan2_raw = radio_commands[1];
  override_msg.chan3_raw = radio_commands[2];
  override_msg.chan4_raw = radio_commands[3];
  override_msg.chan5_raw = radio_commands[4];
  override_msg.chan6_raw = radio_commands[5];
  override_msg.chan7_raw = radio_commands[6];
  override_msg.chan8_raw = radio_commands[7];

  mavlink_msg_rc_channels_override_encode(gcs_link->get_system_id(),gcs_link->get_component_id(),&mavlink_msg,&override_msg);
  gcs_link->send_message(&mavlink_msg);
}

inline void PixhawkParser::parameterRequest()
{
  //publish param_list message to get the parameters:
  mavlink_message_t mavlink_msg;
  mavlink_param_request_list_t mavparamreq;
  mavparamreq.target_system = targetsys_id;
  mavparamreq.target_component = targetcomp_id;
  //encode the mavlink_param_request_list_t into mavlink_message_t
  mavlink_msg_param_request_list_encode(gcs_link->get_system_id(),gcs_link->get_component_id(),&mavlink_msg,&mavparamreq);
  gcs_link->send_message(&mavlink_msg);
}

inline void PixhawkParser::dataRequest(uint8_t stream_id, bool status, uint16_t message_rate)
{
  mavlink_message_t mavlink_msg;
  //construct command
  mavlink_request_data_stream_t mavdatastreamreq;
  mavdatastreamreq.req_message_rate = message_rate; //Hz
  mavdatastreamreq.req_stream_id = stream_id;
  mavdatastreamreq.start_stop = status?1:0;//Enable if status is true
  mavdatastreamreq.target_system = targetsys_id;
  mavdatastreamreq.target_component = targetcomp_id;
  mavlink_msg_request_data_stream_encode(gcs_link->get_system_id(),gcs_link->get_component_id(),&mavlink_msg,&mavdatastreamreq);
  gcs_link->send_message(&mavlink_msg);
}

inline void PixhawkParser::setParameter(std::string id, float parameter_value, MAV_PARAM_TYPE param_type)
{
  mavlink_message_t mavlink_msg;
  mavlink_param_set_t parameter_msg;
  //construct command
  parameter_msg.target_system = targetsys_id;
  parameter_msg.target_component = targetcomp_id;
  strcpy(parameter_msg.param_id, id.c_str());
  parameter_msg.param_value = parameter_value;
  parameter_msg.param_type = param_type;
  mavlink_msg_param_set_encode(gcs_link->get_system_id(), gcs_link->get_component_id(),&mavlink_msg,&parameter_msg);
  gcs_link->send_message(&mavlink_msg);
}

bool PixhawkParser::takeoff()
{
  uint16_t arm_radio_values[8] = {RC_TRIM[0], RC_TRIM[1], RC_MIN[2], RC_MAX[3], 0, 0, 0, 0};
  for(int count = 0;count < 30;count++)//3Sec
  {
    PixhawkParser::sendRadio(arm_radio_values);
    usleep(100000);
  }
  uint16_t default_radio_values[8] = {RC_TRIM[0], RC_TRIM[1], RC_MIN[2], RC_TRIM[3], 0, 0, 0, 0};
  PixhawkParser::sendRadio(default_radio_values);
  ////#TODO: Find pressure from sensordata
  return true;
}

bool PixhawkParser::disarm()
{
  uint16_t disarm_radio_values[8] = {RC_TRIM[0], RC_TRIM[1], RC_MIN[2], RC_MIN[3], 0, 0, 0, 0};
  for(int count = 0;count < 80;count++)//3Sec
  {
    PixhawkParser::sendRadio(disarm_radio_values);
    usleep(50000);
  }
  uint16_t default_radio_values[8] = {RC_TRIM[0], RC_TRIM[1], RC_MIN[2], RC_TRIM[3], 0, 0, 0, 0};
  PixhawkParser::sendRadio(default_radio_values);
  return true;
}

void PixhawkParser::setThrustBounds(double thrust_min_newtons, double thrust_max_newtons)
{
  if(thrust_min_newtons > 0)
  {
    thrust_min_ = thrust_min_newtons;
  }
  if(thrust_max_newtons > 0)
  {
    thrust_max_ = thrust_max_newtons;
  }
}

bool PixhawkParser::command(const ControlCommand &command)// Command the quadcopter with the provided values
{
  //Looks like mavlink has changed and its easier to send rpyt commands TODO
  //mavlink_attitude_control_t attitudemsg;
  //mavlink_message_t mavlink_msg;
  //construct rc override command
  uint16_t rc_data[8] = {900};//All channels to min

  //Map the values from raw units to PWM Values
  rc_data[0] = (uint16_t)common::map(command.command_roll,-M_PI/4, M_PI/4, RC_MIN[0],RC_MAX[0]);//ROll

  rc_data[1] = (uint16_t)common::map(-command.command_pitch,-M_PI/4, M_PI/4, RC_MIN[1],RC_MAX[1]);//PITCH (NWU TO NED)

  rc_data[2] = (uint16_t)common::map(command.command_thrust, thrust_min_,thrust_max_,RC_MIN[2],RC_MAX[2]);//Thrust

  rc_data[3]  = (uint16_t)common::map(-command.command_rate_yaw, -1,1,RC_MIN[3],RC_MAX[3]);//Yaw is normalized between -1 to 1 for now as it is the rate of change and not exactly an angle to command

  //Send Data to Quadcopter
  PixhawkParser::sendRadio(rc_data);

  if(log_enable_)
  {

    //log the data:
    cmd_file_<<common::timeMicroseconds()<<"\t"<<rc_data[0]<<"\t"<<rc_data[1]<<"\t"<<rc_data[2]<<"\t"<<rc_data[3]<<endl;
  }
  if(debug_enable_)
  {
    cout<<rc_data[0]<<"\t"<<rc_data[1]<<"\t"<<rc_data[2]<<"\t"<<rc_data[3]<<endl;
  }
  return true;
}
bool PixhawkParser::land()
{
  mavlink_message_t mavlink_msg;
  //construct command
  mavlink_set_mode_t mavmodereq;
  mavmodereq.base_mode = MAV_MODE_FLAG_CUSTOM_MODE_ENABLED;//This is the base mode to make sure we do not get kicked out. Then we add the custom mode which redefines the base mode
  mavmodereq.target_system = targetsys_id;
  mavmodereq.custom_mode = STABILIZE;
  mavlink_msg_set_mode_encode(gcs_link->get_system_id(),gcs_link->get_component_id(),&mavlink_msg,&mavmodereq);
  gcs_link->send_message(&mavlink_msg);

  //#TODO: Add Landing

  //Disarm:
  PixhawkParser::disarm();
  return true;
}

bool PixhawkParser::calibrateImuBias()
{
  cerr<<"Not Implemented Yet!"<<endl;
  return false;
}

bool PixhawkParser::getSensorData(SensorData &data)
{
  data = sensor_data_;//Copy data
  return true;
}

bool PixhawkParser::reset()
{
  cerr<<"Not Applicable to this quadcopter!"<<endl;
  return false;
}

void PixhawkParser::receiveMavlinkMessage(const mavlink_message_t *message, uint8_t sysid, uint8_t compid)
{
  //if (debug_enable_)
    //printf("Received message from serial with ID #%d (sys:%d|comp:%d):\n", message->msgid, message->sysid, message->compid);
  uint16_t mask;//Mask for triggering callbacks

  switch(message->msgid)
  {
  case MAVLINK_MSG_ID_PARAM_VALUE:
    mavlink_param_value_t paramvalue;
    mavlink_msg_param_value_decode(message,&paramvalue);
    if(debug_enable_)
      printf("Param_index: %d\t Param_id: %s\t Param_value: %f\t Param_type:%d\n",paramvalue.param_index, paramvalue.param_id, paramvalue.param_value, paramvalue.param_type);

    //We only care about parameters we need like RC params. Others for now ignore
    char id[16];//ID
    for (int i = 0;i < 4;i++)
    {
      sprintf(id,"RC%d_MIN",i+1);
      if(!strcmp(paramvalue.param_id, id))
      {
        RC_MIN[i] = uint16_t(paramvalue.param_value);
        printf("RC_MIN[%d]: %d\n",i,RC_MIN[i]);
        break;
      }

      sprintf(id,"RC%d_TRIM",i+1);
      if(!strcmp(paramvalue.param_id, id))
      {
        RC_TRIM[i] = uint16_t(paramvalue.param_value);
        printf("RC_TRIM[%d]: %d\n",i,RC_TRIM[i]);
        break;
      }

      sprintf(id,"RC%d_MAX",i+1);
      if(!strcmp(paramvalue.param_id, id))
      {
        RC_MAX[i] = uint16_t(paramvalue.param_value);
        printf("RC_MAX[%d]: %d\n",i,RC_MAX[i]);
        break;
      }
    }
    break;
  case MAVLINK_MSG_ID_RAW_IMU:
    mavlink_raw_imu_t rawimu_val;
    mavlink_msg_raw_imu_decode(message,&rawimu_val);
    sensor_data_.magdata.x = rawimu_val.xmag;
    sensor_data_.magdata.y = rawimu_val.ymag;
    sensor_data_.magdata.z = rawimu_val.zmag;
    sensor_data_.linacc.x = rawimu_val.xacc;
    sensor_data_.linacc.y = rawimu_val.yacc;
    sensor_data_.linacc.z = rawimu_val.zacc;

    //Trigger signal:
    mask = 1<<7;
    signal_sensor_update_(sensor_data, mask);

    break;
  case MAVLINK_MSG_ID_SCALED_PRESSURE:
    mavlink_scaled_pressure_t pressuremsg;
    mavlink_msg_scaled_pressure_decode(message,&pressuremsg);
    sensor_data_.pressure = pressuremsg.press_abs*0.1;//KPa
    //  ROS_INFO("Data Pressure: %f\t%f\t%f",pressuremsg.press_abs,pressuremsg.press_diff1,pressuremsg.press_diff2);
    sensor_data_.temperature = pressuremsg.temperature*0.01;

    //Trigger signal:
    mask = 1<<3;
    signal_sensor_update_(sensor_data, mask);

    break;
  case MAVLINK_MSG_ID_ATTITUDE:
    mavlink_attitude_t attitudemsg;
    mavlink_msg_attitude_decode(message, &attitudemsg);

    //Transforming to NWU frame Convert pitch to pitch - PI (invert it)
    sensor_data_.rpydata.x = common::map_angle(attitudemsg.roll);
    sensor_data_.rpydata.y = -common::map_angle(attitudemsg.pitch);//Transforming NED to NWU
    sensor_data_.rpydata.z = -common::map_angle(attitudemsg.yaw);//Transforming NED to NWU

    if(log_enable_)
    {
      //log the data:
      imu_file_<<common::timeMicroseconds()<<"\t"<<sensor_data_.rpydata.x<<"\t"<<sensor_data_.rpydata.y<<"\t"<<sensor_data_.rpydata.z<<endl;
    }

    //Trigger signal:
    mask = 1<<1;
    signal_sensor_update_(sensor_data, mask);

    break;
  case MAVLINK_MSG_ID_SYS_STATUS:
    mavlink_sys_status_t extended_statusmsg;
    mavlink_msg_sys_status_decode(message,&extended_statusmsg);
    //ROS_INFO("Sys: %d", extended_statusmsg.voltage_battery);
    sensor_data_.battery_volts = ((double)extended_statusmsg.voltage_battery)/1000.0f;//in volts

    //Trigger for this is done in heartbeat since both are in the same group for mask
    break;
  case MAVLINK_MSG_ID_RC_CHANNELS_RAW:
    mavlink_rc_channels_raw_t rcmessage;
    mavlink_msg_rc_channels_raw_decode(message,&rcmessage);
    //if(debug_enable_)
     // printf("RAW RC input: \t R1: %u\t R2 %u \t R_3 %u\t R_4 %u\tR_5 %u\t R_6 %u \t R_7 %u\t R_8 %u \n", rcmessage.chan1_raw, rcmessage.chan2_raw, rcmessage.chan3_raw, rcmessage.chan4_raw, rcmessage.chan5_raw, rcmessage.chan6_raw, rcmessage.chan7_raw, rcmessage.chan8_raw);
    break;
  case MAVLINK_MSG_ID_SERVO_OUTPUT_RAW:
    mavlink_servo_output_raw_t servooutmessage;
    mavlink_msg_servo_output_raw_decode(message,&servooutmessage);
    sensor_data_.motorpwm[0] = servooutmessage.servo1_raw;
    sensor_data_.motorpwm[1] = servooutmessage.servo2_raw;
    sensor_data_.motorpwm[2] = servooutmessage.servo3_raw;
    sensor_data_.motorpwm[3] = servooutmessage.servo4_raw;
    sensor_data_.motorpwm[4] = servooutmessage.servo5_raw;
    sensor_data_.motorpwm[5] = servooutmessage.servo6_raw;
    sensor_data_.motorpwm[6] = servooutmessage.servo7_raw;
    sensor_data_.motorpwm[7] = servooutmessage.servo8_raw;
    //if(debug_enable_)
     // printf("RAW Servo Channels output: \n\t SERVO_1: %u \t SERVO_2 %u \t SERVO_3 %u\t SERVO_4 %u \t SERVO_5 %u \t SERVO_6 %u \t SERVO_7 %u \t SERVO_8 %u \n", servooutmessage.servo1_raw, servooutmessage.servo2_raw, servooutmessage.servo3_raw, servooutmessage.servo4_raw, servooutmessage.servo5_raw, servooutmessage.servo6_raw, servooutmessage.servo7_raw, servooutmessage.servo8_raw);
    if(log_enable_)
    {
      //log the data:
      servo_file_<<common::timeMicroseconds()<<"\t"<<servooutmessage.servo1_raw<<"\t"<<servooutmessage.servo2_raw<<"\t"<<servooutmessage.servo3_raw<<"\t"<<servooutmessage.servo4_raw<<endl;
    }

    //Trigger signal:
    mask = 1<<6;
    signal_sensor_update_(sensor_data, mask);

    break;
  case MAVLINK_MSG_ID_HEARTBEAT:
    //#TODO: Add if quadcopter does not send heart beat for few seconds
    mavlink_heartbeat_t heartbeat;
    mavlink_msg_heartbeat_decode(message,&heartbeat);
    //set the target_sys id to this one:
    if(!heartbeat_received_)
    {
      //Read the parameters etc
      heartbeat_received_ = true;
      targetsys_id = message->sysid;
      printf("targetsys_id: %d\n",targetsys_id);
      //Request Parameters:
      PixhawkParser::parameterRequest();
      usleep(3000000);//Wait for 3 seconds for parameters to be received

      //Setup the data to be requested:

      //Attitude
      PixhawkParser::dataRequest(MAV_DATA_STREAM_EXTRA1, true, 20);
      usleep(50000);

      //EXTENDED (Battery)
      PixhawkParser::dataRequest(MAV_DATA_STREAM_EXTENDED_STATUS, true, 2);
      usleep(50000);

      //RADIO
      PixhawkParser::dataRequest(MAV_DATA_STREAM_RC_CHANNELS, true, 20);
      usleep(50000);

      //RAW IMU
      PixhawkParser::dataRequest(MAV_DATA_STREAM_RAW_SENSORS, true, 20);

      //Set CH7 to 34:
      PixhawkParser::setParameter("CH7_OPT", 34.0, MAV_PARAM_TYPE_INT8);
    }

    //fprintf(stdout,"Sys_status: %s MODE: %d\n",base_mode_map(heartbeat.base_mode).c_str(),heartbeat.custom_mode);
    switch(heartbeat.system_status)
    {
    case MAV_STATE_UNINIT:
      sensor_data_.quadstate = "UNINTIALIZED ";
      break;
    case MAV_STATE_BOOT:
      sensor_data_.quadstate = "BOOTING ";
      break;
    case MAV_STATE_CALIBRATING:
      sensor_data_.quadstate = "CALIBRATING ";
      break;
    case MAV_STATE_STANDBY:
      sensor_data_.quadstate = "STANDBY ";
      break;
    case MAV_STATE_ACTIVE:
      sensor_data_.quadstate = "ACTIVE ";
      break;
    case MAV_STATE_CRITICAL:
      sensor_data_.quadstate = "CRITICAL ";
      break;
    case MAV_STATE_EMERGENCY:
      sensor_data_.quadstate = "EMERGENCY ";
      break;
    case MAV_STATE_POWEROFF:
      sensor_data_.quadstate = "POWEROFF ";
      break;
    case MAV_STATE_ENUM_END:
      sensor_data_.quadstate = "ENUM_END ";
      break;
    }
    if(heartbeat.base_mode & MAV_MODE_FLAG_STABILIZE_ENABLED)
    {
      sensor_data_.quadstate +=  " STABILIZE";
    }
    if(heartbeat.base_mode & MAV_MODE_FLAG_SAFETY_ARMED)
    {
      sensor_data_.quadstate +=  " ARMED";
      sensor_data_.armed = true;
    }
    else
    {
      sensor_data_.armed = false;
      sensor_data_.quadstate += " DISARMED";
    }

    //Trigger signal:
    mask = 1;
    signal_sensor_update_(sensor_data, mask);

    break;
  case MAVLINK_MSG_ID_STATUSTEXT:
    mavlink_statustext_t statusmsg;
    mavlink_msg_statustext_decode(message,&statusmsg);
    printf("Status: %s\n",statusmsg.text);
    break;
  }
}
