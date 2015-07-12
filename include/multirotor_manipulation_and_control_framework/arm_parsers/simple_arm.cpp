#include "simple_arm.h"

using namespace std;

SimpleArm::SimpleArm(int deviceIndex, uint16_t baudrate):ArmParser()
                                                        , se3(gcop::SE3::Instance())
                                                        , thread_close(false)
{
  arm_id_[0] = 1; arm_id_[1] = 2;//Set IDs
  arm_min_angles_[0] = 937; arm_min_angles_[1] = 0;//Set Min Angles
  arm_max_angles_[0] = 2337; arm_max_angles_[1] = 3070;//Set Max angles
  arm_offset_angles_[0] = 1144; arm_offset_angles_[1] = 2047; //Set Default angles (2167 - 180degrees)
  arm_default_velocities_[0] = 52; arm_default_velocities_[1] = 52;//Set default velocity of the arm
  arm_max_angle_velocities_[0] = 176; arm_max_angle_velocities_[1] = 176; //Max angular velocity (unit: 0.114 rpm)
  l1 = 0.175; l2 = 0.42; x1 = 0.025;//Physical lengths of the arm used for inverse kinematics

  //Open the communication to arm:
  uint16_t baudnum = 2000000 / (baudrate + 1);
  printf("Initializing dynamixel with [ttyUSB/ACM%d] , baudrate: %d\n",deviceIndex, baudrate);
  if( dxl_initialize(deviceIndex, baudnum) == 0 )
  {
    printf( "Failed to open USB2Dynamixel!\n" );
    return;
  }
  else
  {
    printf( "Succeed to open USB2Dynamixel!\n" );
  }

  //Power the motors:
  if(!powerMotors(true))
  {
    printf("Cannot power motors on! \n");
    return;
  }

  //Initialize arm feedback data
  sensor_data_.joint_angles_.resize(NOFJOINTS);
  sensor_data_.joint_velocities_.resize(NOFJOINTS);

  //Create a thread to receive feedback from arm at 20Hz:
  receive_thread_ = boost::thread(boost::bind(&SimpleArm::serialReceiveThread, this));

  //Set Default Joint Angles and Velocities:
  {
    std::vector<double> joint_angles(NOFJOINTS);
    std::vector<double> joint_velocities(NOFJOINTS);
    joint_angles[0] = M_PI/2; joint_angles[1] = -M_PI;//rad
    joint_velocities[0] = 0.621; joint_velocities[1] = 0.621;//rad/s
    //joint_velocities[0] = 0.321; joint_velocities[1] = 0.321;//rad/s
    setAngles(joint_angles, &joint_velocities);

    //Wait till the joint angles are achieved:
    {
      unsigned long count = 0;
      while(count < 100)//100 x 100 ms =  10s
      {
        bool result = true;
        {
          boost::mutex::scoped_lock lock(serial_mutex);//Lock until data is copied over
          for(int count = 0; count < NOFJOINTS; count++)
          {
            //cout<<"Joint ["<<count<<"]: "<<sensor_data.joint_angles_[count]<<"\t"<<sensor_data.joint_velocities_[count]<<endl;
            if(abs(sensor_data.joint_angles_[count] - joint_angles[count]) > 4.0*(M_PI/180.0)) //error of 4 degrees
            {
              result = false;
              break;
            }
          }
        }

        if(result == true)
          break;
        usleep(100000);//Sleep microseconds
        //cout<<count<<endl;
        count++;
      }
    }
  }

}

bool SimpleArm::setAngles(const vector<double> &joint_angles, const vector<double> *joint_velocities)
{
  if(log_enable_)
    command_log_file_<<common::timeMicroseconds()<<"\t";

  boost::mutex::scoped_lock lock(serial_mutex);//Lock until data is copied over
  for(int count_joints = 0; count_joints < NOFJOINTS; count_joints++)
  {
    //Command Angle:
    //Limit Joint angles:
    uint16_t command_pwm = 0;
    int scaled_angle = round(joint_angles[count_joints]*(180.0/(M_PI*ANGRES)));
    if(scaled_angle <  - int(arm_offset_angles_[count_joints]))
      command_pwm = 0;
    else
      command_pwm = arm_offset_angles_[count_joints] + scaled_angle;
    command_pwm = command_pwm > arm_max_angles_[count_joints]?arm_max_angles_[count_joints]:command_pwm < arm_min_angles_[count_joints]?arm_min_angles_[count_joints]:command_pwm;
    dxl_write_word(arm_id_[count_joints], P_GOAL_POSITION_L, command_pwm);//Write to Motors
    if(log_enable_)
    {
      command_log_file_<<joint_angles[count_joints]<<"\t";
    }

    //Command Velocities if provided:
    if(joint_velocities != 0)
    {
      uint16_t command_vel = uint16_t((*joint_velocities)[count_joints]/ANGVELRES);
      command_vel = command_vel > arm_max_angle_velocities_[count_joints]?arm_max_angle_velocities_[count_joints]:command_vel<=0?1:command_vel;//We do not want zero velocity as that implies full speed
      dxl_write_word(arm_id_[count_joints], P_MOVING_SPEED_L, command_vel);//Write to Motors
      if(log_enable_)
      {
        command_log_file_<<(*joint_velocities)[count_joints]<<"\t";
      }
    }
  }

  if(log_enable_)
    command_log_file_<<endl;
}

bool SimpleArm::setEndEffectorPose(const Eigen::Matrix4d &end_effector_pose)
{
  //Inverse Kinematics:
  static vector<vector<double> > joint_angles;// Joint Angles
  double value = inverseKinematics(end_effector_pose, joint_angles);//Find the solutions
  //DEBUG
  //std::cout<<"Solution_1: "<<joint_angles[0][0]<<","<<joint_angles[0][1]<<endl<<"Solution_2: "<<joint_angles[1][0]<<","<<joint_angles[1][1]<<endl;

  //Using upper elbow solution in local frame:
  if(value >= 0)
    setAngles(joint_angles[0]);
  else
    return false;

  return true;
}

bool SimpleArm::powerMotors(bool state)
{
  int power_motors = state?1:0;
  boost::mutex::scoped_lock lock(serial_mutex);//Lock until data is copied over
  for(int count_joints = 0;count_joints < NOFJOINTS; count_joints++)
  {
    dxl_write_byte( arm_id_[count_joints], P_TORQUE_ENABLE, power_motors);
    int CommStatus = dxl_get_result();

    if( CommStatus == COMM_RXSUCCESS )
    {
      printf("Power motors on/off\n");
      PrintErrorCode();
    }
    else
    {
      PrintCommStatus(CommStatus);
      return false;//Cannot power the motor
    }
  }
  return true;
}

void SimpleArm::enableLog(string log_directory)
{
  feedback_log_file_.open((log_directory+"/arm_feedback.dat").c_str());
  command_log_file_.open((log_directory+"/arm_commands.dat").c_str());

  feedback_log_file_.precision(10);
  command_log_file_.precision(10);

  feedback_log_file_<<"#Time\t Joint_Angle[0]\t Joint_Velocity[0]\t Joint_Angle[1]\t Joint_Velocity[1]"<<endl;
  command_log_file_<<"#Time\t Command_Joint_Angle[0]\t Command_Joint_Velocity[0]\t Command_Joint_Angle[1]\t Command_Joint_Velocity[1]"<<endl;

  log_enable_ = true;
}


bool SimpleArm::grip(double value)
{
  //Currently Not Implemented
  cerr<<"Currently not implemented"<<endl;
}

bool SimpleArm::forwardKinematics(Eigen::Matrix4d &end_effector_pose, const vector<double> &joint_angles)
{
  const double &a1 = joint_angles[0]; // first joint
  const double &a2 = joint_angles[1]; // second joint

  Eigen::Vector3d xyz, rpy;

  xyz[0] = l2*(cos(a1)*sin(a2) + cos(a2)*sin(a1)) + x1*cos(a1) + l1*sin(a1);//X
  xyz[1] = 0;
  xyz[2] = l1*cos(a1) - l2*(sin(a1)*sin(a2) - cos(a1)*cos(a2)) - x1*sin(a1);//Z

  rpy.setZero();
  rpy[1] = a1+a2;//Pitch

  se3.rpyxyz2g(end_effector_pose, rpy, xyz);

  return true;
}

double SimpleArm::inverseKinematics(const Eigen::Matrix4d &end_effector_pose, vector<vector<double> > &joint_angles)
{
  //Resizing the angles
  joint_angles.resize(2);
  joint_angles[0].resize(2, 0);
  joint_angles[1].resize(2, 0);

  const double &p0 = end_effector_pose(0,3);
  const double &p2 = end_effector_pose(2,3);

  // it easier to derive IK in terms of polar coordinates
  double r = sqrt(p0*p0 + p2*p2);    // distance to end-eff

  double xr = p0;

  double c = atan2(p2, xr);                         // angle to end-eff

  double l1_2 = l1*l1;
  double l1_4 = l1_2*l1_2;
  double l2_2 = l2*l2;
  double l2_4 = l2_2*l2_2;
  double r_2 = r*r;
  double r_4 = r_2*r_2;
  double x1_2 = x1*x1;
  double x1_4 = x1_2*x1_2;

  double C = - l1_4 + 2*l1_2*l2_2 + 2*l1_2*r_2 - 2*l1_2*x1_2 - l2_4 + 2*l2_2*r_2 + 2*l2_2*x1_2 - r_4 + 2*r_2*x1_2 - x1_4;
  // outside of manipulator workspace
  if (C < 0)
    return C;

  double ks[2][2];

  // Grobner basis
  ks[0][0] = -(sqrt(C) - 2*l1*r*cos(c) + 2*r*x1*sin(c))/(l1_2 + 2*sin(c)*l1*r - l2_2 + r_2 + 2*cos(c)*r*x1 + x1_2);
  ks[1][0] = (sqrt(C) + 2*l1*r*cos(c) - 2*r*x1*sin(c))/(l1_2 + 2*sin(c)*l1*r - l2_2 + r_2 + 2*cos(c)*r*x1 + x1_2);

  ks[0][1] = -(2*l2*x1 + sqrt(C))/(l1_2 - 2*l1*l2 + l2_2 - r_2 + x1_2);
  ks[1][1] = -(2*l2*x1 - sqrt(C))/(l1_2 - 2*l1*l2 + l2_2 - r_2 + x1_2);

  for (int i = 0; i < 2; ++i) {
    for (int j = 0; j < 2; ++j) {
      double &k = ks[i][j];
      joint_angles[i][j] = atan2(2*k/(1+k*k), (1-k*k)/(1+k*k));
    }
  }
  return C;
}

void SimpleArm::serialReceiveThread()
{
  //In this thread keep receiving serial data and sleep for some time (20Hz = 50 ms)
  while(!thread_close)
  {
    //Log time
    if(log_enable_)
    {
      feedback_log_file_<<common::timeMicroseconds()<<"\t";
    }

    //Find the current motor position and velocities
    {
      uint16_t present_pos, present_vel;//Temp variables
      int CommStatus;
      boost::mutex::scoped_lock lock(serial_mutex);//Lock until data is copied over
      for(int count_joints = 0; count_joints < NOFJOINTS; count_joints++)
      {
        present_pos = dynamixelsdk::dxl_read_word( arm_id_[count_joints], P_PRESENT_POSITION_L );
        CommStatus = dynamixelsdk::dxl_get_result();

        if( CommStatus == COMM_RXSUCCESS )
        {
          sensor_data_.joint_angles_[count_joints] = (double(present_pos) - double(arm_offset_angles_[count_joints]))*((M_PI*ANGRES)/180.0);
          common::map_angle(sensor_data_.joint_angles_[count_joints]);
          if(log_enable_)
          {
            feedback_log_file_<<sensor_data_.joint_angles_[count_joints]<<"\t";
          }
          //[DEBUG]
          //std::cout<<"Current Position ID["<<arm_id_[count_joints]<<"] :"<<sensor_data_.joint_angles_[count_joints]<<std::endl;
        }
        else
        {
          //[DEBUG]
          PrintCommStatus(CommStatus);
          //#TODO: Check number of errors and print in sensor status or close the thread
          return;
        }

        /////Velocity Reading:
        present_vel = dynamixelsdk::dxl_read_word( arm_id_[count_joints], P_PRESENT_SPEED_L );
        CommStatus = dynamixelsdk::dxl_get_result();

        if( CommStatus == COMM_RXSUCCESS )
        {
          int actual_vel = 0;

          //Find the direction of motor:
          if(present_vel <= 1023)
          {
            actual_vel = present_vel;
          }
          else
          {
            actual_vel = -(present_vel - 1024);
          }

          //Set Joint Velocities
          sensor_data_.joint_velocities_[count_joints] = actual_vel*ANGVELRES;
          if(log_enable_)
          {
            feedback_log_file_<<sensor_data_.joint_velocities_[count_joints]<<"\t";
          }
          //[DEBUG]
          //std::cout<<"Current Velocity ID["<<arm_id_[count_joints]<<"] :"<<sensor_data.joint_velocities_[count_joints]<<std::endl;
        }
        else
        {
          //[DEBUG]
          PrintCommStatus(CommStatus);
          return;
        }
      }
      if(log_enable_)
      {
        feedback_log_file_<<endl;
      }
    }

    // Signal that data is updated
    signal_feedback_received_(sensor_data);//Call signal with the updated feedback data

    usleep(50000);//sleep for microseconds
  }
}
