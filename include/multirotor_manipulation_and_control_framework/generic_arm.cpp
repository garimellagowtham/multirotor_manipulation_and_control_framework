#include "generic_arm.h"

using namespace std;

GenericArm::GenericArm(ros::NodeHandle &nh, int njoints): ArmParser()
                                                                , number_of_joints_(njoints)
                                                                , so3(gcop::SO3::Instance())
{

  assert(number_of_joints_ > 0);//Make sure there is atleast one joint in the arm

  //Create Ros publishers and subscribers:
  joint_pub_ = nh.advertise<sensor_msgs::JointState>("command_joint_states", 1);
  gripper_pub_ = nh.advertise<std_msgs::Float64>("ee_grpper",1);
  end_effector_pub_ = nh.advertise<geometry_msgs::Pose>("ee_pose",1);
  power_motors_pub_ = nh.advertise<std_msgs::Bool>("power_motors",1);

  joint_sub_ = nh.subscribe("/joint_states",1, &GenericArm::stateReceived, this);

  //Resize the feedback msg size:
  sensor_data_.joint_angles_.resize(number_of_joints_,0);
  sensor_data_.joint_velocities_.resize(number_of_joints_,0);
}

bool GenericArm::setAngles(const vector<double> &joint_angles, const vector<double> *joint_velocities)
{
  if(log_enable_)
    command_log_file_<<common::timeMicroseconds()<<"\t";
  assert(number_of_joints_ == joint_angles.size());
  sensor_msgs::JointState command_msg;
  command_msg.position = joint_angles;//Copy the data
  if(joint_velocities)
    command_msg.velocity = (*joint_velocities);
  command_msg.header.stamp = ros::Time::now();

  //Log:
  for(int count_joints = 0; count_joints < number_of_joints_; count_joints++)
  {
    if(log_enable_)
    {
      command_log_file_<<joint_angles[count_joints]<<"\t";
    }
    if(joint_velocities)
    {
      if(log_enable_)
      {
        command_log_file_<<(*joint_velocities)[count_joints]<<"\t";
      }
    }
  }
  if(log_enable_)
    command_log_file_<<endl;

  joint_pub_.publish(command_msg);
  return true;
}

bool GenericArm::setEndEffectorPose(const Eigen::Matrix4d &end_effector_pose)
{
  geometry_msgs::Pose pose_msg;

  //Fill position
  pose_msg.position.x = end_effector_pose(0,3);
  pose_msg.position.y = end_effector_pose(1,3);
  pose_msg.position.z = end_effector_pose(2,3);

  //Fill Orientation:
  Eigen::Vector4d quat;
  so3.g2quat(quat, end_effector_pose.topLeftCorner(3,3));
  pose_msg.orientation.w = quat(0);
  pose_msg.orientation.x = quat(1);
  pose_msg.orientation.y = quat(2);
  pose_msg.orientation.z = quat(3);

  if(log_enable_)
  {
    //Since we do not use IK here, we just log the end effector pose here:
    command_log_file_<<common::timeMicroseconds()<<"\t "<<pose_msg.position.x<<"\t "<<pose_msg.position.y<<"\t "<<pose_msg.position.z
                    <<"\t "<<pose_msg.orientation.w<<"\t "<<pose_msg.orientation.x<<"\t "<<pose_msg.orientation.y<<"\t "<<pose_msg.orientation.z<<endl;
  }

  //Publish msg
  end_effector_pub_.publish(pose_msg);

  return true;
}

bool GenericArm::powerMotors(bool state)
{
  //Fill msg
  std_msgs::Bool msg;
  msg.data = state;

  //Publish msg
  power_motors_pub_.publish(msg);

  return true;
}

void GenericArm::enableLog(string log_directory)
{
  feedback_log_file_.open((log_directory+"/arm_feedback.dat").c_str());
  command_log_file_.open((log_directory+"/arm_commands.dat").c_str());

  feedback_log_file_.precision(10);
  command_log_file_.precision(10);

  feedback_log_file_<<"#Time\t";
  command_log_file_<<"#Time\t";
  for(int count =0; count < number_of_joints_; count++)
  {
    feedback_log_file_<<" Joint_Angle["<<count<<"]\t Joint_Velocity["<<count<<"]\t ";
    command_log_file_<<" Command_Joint_Angle["<<count<<"]\t Command_Joint_Velocity["<<count<<"]\t ";
  }
  feedback_log_file_<<endl;
  command_log_file_<<endl;

  log_enable_ = true;
}

bool GenericArm::grip(double value)
{
  //Publish gripper msg:
  std_msgs::Float64 msg;
  msg.data = value;
  gripper_pub_.publish(msg);
  return true;
}

void GenericArm::stateReceived(const sensor_msgs::JointState::ConstPtr &joint_state)
{
  assert(number_of_joints_ == joint_state->position.size());//Check number of joints

  //Copy the joint angles and velocities data:
  sensor_data_.joint_angles_ = joint_state->position;
  sensor_data_.joint_velocities_ = joint_state->velocity;

  //Signal
  signal_feedback_received_(sensor_data);//Signal the callback functions

  //Log Data
  if(log_enable_)
  {
    feedback_log_file_<<common::timeMicroseconds()<<"\t";
  }

  for(int count_joints = 0; count_joints < number_of_joints_; count_joints++)
  {
    if(log_enable_)
    {
      feedback_log_file_<<joint_state->position[count_joints]<<" \t "<<joint_state->velocity[count_joints]<<" \t ";
    }
  }
}


