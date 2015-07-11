#include <iostream>
#include <ros/ros.h>
#include <multirotor_manipulation_and_control_framework/simple_arm.h>

using namespace std;

boost::shared_ptr<ArmParser> arm_instance_;

void armFeedback(const ArmParser::ArmSensorData &sensor_data)
{
  static ros::Time t1 = ros::Time::now();
  ros::Time t2 = ros::Time::now();
  ROS_INFO("Time diff: %f",(t2 - t1).toSec());
  //Print the joint angles and velocities
  ROS_INFO("Joint Angles and velocities: %f,%f\t%f,%f",sensor_data.joint_angles_[0], sensor_data.joint_angles_[1],sensor_data.joint_velocities_[0], sensor_data.joint_velocities_[1]);
  t1 = t2;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "arm_node");
  ros::NodeHandle nh("~");//Create NodeHandle

  arm_instance_.reset(new SimpleArm(0, 57600));//Create the arm instance

  //Set slot for receiving arm feedback
  arm_instance_->signal_feedback_received_.connect(armFeedback);

  //Test arm angles:
  ROS_INFO("Testing commanding angles");

  ROS_INFO("Commanding 0 degrees for both arms");
  {
    std::vector<double> joint_angles(2,0);//Create a vector
    arm_instance_->setAngles(joint_angles);//Set Joint Angles
  }

  ros::Duration(10.0).sleep();// Sleep for seconds

  ROS_INFO("Commanding 0 degree and pi/2 for the second arm");
  {
    std::vector<double> joint_angles(2,0);//Create a vector
    joint_angles[1] = M_PI/2;
    arm_instance_->setAngles(joint_angles);//Set Joint Angles
  }
  ros::Duration(10.0).sleep();// Sleep for seconds

  ROS_INFO("Commanding pi/2 degree and -pi/2 for the second arm with velocities 0.1 and 0.2 rad/s");
  {
    std::vector<double> joint_angles(2,0);//Create a vector
    joint_angles[0] = M_PI/4;
    joint_angles[1] = -M_PI/2;
    std::vector<double> joint_velocities(2,0.5);//Create a vector
    joint_velocities[1] = 0.2;
    arm_instance_->setAngles(joint_angles, &joint_velocities);//Set Joint Angles
  }
  ros::Duration(10.0).sleep();// Sleep for seconds

  //Testing Inverse and  Inverse Kinematics:
  ROS_INFO("Testing Inverse Kinematics");
  {
    Eigen::Matrix4d end_effector_pose;
    end_effector_pose(0,3) = 0.4;
    end_effector_pose(2,3) = 0.3;
    arm_instance_->setEndEffectorPose(end_effector_pose);
  }
  ros::Duration(10.0).sleep();// Sleep for seconds

  //Power off motors:
  ROS_INFO("Powering off");
  arm_instance_->powerMotors(false);

  return 0;
}
