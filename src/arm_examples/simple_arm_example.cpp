#include <iostream>
#include <ros/ros.h>
#include <multirotor_manipulation_and_control_framework/simple_arm.h>

using namespace std;

boost::shared_ptr<ArmParser> arm_instance_;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "arm_node");
  ros::NodeHandle nh("~");//Create NodeHandle

  arm_instance_.reset(new SimpleArm(0, 57600));//Create the arm instance

  //Testing Inverse and Forward Kinematics:
  ROS_INFO("Testing Inverse Kinematics");
  {
    Eigen::Matrix4d end_effector_pose;
    end_effector_pose(0,3) = 0.4;
    end_effector_pose(2,3) = 0.3;
    arm_instance_->setEndEffectorPose(end_effector_pose);
  }
  ros::Duration(13.0).sleep();// Sleep for seconds
  return 0;

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

  ROS_INFO("Commanding -pi/2 degree and pi/2 for the second arm with velocities 0.1 and 0.2 rad/s");
  {
    std::vector<double> joint_angles(2,0);//Create a vector
    joint_angles[0] = M_PI/1.2;
    joint_angles[1] = M_PI/2;
    std::vector<double> joint_velocities(2,0.5);//Create a vector
    joint_velocities[1] = 0.05;
    arm_instance_->setAngles(joint_angles, &joint_velocities);//Set Joint Angles
  }
  ros::Duration(10.0).sleep();// Sleep for seconds

    

  //Power off motors:
  ROS_INFO("Powering off");
  arm_instance_->powerMotors(false);

  return 0;
}
