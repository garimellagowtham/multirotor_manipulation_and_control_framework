#include <iostream>
#include <ros/ros.h>
#include <arm_parsers/generic_arm.h>

using namespace std;

boost::shared_ptr<ArmParser> arm_instance_;
void armFeedback(const ArmParser::ArmSensorData &sensor_data)
{
  static ros::Time t1 = ros::Time::now();
  ros::Time t2 = ros::Time::now();
  ROS_INFO("Time diff: %f",(t2 - t1).toSec());
  //Print the joint angles and velocities
  for(int count = 0; count < 8; count++)
  {
    cout<<"J["<<count<<"]: "<<sensor_data.joint_angles_[count]<<"\t"<<sensor_data.joint_velocities_[count]<<endl;
  }
  t1 = t2;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "arm_node");
  ros::NodeHandle nh("~");//Create NodeHandle

  arm_instance_.reset(new GenericArm(nh));//Create the arm instance

  //Test arm angles:
  ROS_INFO("Testing commanding angles.\n\n\t Press any key to continue...");
  getchar();

  ROS_INFO("Commanding 0 degrees for all the joints");
  {
    std::vector<double> joint_angles(7,0);//Create a vector
    joint_angles[1] = -1.6;//Default config
    arm_instance_->setAngles(joint_angles);//Set Joint Angles
  }
  getchar();

  ros::Duration(1.0).sleep();// Sleep for 1 second

  ROS_INFO("Commanding to fold config:");
  {
    std::vector<double> joint_angles(7,0);//Create a vector
    joint_angles[3] = -M_PI/2;
    joint_angles[6] = 0.002;
    arm_instance_->setAngles(joint_angles);//Set Joint Angles
  }
  getchar();
  ros::Duration(1.0).sleep();// Sleep for 1 second

  ROS_INFO("Commanding back to zero config with different velocities");
  {
    std::vector<double> joint_angles(7,0);//Create a vector
    joint_angles[1] = -1.6;//Default config
    std::vector<double> joint_velocities(7,0.5);//Create a vector
    joint_velocities[0] = 0.05;
    arm_instance_->setAngles(joint_angles, &joint_velocities);//Set Joint Angles
  }
  getchar();
  ros::Duration(1.0).sleep();// Sleep for 1 second

  return 0;
}
