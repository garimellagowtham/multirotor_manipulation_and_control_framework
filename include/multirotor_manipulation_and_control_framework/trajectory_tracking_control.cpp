#include "trajectory_tracking_control.h"
#include <assert.h>

using namespace std;

TrajectoryTrackingController::TrajectoryTrackingController(): log_enable_(false)
                                                              , so3(gcop::SO3::Instance())
                                                              , bound_velocity_difference_(3)
{
  //Default Gains:
  controller_gains_.kpr = 1.0;
  controller_gains_.kpt = 0.5;
  controller_gains_.kdr = 1.0;
  controller_gains_.kdt = 0.5;
  controller_gains_.kpy = 0.5;
  controller_gains_.kiy = 0.001;
  controller_gains_.kit = 0.001;
  //Default Goal State:
  goal_state_.first.setIdentity();
  goal_state_.second.setZero();
  //Default Bias:
  controller_bias_.force_bias = Eigen::Vector3d(0, 0, -0.5*9.81);//N
  controller_bias_.rateyaw_bias = 0.0;
  //Default Command Bounds:
  controller_command_bound_.command_roll =  M_PI/18;//10 degrees
  controller_command_bound_.command_pitch = M_PI/18;
  controller_command_bound_.command_thrust = 0.5;//N about Fext
  controller_command_bound_.command_rate_yaw = 0.2;// Rate yaw command is between -1 and 1
}

void TrajectoryTrackingController::setGains(const ControllerGains &gains)
{
  //#TODO: Add Prechecks on gains
  controller_gains_ = gains;
}

void TrajectoryTrackingController::setGoal(const gcop::Body3dState &goal_state)
{
  //#TODO: Add Prechecks like geo-fencing
  goal_state_ = goal_state;
  so3.g2q(goal_rpy_orientation_, goal_state.first);
}
void TrajectoryTrackingController::getGoal(gcop::Body3dState &goal_state)
{
  //Copy to the requested value
  goal_state = goal_state_;
}
bool TrajectoryTrackingController::setBound(const QuadcopterParser::ControlCommand &command_bound)
{
  //PreCheck:
  if(!((command_bound.command_pitch < M_PI/4) && (command_bound.command_pitch > -M_PI/18)))
    return false;
  if(!((command_bound.command_roll < M_PI/4) && (command_bound.command_roll > -M_PI/18)))
    return false;
  if(!((command_bound.command_thrust < 2.0) && (command_bound.command_thrust > 1.0)))
    return false;
  if(!((command_bound.command_rate_yaw < 1.0) && (command_bound.command_rate_yaw > 0.2)))
    return false;
  //Copy Bounds:
  controller_command_bound_ = command_bound;
  return true;
}

bool TrajectoryTrackingController::setBias(const ControllerBias &controller_bias)
{
  //Precheck:
  if(controller_bias.force_bias[0] > 1.0 || controller_bias.force_bias[0] < -1.0)
    return false;
  if(controller_bias.force_bias[1] > 1.0 || controller_bias.force_bias[1] < -1.0)
    return false;
  if(controller_bias.force_bias[2] > -0.2*9.81 || controller_bias.force_bias[2] < -2.0*9.81)
    return false;
  if(controller_bias.rateyaw_bias > 0.3 || controller_bias.rateyaw_bias < -0.3)
    return false;
  //Copy Bias:
  controller_bias_ = controller_bias;
  return true;
}

bool TrajectoryTrackingController::setCtrl(QuadcopterParser::ControlCommand &result_command, const gcop::Body3dState &filtered_state)
{
  Eigen::Vector3d rpy_orientation;//Orientation of quadcopter
  Eigen::Vector3d dx, ddx;// Errors in position controller

  so3.g2q(rpy_orientation, filtered_state.first);//Get Orientation of quadcopter

  dx = goal_state_.second.head<3>() - filtered_state.second.head<3>();

  ddx = goal_state_.second.tail<3>() - filtered_state.second.tail<3>();

  //Check bound on difference:
  if(ddx.norm() > bound_velocity_difference_)
  {
    cerr<<"Velocity Bound exceeded: "<<ddx.norm()<<endl;
    return false;
    //ddx = velbound*ddx.normalized();//Bind the vel difference
  }

  //Integrator:
  if(integrator_enable_)
  {
    //#TODO: Find a better way of doing yaw integration
    if(filtered_state.second[2] > 0.4)//Only start yaw integration after quad is atleast 0.4 m high from ground
    {
      controller_bias_.rateyaw_bias += -controller_gains_.kiy*(rpy_orientation[2] - goal_rpy_orientation_[2]);//Integrator to avoid yaw bias in terms of motors not providing same torque for some reason
      controller_bias_.force_bias[0] -= (controller_gains_.kit/4)*dx[0];//X integration
      controller_bias_.force_bias[1] -= (controller_gains_.kit/4)*dx[1];//X integration
      controller_bias_.force_bias[2] -= (controller_gains_.kit)*dx[2];//X integration
      //controller_bias_.force_bias -= Eigen::Vector3d(controller_gains_.kit/4, controller_gains_.kit/4, controller_gains_.kit).cwiseProduct(dx);
    }
    else//If lower than 0.4 meters high only integrate z do not integrate x, y
    {
      controller_bias_.force_bias[2] -= (controller_gains_.kit)*dx[2];//X integration
    }
    //cout<<"Fext ["<<Fext[0]<<"]\t["<<Fext[1]<<"]\t"<<Fext[2]<<"]"<<endl;//#DEBUG
  }

  //desired vector to goal:
  Eigen::Vector3d desired_vector_to_goal = -controller_bias_.force_bias + Eigen::Vector3d(controller_gains_.kpr*dx[0],controller_gains_.kpr*dx[1],controller_gains_.kpt*dx[2]) -Eigen::Vector3d(controller_gains_.kdr*ddx[0],controller_gains_.kdr*ddx[1],controller_gains_.kdt*ddx[2]);
  Eigen::Vector3d desired_rpy;//Orientation for commanding


  //Get RPY to goal
  Eigen::Vector3d yaw_vector(cos(rpy_orientation[2]),sin(rpy_orientation[2]),0);
  Eigen::Matrix3d desired_orientation;
  desired_orientation.row(2) = desired_vector_to_goal.normalized();
  desired_orientation.row(1) = desired_orientation.row(2).cross(yaw_vector).normalized();//desiredvec is normalized and is towards z axis
  desired_orientation.row(0) = desired_orientation.row(1).cross(desired_orientation.row(2));
  desired_orientation = desired_orientation.transpose();
  so3.g2q(desired_rpy, desired_orientation);

  //set throttle: //#TODO: Check This is kind of fishy
  result_command.command_thrust = desired_vector_to_goal.dot(Eigen::Vector3d(0,0,1.0));
  result_command.command_thrust = -controller_bias_.force_bias[2] + bind(result_command.command_thrust + controller_bias_.force_bias[2], controller_command_bound_.command_thrust);

  //set yaw command:
  result_command.command_rate_yaw = controller_bias_.rateyaw_bias - controller_gains_.kpy*(rpy_orientation[2] - goal_rpy_orientation_[2]);
  result_command.command_rate_yaw = bind(result_command.command_rate_yaw, controller_command_bound_.command_rate_yaw);

  //set roll and pitch command:
  result_command.command_roll = bind(desired_rpy[0], controller_command_bound_.command_roll);
  result_command.command_pitch = bind(desired_rpy[1], controller_command_bound_.command_pitch);

  return true;
}

