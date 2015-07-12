#include "point3d_estimator.h"

Point3DEstimator::Point3DEstimator(ros::NodeHandle &nh, QuadcopterParser &quad_parser): StateEstimator(quad_parser)
                                                                 , kalman_predictor_(point3d_sys_)
                                                                 , kalman_corrector_(point3d_sys_.X, gps_sensor_)
                                                                 , nh_(nh)
                                                                 , so3(gcop::SO3::Instance())
{
  //Set initial guess and covariance:
  point3d_sys_.sa = 0.5;//Covariance of process noise in point 3d system (acceleration noise). The system assumes zero acceleration with white noise 0.5
  gps_sensor_.sxy = 0.01;//1cm Measurement std deviation in x and y directions
  gps_sensor_.sz = 0.01;//1cm  Measurement std deviation in z direction
  point3d_state_.P.topLeftCorner<3,3>().diagonal().setConstant(2);  // q Stdeviation etc
  point3d_state_.P.bottomRightCorner<3,3>().diagonal().setConstant(2);  // v Initially large stdeviation

  //ROS Stuff:
  mocap_sub_ = nh_.subscribe("uav_pose", 1, &Point3DEstimator::poseCallback, this);
}

void Point3DEstimator::predictState(gcop::Body3dState &state)
{
  ros::Time time_predict = ros::Time::now();
  double dt = (time_predict - current_time_).toSec();

  //Prediction:
  gcop::Point3dState predict_point3d_state_;
  Eigen::Vector3d u(0, 0, 0);//no inputs constant velocity model
  kalman_predictor_.Predict(predict_point3d_state_, time_predict.toSec(), point3d_state_, u, dt);

  //Fill body3dState:
  state.second.head<3>() = predict_point3d_state_.q;
  state.second.tail<3>() = predict_point3d_state_.v;
  //Angular velocity is not observed here
  so3.quat2g(state.first, current_pose_);//Fill orientation
}

void Point3DEstimator::poseCallback(const geometry_msgs::Pose &pose)
{
  static ros::Time t0 = ros::Time::now();//Starting time

  current_time_ = ros::Time::now();//Current time;

  double dt = (current_time_ - t0).toSec();//DeltaT


  //Prediction:
  gcop::Point3dState predict_point3d_state_;
  Eigen::Vector3d u(0, 0, 0);//no inputs constant velocity model
  kalman_predictor_.Predict(predict_point3d_state_, current_time_.toSec(), point3d_state_, u, dt);

  //Correction:
  Eigen::Vector3d z;//Measurement
  z<<pose.position.x, pose.position.y, pose.position.z;
  kalman_corrector_.Correct(point3d_state_, current_time_.toSec(), predict_point3d_state_, u, z);

  //Copy orientation:
  current_pose_<<pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z;
}


