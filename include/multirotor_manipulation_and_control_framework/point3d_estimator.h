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

#ifndef POINT3D_ESTIMATOR_H
#define POINT3D_ESTIMATOR_H

#include "state_estimator.h"
#include<gcop/kalmanpredictor.h>
#include<gcop/kalmancorrector.h>
#include<gcop/point3d.h>
#include<gcop/point3dgps.h>
#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <gcop/so3.h>

/**
 * @brief This class implements an estimator for Position of the body using a motion capture system
 * 
 *  It employs a constant velocity model and uses position estimates from MOCAP system to correct the position
 *  The orientation is passed unfiltered from measurements of MOCAP. The angular velocity is \b NOT observed in this filter.
 */
class Point3DEstimator : public StateEstimator
{
  typedef gcop::KalmanPredictor<gcop::Point3dState, 6, 3, Eigen::Dynamic> Point3dKalmanPredictor;
  typedef gcop::KalmanCorrector<gcop::Point3dState, 6, 3, Eigen::Dynamic, Eigen::Vector3d, 3> Point3dGpsKalmanCorrector;
public:
    /**
    * @brief Constructor
    *
    * @param nh             NodeHandle using which publishers and subscribers are created
    * @param quad_parser    Quadcopter Parser which provides imu data etc for estimation
    */
    Point3DEstimator(ros::NodeHandle &nh, QuadcopterParser &quad_parser);
    void predictState(gcop::Body3dState &state);
protected:
    gcop::Point3d point3d_sys_;///< Point3d System
    gcop::Point3dGps<> gps_sensor_;///< GPS Sensor
    Point3dKalmanPredictor kalman_predictor_;///< Kalman Predictor
    Point3dGpsKalmanCorrector kalman_corrector_;///< Kalman Corrector for GPS type data
    gcop::Point3dState point3d_state_;///< State of point3d system
    ros::Time current_time_;///< Current time of the system
    Eigen::Vector4d current_pose_;///< Quaternion of pose (w,x,y,z)
    gcop::SO3 &so3;///< Reference to SO3 instance

    //ROS stuff:
    ros::NodeHandle &nh_;///< NodeHandle passed
    ros::Subscriber mocap_sub_;///< Motion Capture subscriber
protected:
    /**
    * @brief Callback when external pose from Motion Capture system arrives
    *
    * @param pose     External Pose from Motion Capture system
    */
    void poseCallback(const geometry_msgs::Pose &pose);
};

#endif // POINT3D_ESTIMATOR_H
