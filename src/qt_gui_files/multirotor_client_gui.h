/*
 * Copyright (c) 2011, Dorian Scholz, TU Darmstadt
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the TU Darmstadt nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef rqt_multirotor_client_gui_H
#define rqt_multirotor_client_gui_H

#include <iostream>

#include <ros/ros.h>

#include <pluginlib/class_list_macros.h>
#include <rqt_gui_cpp/plugin.h>
#include <ui_multirotor_client_widget.h>





#include <QCloseEvent>
#include <QCheckBox>
//#include <QMenuBar>
#include <QPushButton>
#include <QMessageBox>
#include <QTableWidget>
#include <QTableWidgetItem>
#include <QString>
#include <QTimer>
//#include <QMutex>
#include <boost/thread/mutex.hpp>
#include <QDockWidget>
#include <QFileDialog>

namespace multirotor_manipulation_and_control_framework {

class MultirotorClientGui
  : public rqt_gui_cpp::Plugin
{

  Q_OBJECT

public:

  MultirotorClientGui();

  ~MultirotorClientGui();

  virtual void initPlugin(qt_gui_cpp::PluginContext& context);

	virtual bool eventFilter(QObject* watched, QEvent* event);


protected:

  qt_gui_cpp::PluginContext* context_;

	Ui::MultirotorClientWidget ui_; 

  QTimer *timer;

	QWidget* widget_;

/*	ros::Subscriber vrpndata_sub;

	ros::Subscriber camdata_sub;

	ros::Subscriber joydata_sub;

	ros::Subscriber gcoptraj_sub;

	ros::Publisher iterationreq_pub;

	ros::Publisher jointstate_pub;

	ros::Publisher armtarget_pub;
  */


	virtual void shutdownPlugin();

	/*QString str;// Data to be put into the textbox;

	char buffer[600];//buffer for creating Qstring data

	void vrpnCallback(const geometry_msgs::TransformStamped::ConstPtr &currframe);
	void ClosingafterGrabbing(const ros::TimerEvent &); //Timer Callback for Closing after grabbing an object
	void camcmdCallback(const geometry_msgs::TransformStamped::ConstPtr &currframe);
	void joyCallback(const sensor_msgs::Joy::ConstPtr &joymsg);
	void gcoptrajectoryCallback(const gcop_comm::CtrlTraj &traj_msg);
  */

	/*protected slots:
	virtual void wrappertakeoff();
	virtual void wrapperLand();
	virtual void wrapperDisarm();
	virtual void wrapperimu_recalib(int);
	virtual void follow_trajectory(int);
	virtual void integrator_control(int);
	virtual void enable_disablecontroller(int);
	virtual void enable_disablecamctrl(int);
	virtual void enable_disablelog(int);
	virtual void enable_disablemanualarmctrl(int);
	virtual void RefreshGui();
  */
	//virtual void Capture_Target();
private slots:
    void on_pushButton_clicked();
};

}
#endif // rqt_multirotor_client_gui_H
