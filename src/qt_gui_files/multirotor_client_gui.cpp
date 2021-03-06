/*
 * Copyright (c) 2011,  Gowtham Garimella JHU
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

#include "multirotor_client_gui.h"


namespace multirotor_manipulation_and_control_framework {

  MultirotorClientGui::MultirotorClientGui() : rqt_gui_cpp::Plugin()
                                   , context_(0)
                                   , widget_(0)
  {
    setObjectName("MultirotorClient");
  }

  MultirotorClientGui::~MultirotorClientGui()
  {
  }

  void MultirotorClientGui::initPlugin(qt_gui_cpp::PluginContext& context)
  {
    widget_ = new QWidget();

    ui_.setupUi(widget_); 

    context_ = &context;

    widget_->setWindowTitle("MultirotorClient[*]");

    if (context.serialNumber() != 1)
    {
      widget_->setWindowTitle(widget_->windowTitle() + " (" + QString::number(context.serialNumber()) + ")");
    }

    context.addWidget(widget_);

    // trigger deleteLater for plugin when widget or frame is closed
    widget_->installEventFilter(this);

    //Connect all the slots as needed
    connect(ui_.pushButton, SIGNAL(clicked()), this, SLOT(on_pushButton_clicked()));

    //Setup the timer:
  }

  bool MultirotorClientGui::eventFilter(QObject* watched, QEvent* event)
  {
    if (watched == widget_ && event->type() == QEvent::Close)
    {
      ROS_INFO("Closing...");
      event->ignore();
      context_->closePlugin();
      return true;
    }
    return QObject::eventFilter(watched, event);
  }

  void MultirotorClientGui::shutdownPlugin()
  {
  }

  void MultirotorClientGui::on_pushButton_clicked()
  {
    ROS_INFO("Clicked Browse...");
    QString filename = QFileDialog::getOpenFileName(widget_, "Open Trajectory File", "/home","All files (*.*)");
    ui_.fileName->setText(filename);
    //Open the file
  }

}
PLUGINLIB_EXPORT_CLASS(multirotor_manipulation_and_control_framework::MultirotorClientGui, rqt_gui_cpp::Plugin)
