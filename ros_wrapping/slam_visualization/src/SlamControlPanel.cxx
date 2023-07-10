//==============================================================================
// Copyright 2019-2020 Kitware, Inc., Kitware SAS
// Author: Cadart Nicolas (Kitware SAS)
// Creation date: 2022-06-16
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//==============================================================================

#include "SlamControlPanel.h"

#include <QGridLayout>
#include <QGroupBox>
#include <QLineEdit>
#include <QVBoxLayout>
#include <pluginlib/class_list_macros.hpp>

namespace slam_visualization
{

//----------------------------------------------------------------------------
SlamControlPanel::SlamControlPanel(QWidget* parent)
  : rviz::Panel{ parent }
{
  CreateLayout();

  this->CommandPublisher = this->Nh.advertise<lidar_slam::SlamCommand>("slam_command", 1);
  this->ConfidenceSubscriber =
    this->Nh.subscribe("slam_confidence", 5, &SlamControlPanel::SlamConfidenceCallback, this);
  this->ResetClient = this->Nh.serviceClient<lidar_slam::reset>("lidar_slam/reset");
}

//----------------------------------------------------------------------------
void SlamControlPanel::CreateLayout()
{
  // Reset the SLAM process
  auto resetStateButton = new QPushButton;
  resetStateButton->setText("Reset state");
  connect(resetStateButton, &QPushButton::clicked, this, &SlamControlPanel::ResetSlamState);

  // Disable the update of the maps, they will remain the same
  auto disableMapUpdateButton = new QPushButton;
  disableMapUpdateButton->setText("Disable map update");
  connect(disableMapUpdateButton, &QPushButton::clicked, this, &SlamControlPanel::DisableMapUpdate);

  // Enable the expansion of the maps, keeping the initial one
  auto enableMapExpansionButton = new QPushButton;
  enableMapExpansionButton->setText("Enable map expansion");
  connect(enableMapExpansionButton, &QPushButton::clicked, this, &SlamControlPanel::EnableMapExpansion);

  // Enable the complete update of the maps
  auto enableMapUpdateButton = new QPushButton;
  enableMapUpdateButton->setText("Enable map update");
  connect(enableMapUpdateButton, &QPushButton::clicked, this, &SlamControlPanel::EnableMapUpdate);

  // Turn on/off the SLAM process
  auto switchOnOffButton = new QPushButton;
  switchOnOffButton->setText("Switch on/off");
  connect(switchOnOffButton, &QPushButton::clicked, this, &SlamControlPanel::SwitchOnOff);

  // Save trajectory
  this->SaveTrajButton = new QPushButton;
  this->SaveTrajButton->setText("Save trajectory");
  this->SaveTrajButton->setDisabled(true);
  connect(this->SaveTrajButton, &QPushButton::clicked, this, &SlamControlPanel::SaveTraj);

  auto trajPathEdit = new QLineEdit;
  connect(trajPathEdit, &QLineEdit::textChanged, this, &SlamControlPanel::SetTrajPath);

  // Create trajectory space with a button and a line edit to set a path
  auto trajLayout = new QHBoxLayout;
  trajLayout->addWidget(this->SaveTrajButton);
  trajLayout->addWidget(trajPathEdit);

  // Save maps
  this->SaveMapsButton = new QPushButton;
  this->SaveMapsButton->setText("Save maps");
  this->SaveMapsButton->setDisabled(true);
  connect(this->SaveMapsButton, &QPushButton::clicked, this, &SlamControlPanel::SaveMaps);

  auto mapsPathEdit = new QLineEdit;
  connect(mapsPathEdit, &QLineEdit::textChanged, this, &SlamControlPanel::SetMapsPath);

  // Create maps space with a button and a line edit to set a path
  auto mapsLayout = new QHBoxLayout;
  mapsLayout->addWidget(this->SaveMapsButton);
  mapsLayout->addWidget(mapsPathEdit);

  // Calibrate with external poses
  this->CalibrateButton = new QPushButton;
  this->CalibrateButton->setText("Calibrate");
  this->CalibrateButton->setDisabled(true);
  connect(this->CalibrateButton, &QPushButton::clicked, this, &SlamControlPanel::Calibrate);

  auto posesPathEdit = new QLineEdit;
  connect(posesPathEdit, &QLineEdit::textChanged, this, &SlamControlPanel::SetPosesPath);

  // Create poses space with a button and a line edit to set a path
  auto calibrateLayout = new QHBoxLayout;
  calibrateLayout->addWidget(this->CalibrateButton);
  calibrateLayout->addWidget(posesPathEdit);

  // Create the whole command space
  auto commandLayout = new QVBoxLayout;
  commandLayout->addWidget(resetStateButton);
  commandLayout->addWidget(disableMapUpdateButton);
  commandLayout->addWidget(enableMapExpansionButton);
  commandLayout->addWidget(enableMapUpdateButton);
  commandLayout->addWidget(switchOnOffButton);
  commandLayout->addLayout(trajLayout);
  commandLayout->addLayout(mapsLayout);
  commandLayout->addLayout(calibrateLayout);

  auto commandBox = new QGroupBox;
  commandBox->setLayout(commandLayout);
  commandBox->setTitle("Commands");

  // Confidence
  auto failureLabel = new QLabel{ "SLAM has failed: " };
  this->FailureValueLabel = new QLabel;
  this->FailureValueLabel->setTextFormat(Qt::TextFormat::PlainText);
  failureLabel->setBuddy(this->FailureValueLabel);

  auto overlapLabel = new QLabel{ "Overlap:" };
  this->OverlapValueLabel = new QLabel;
  this->OverlapValueLabel->setTextFormat(Qt::TextFormat::PlainText);
  overlapLabel->setBuddy(overlapLabel);

  auto complyMotionLimitsLabel = new QLabel{ "Comply motion limits: " };
  this->ComplyMotionLimitsValueLabel = new QLabel;
  this->ComplyMotionLimitsValueLabel->setTextFormat(Qt::TextFormat::PlainText);
  complyMotionLimitsLabel->setBuddy(this->ComplyMotionLimitsValueLabel);

  auto stdPositionErrorValueLabel = new QLabel{ "Position error estimated: " };
  this->StdPositionErrorValueLabel = new QLabel;
  this->StdPositionErrorValueLabel->setTextFormat(Qt::TextFormat::PlainText);
  stdPositionErrorValueLabel->setBuddy(this->ComplyMotionLimitsValueLabel);

  auto computationTimeLabel = new QLabel{ "Frame computation time: " };
  this->ComputationTimeValueLabel = new QLabel;
  this->ComputationTimeValueLabel->setTextFormat(Qt::TextFormat::PlainText);
  computationTimeLabel->setBuddy(this->ComputationTimeValueLabel);

  auto confidenceLayout = new QGridLayout;
  confidenceLayout->addWidget(failureLabel, 1, 0, Qt::AlignLeft);
  confidenceLayout->addWidget(this->FailureValueLabel, 1, 1, Qt::AlignRight);
  confidenceLayout->addWidget(overlapLabel, 2,0, Qt::AlignLeft);
  confidenceLayout->addWidget(this->OverlapValueLabel, 2, 1, Qt::AlignRight);
  confidenceLayout->addWidget(complyMotionLimitsLabel, 3, 0, Qt::AlignLeft);
  confidenceLayout->addWidget(this->ComplyMotionLimitsValueLabel, 3, 1, Qt::AlignRight);
  confidenceLayout->addWidget(stdPositionErrorValueLabel, 4, 0, Qt::AlignLeft);
  confidenceLayout->addWidget(this->StdPositionErrorValueLabel, 4, 1, Qt::AlignRight);
  confidenceLayout->addWidget(computationTimeLabel, 5, 0, Qt::AlignLeft);
  confidenceLayout->addWidget(this->ComputationTimeValueLabel, 5, 1, Qt::AlignRight);

  auto confidenceBox = new QGroupBox;
  confidenceBox->setTitle("Confidence estimator");
  confidenceBox->setLayout(confidenceLayout);

  auto mainLayout = new QVBoxLayout;
  mainLayout->addWidget(commandBox);
  mainLayout->addWidget(confidenceBox);

  setLayout(mainLayout);
}

//----------------------------------------------------------------------------
void SlamControlPanel::ResetSlamState()
{
  // Send command to reset the SLAM algorithm
  this->SendCommand(lidar_slam::SlamCommand::RESET_SLAM);
  // Call service for aggregation node
  lidar_slam::reset srv;
  this->ResetClient.call(srv);
}

//----------------------------------------------------------------------------
void SlamControlPanel::DisableMapUpdate()
{
  this->SendCommand(lidar_slam::SlamCommand::DISABLE_SLAM_MAP_UPDATE);
}

//----------------------------------------------------------------------------
void SlamControlPanel::EnableMapExpansion()
{
  this->SendCommand(lidar_slam::SlamCommand::ENABLE_SLAM_MAP_EXPANSION);
}

//----------------------------------------------------------------------------
void SlamControlPanel::EnableMapUpdate()
{
  this->SendCommand(lidar_slam::SlamCommand::ENABLE_SLAM_MAP_UPDATE);
}

//----------------------------------------------------------------------------
void SlamControlPanel::SwitchOnOff()
{
  this->SendCommand(lidar_slam::SlamCommand::SWITCH_ON_OFF);
}

//----------------------------------------------------------------------------
void SlamControlPanel::SaveTraj()
{
  this->SendCommand(lidar_slam::SlamCommand::SAVE_TRAJECTORY, this->TrajectoryPath);
}

//----------------------------------------------------------------------------
void SlamControlPanel::SetTrajPath(const QString &text)
{
  this->SaveTrajButton->setDisabled(false);
  this->TrajectoryPath = text.toStdString();
}

//----------------------------------------------------------------------------
void SlamControlPanel::SaveMaps()
{
  this->SendCommand(lidar_slam::SlamCommand::SAVE_FILTERED_KEYPOINTS_MAPS, this->MapsPath);
}

//----------------------------------------------------------------------------
void SlamControlPanel::SetMapsPath(const QString &text)
{
  this->SaveMapsButton->setDisabled(false);
  this->MapsPath = text.toStdString();
}

//----------------------------------------------------------------------------
void SlamControlPanel::Calibrate()
{
  this->SendCommand(lidar_slam::SlamCommand::CALIBRATE_WITH_POSES, this->PosesPath);
}

//----------------------------------------------------------------------------
void SlamControlPanel::SetPosesPath(const QString &text)
{
  this->CalibrateButton->setDisabled(false);
  this->PosesPath = text.toStdString();
}

//----------------------------------------------------------------------------
void SlamControlPanel::SendCommand(
  lidar_slam::SlamCommand::_command_type command, lidar_slam::SlamCommand::_string_arg_type arg)
{
  lidar_slam::SlamCommand msg;
  msg.command = command;
  msg.string_arg = std::move(arg);
  this->CommandPublisher.publish(msg);
}

//----------------------------------------------------------------------------
void SlamControlPanel::SlamConfidenceCallback(const lidar_slam::ConfidenceConstPtr& confidence)
{
  QPalette palette1 = this->FailureValueLabel->palette();
  palette1.setColor(this->FailureValueLabel->foregroundRole(),
    confidence->failure ? Qt::red : Qt::black);

  this->FailureValueLabel->setPalette(palette1);
  this->FailureValueLabel->setText(confidence->failure ? "Yes" : "No");

  this->OverlapValueLabel->setText(
    QString::number(static_cast<int>(confidence->overlap * 100)) + '%');

  QPalette palette2 = this->ComplyMotionLimitsValueLabel->palette();
  palette2.setColor(this->ComplyMotionLimitsValueLabel->foregroundRole(),
    confidence->comply_motion_limits ? Qt::black : Qt::red);

  this->ComplyMotionLimitsValueLabel->setPalette(palette2);
  this->ComplyMotionLimitsValueLabel->setText(confidence->comply_motion_limits ? "Yes" : "No");

  this->StdPositionErrorValueLabel->setText(
    QString::number(confidence->std_position_error) + " m");

  this->ComputationTimeValueLabel->setText(
    QString::number(confidence->computation_time * 1000.0) + " ms");
}

} // namespace slam_visualization.

PLUGINLIB_EXPORT_CLASS(slam_visualization::SlamControlPanel, rviz::Panel)
