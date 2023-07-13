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
#include <QFileDialog>
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
  this->SavePcClient = this->Nh.serviceClient<lidar_slam::save_pc>("lidar_slam/save_pc");
}

//----------------------------------------------------------------------------
void SlamControlPanel::CreateLayout()
{
  // Reset the SLAM process
  auto resetStateButton = new QPushButton;
  resetStateButton->setText("Reset state");
  resetStateButton->setToolTip("Clear the map, reset the pose to\n"
                               "initial pose and empty logged info.");
  connect(resetStateButton, &QPushButton::clicked, this, &SlamControlPanel::ResetSlamState);

  // Turn on/off the SLAM process
  auto switchOnOffButton = new QPushButton;
  switchOnOffButton->setText("Switch on/off");
  switchOnOffButton->setToolTip("Disable the SLAM process but keep current state");
  connect(switchOnOffButton, &QPushButton::clicked, this, &SlamControlPanel::SwitchOnOff);

  // Disable the update of the maps, they will remain the same
  auto disableMapUpdateButton = new QPushButton;
  disableMapUpdateButton->setText("Disable map update");
  disableMapUpdateButton->setToolTip("Fix the map to its current state\n"
                                     "new keypoints will not be added.");
  connect(disableMapUpdateButton, &QPushButton::clicked, this, &SlamControlPanel::DisableMapUpdate);

  // Enable the expansion of the maps, keeping the initial one
  auto enableMapExpansionButton = new QPushButton;
  enableMapExpansionButton->setText("Enable map expansion");
  enableMapExpansionButton->setToolTip("Add new keypoints to the SLAM maps as soon as\n"
                                       "they don't replace an initial keypoint (loaded from file)");
  connect(enableMapExpansionButton, &QPushButton::clicked, this, &SlamControlPanel::EnableMapExpansion);

  // Enable the complete update of the maps
  auto enableMapUpdateButton = new QPushButton;
  enableMapUpdateButton->setText("Enable map update");
  enableMapUpdateButton->setToolTip("Add all new keypoints to the SLAM maps");
  connect(enableMapUpdateButton, &QPushButton::clicked, this, &SlamControlPanel::EnableMapUpdate);

  // Save trajectory
  auto saveTrajButton = new QPushButton;
  saveTrajButton->setText("Save trajectory");
  saveTrajButton->setToolTip("This will save the trajectory as a CSV type file\n"
                             "with header <t,x,y,z,x0,y0,z0,x1,y1,z1,x2,y2,z2>\n"
                             "x,y,z being the position and \n"
                             "xi,yi,zi being an axis of the 3D frame \n"
                             "of the trajectory pose (ith column of the rotation).");
  connect(saveTrajButton, &QPushButton::clicked, this, &SlamControlPanel::SaveTraj);

  // Save maps
  auto saveMapsButton = new QPushButton;
  saveMapsButton->setText("Save maps");
  saveMapsButton->setToolTip("Save the maps to the required path.\n"
                             "For example for the input prefix path path/to/map,\n"
                             "one map by keypoint type will be saved as\n"
                             "path/to/map_kptType.pcd.\n"
                             "If the aggregation node has been turned on,\n"
                             "the aggregated map will also\n"
                             "be saved as path/to/map_posixTime.pcd.");
  connect(saveMapsButton, &QPushButton::clicked, this, &SlamControlPanel::SaveMaps);

  // Calibrate with external poses
  auto calibrateButton = new QPushButton;
  calibrateButton->setText("Calibrate");
  calibrateButton->setToolTip("Estimate the calibration between\n"
                              "the frame tracked by the SLAM and\n"
                              "the frame tracked in the input file.\n"
                              "The file must contain the fields\n"
                              "<t,x,y,z,x0,y0,z0,x1,y1,z1,x2,y2,z2>\n"
                              "t being the time, x,y,z being the position and \n"
                              "xi,yi,zi being an axis of the 3D frame \n"
                              "of the trajectory pose (ith column\n"
                              "of the rotation matrix).");
  connect(calibrateButton, &QPushButton::clicked, this, &SlamControlPanel::Calibrate);

  // Create the whole command space
  auto commandLayout = new QVBoxLayout;
  commandLayout->addWidget(resetStateButton);
  commandLayout->addWidget(disableMapUpdateButton);
  commandLayout->addWidget(enableMapExpansionButton);
  commandLayout->addWidget(enableMapUpdateButton);
  commandLayout->addWidget(switchOnOffButton);
  commandLayout->addWidget(saveTrajButton);
  commandLayout->addWidget(saveMapsButton);
  commandLayout->addWidget(calibrateButton);

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
  QString filePath = QFileDialog::getSaveFileName(this, "Create a CSV file", "", "Trajectory Files (*.csv)");
  QFileInfo fileInfo(filePath);
  QString extension = fileInfo.suffix();
  if (extension != "csv")
    filePath = fileInfo.path() + "/" + fileInfo.baseName() + ".csv";
  this->SendCommand(lidar_slam::SlamCommand::SAVE_TRAJECTORY, filePath.toStdString());
}

//----------------------------------------------------------------------------
void SlamControlPanel::SaveMaps()
{
  QString filePath = QFileDialog::getSaveFileName(this, "Create a file prefix", "", "Prefix");
  QFileInfo fileInfo(filePath);
  QString extension = fileInfo.suffix();
  if (extension != "")
    filePath = fileInfo.path() + "/" + fileInfo.baseName();
  // Save SLAM keypoint maps
  this->SendCommand(lidar_slam::SlamCommand::SAVE_FILTERED_KEYPOINTS_MAPS, filePath.toStdString());
  // Save aggregated points if available
  lidar_slam::save_pc srv;
  srv.request.output_prefix_path = filePath.toStdString();
  srv.request.format = 0;
  this->SavePcClient.call(srv);
}

//----------------------------------------------------------------------------
void SlamControlPanel::Calibrate()
{
  QString filePath = QFileDialog::getOpenFileName(this, tr("Open File"), "/home", tr("Trajectory files (*.csv)"));
  this->SendCommand(lidar_slam::SlamCommand::CALIBRATE_WITH_POSES, filePath.toStdString());
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
