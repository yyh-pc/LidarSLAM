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
#include <QPushButton>
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
}

//----------------------------------------------------------------------------
void SlamControlPanel::CreateLayout()
{
  auto resetStateButton = new QPushButton;
  resetStateButton->setText("Reset state");
  connect(resetStateButton, &QPushButton::clicked, this, &SlamControlPanel::ResetSlamState);

  auto disableMapUpdateButton = new QPushButton;
  disableMapUpdateButton->setText("Disable map update");
  connect(disableMapUpdateButton, &QPushButton::clicked, this, &SlamControlPanel::DisableMapUpdate);

  auto enableMapExpansionButton = new QPushButton;
  enableMapExpansionButton->setText("Enable map expansion");
  connect(
    enableMapExpansionButton, &QPushButton::clicked, this, &SlamControlPanel::EnableMapExpansion);

  auto enableMapUpdateButton = new QPushButton;
  enableMapUpdateButton->setText("Enable map update");
  connect(enableMapUpdateButton, &QPushButton::clicked, this, &SlamControlPanel::EnableMapUpdate);

  auto commandLayout = new QVBoxLayout;
  commandLayout->addWidget(resetStateButton);
  commandLayout->addWidget(disableMapUpdateButton);
  commandLayout->addWidget(enableMapExpansionButton);
  commandLayout->addWidget(enableMapUpdateButton);

  auto commandBox = new QGroupBox;
  commandBox->setLayout(commandLayout);
  commandBox->setTitle("Commands");

  // Confidence
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
  confidenceLayout->addWidget(overlapLabel, 0, 0, Qt::AlignLeft);
  confidenceLayout->addWidget(this->OverlapValueLabel, 0, 1, Qt::AlignRight);
  confidenceLayout->addWidget(complyMotionLimitsLabel, 1, 0, Qt::AlignLeft);
  confidenceLayout->addWidget(this->ComplyMotionLimitsValueLabel, 1, 1, Qt::AlignRight);
  confidenceLayout->addWidget(stdPositionErrorValueLabel, 2, 0, Qt::AlignLeft);
  confidenceLayout->addWidget(this->StdPositionErrorValueLabel, 2, 1, Qt::AlignRight);
  confidenceLayout->addWidget(computationTimeLabel, 3, 0, Qt::AlignLeft);
  confidenceLayout->addWidget(this->ComputationTimeValueLabel, 3, 1, Qt::AlignRight);

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
  this->SendCommand(lidar_slam::SlamCommand::RESET_SLAM);
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
  this->OverlapValueLabel->setText(
    QString::number(static_cast<int>(confidence->overlap * 100)) + '%');

  QPalette palette = this->ComplyMotionLimitsValueLabel->palette();
  palette.setColor(this->ComplyMotionLimitsValueLabel->foregroundRole(),
    confidence->comply_motion_limits ? Qt::black : Qt::red);

  this->ComplyMotionLimitsValueLabel->setPalette(palette);
  this->ComplyMotionLimitsValueLabel->setText(confidence->comply_motion_limits ? "Yes" : "No");

  this->StdPositionErrorValueLabel->setText(
    QString::number(confidence->std_position_error) + " m");

  this->ComputationTimeValueLabel->setText(
    QString::number(confidence->computation_time * 1000.0) + " ms");
}

} // namespace slam_visualization.

PLUGINLIB_EXPORT_CLASS(slam_visualization::SlamControlPanel, rviz::Panel)
