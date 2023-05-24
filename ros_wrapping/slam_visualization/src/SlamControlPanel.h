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

#ifndef SLAM_CONTROL_PANEL_H
#define SLAM_CONTROL_PANEL_H

#include <QLabel>
#include <QPushButton>
#include <lidar_slam/Confidence.h>
#include <lidar_slam/SlamCommand.h>
#include <ros/node_handle.h>
#include <ros/publisher.h>
#include <ros/subscriber.h>
#include <rviz/panel.h>

namespace slam_visualization
{

class SlamControlPanel : public rviz::Panel
{
public:
  //----------------------------------------------------------------------------
  /*!
   * @brief     Constructor.
   * @param[in] widget Parent widget.
   */
  explicit SlamControlPanel(QWidget* parent = nullptr);

  ~SlamControlPanel() override = default;

public Q_SLOTS:
  //----------------------------------------------------------------------------
  /*!
   * @brief Send a RESET_STATE command to the slam node.
   */
  void ResetSlamState();

  //----------------------------------------------------------------------------
  /*!
   * @brief Send a DISABLE_MAP_UPDATE command to the slam node.
   */
  void DisableMapUpdate();

  //----------------------------------------------------------------------------
  /*!
   * @brief Send a ENABLE_SLAM_MAP_EXPANSION command to the slam node.
   */
  void EnableMapExpansion();

  //----------------------------------------------------------------------------
  /*!
   * @brief Send a ENABLE_SLAM_MAP_UPDATE command to the slam node.
   */
  void EnableMapUpdate();

  //----------------------------------------------------------------------------
  /*!
   * @brief Send a SWITCH_ON_OFF command to the slam node.
   */
  void SwitchOnOff();

  //----------------------------------------------------------------------------
  /*!
   * @brief Send a SAVE_TRAJECTORY command to the slam node.
   */
  void SaveTraj();

  //----------------------------------------------------------------------------
  /*!
   * @brief Set the path to save the trajectory
   */
  void SetTrajPath(const QString &text);

private:
  //----------------------------------------------------------------------------
  /*!
   * @brief Create and fill the widget layout.
   */
  void CreateLayout();

  //----------------------------------------------------------------------------
  /*!
   * @brief SLAM confidence callback.
   * @param[in] confidence Confidence values.
   */
  void SlamConfidenceCallback(const lidar_slam::ConfidenceConstPtr& confidence);

  //----------------------------------------------------------------------------
  /*!
   * @brief Send a command to the SLAM node.
   * @param[in] command Command type. See the enum in lidar_slam::SlamCommand
   * @param[in] arg Optional command arg.
   */
  void SendCommand(lidar_slam::SlamCommand::_command_type command,
    lidar_slam::SlamCommand::_string_arg_type arg = "");

  //----------------------------------------------------------------------------

  // UI widgets
  QLabel* FailureValueLabel = nullptr;
  QLabel* OverlapValueLabel = nullptr;
  QLabel* ComplyMotionLimitsValueLabel = nullptr;
  QLabel* StdPositionErrorValueLabel = nullptr;
  QLabel* ComputationTimeValueLabel = nullptr;
  QPushButton* SaveTrajButton = nullptr;

  // ROS interface
  ros::NodeHandle Nh;
  ros::Publisher CommandPublisher;
  ros::Subscriber ConfidenceSubscriber;

  // Path storage
  std::string TrajectoryPath;
};

} // namespace lidar_visualization.

#endif // SLAM_CONTROL_PANEL_H
