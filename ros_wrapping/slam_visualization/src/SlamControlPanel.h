#pragma once

#include <QLabel>
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
  QLabel* OverlapValueLabel = nullptr;
  QLabel* ComplyMotionLimitsValueLabel = nullptr;
  QLabel* ComputationTimeValueLabel = nullptr;

  // ROS interface
  ros::NodeHandle Nh;
  ros::Publisher CommandPublisher;
  ros::Subscriber ConfidenceSubscriber;
};

} // namespace lidar_visualization.
