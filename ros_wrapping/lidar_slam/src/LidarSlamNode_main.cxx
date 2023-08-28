#include "LidarSlamNode.h"

//------------------------------------------------------------------------------
/*!
 * @brief Main node entry point.
 */
int main(int argc, char **argv)
{
  ros::init(argc, argv, "lidar_slam");
  ros::NodeHandle nh;
  ros::NodeHandle priv_nh("~");

  // Create lidar slam node, which subscribes to pointclouds coming from conversion node
  // and to external sensor messages in parallel.
  LidarSlamNode slam(nh, priv_nh);
  // std::cout << " now time: " << ros::Time::now().toSec() << std::endl;
  // Handle callbacks until shut down
  ros::spin();

  return 0;
}

