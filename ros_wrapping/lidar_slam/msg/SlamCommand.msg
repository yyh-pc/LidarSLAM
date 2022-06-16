# The ID of the command to run.
uint8 command

# Optional string argument
string string_arg

# ==============================================================================
# Commands description :

# Run GPS/SLAM calibration from recorded GPS positions and SLAM poses.
# An ICP will match the SLAM trajectory to GPS', trying to estimate the best
# global rigid transform to apply to SLAM trajectory to git GPS trajectory.
# This resulting transform is published on tf2 server as a static TF, linking
# SLAM odometry frame to GPS world/map frame.
uint8 GPS_SLAM_CALIBRATION = 0

# Force SLAM pose to be reset to last acquired GPS pose.
# NOTE: SLAM node use of GPS must be enabled.
# NOTE: Pose graph optimization or GPS/SLAM calibration may have been already
#       executed in order to match GPS and SLAM coordinates systems.
uint8 SET_SLAM_POSE_FROM_GPS = 2

# Disable/enable SLAM maps update with new frames keypoints. This can be useful
# if we want to keep the pose graph optimized SLAM map
# or any initially loaded map untouched
uint8 DISABLE_SLAM_MAP_UPDATE = 8
uint8 ENABLE_SLAM_MAP_EXPANSION = 9
uint8 ENABLE_SLAM_MAP_UPDATE = 10

# Save/Load keypoints maps (edges, planes and blobs if used) to/from disk.
# Use 'string_arg' to indicate path prefix of maps : "/path/to/slam_map_" will
# save/load to "/path/to/slam_map_edges.pcd", "/path/to/slam_map_planes.pcd"
# and "/path/to/slam_map_blobs.pcd".
# WARNING : this process is not real time.
uint8 SAVE_KEYPOINTS_MAPS = 16
uint8 SAVE_FILTERED_KEYPOINTS_MAPS = 17
uint8 LOAD_KEYPOINTS_MAPS = 18

# Stop the slam and optimize pose graph
# The pose graph optimization is done via g2o, fusing SLAM, GPS and or landmarks detections.
# If logging is enabled, SLAM maps will also
# be updated. This allows to correct SLAM drift and to link more precisely
# SLAM odometry frame to GPS world/map frame.
# WARNING : this process is not real time.
uint8 OPTIMIZE_GRAPH = 20
