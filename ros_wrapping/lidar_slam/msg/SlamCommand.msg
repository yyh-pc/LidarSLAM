# The ID of the command to run.
uint8 command

# Commands description :

# Run GPS/SLAM calibration from recorded GPS positions and SLAM poses.
# An ICP will match the SLAM trajectory to GPS, trying to estimate the best
# global rigid transform to apply to SLAM trajectory to git GPS trajectory.
# This resulting transform is published on tf2 server as a static TF, linking
# SLAM coordinates to real world coordinates.
uint8 GPS_SLAM_CALIBRATION = 0

# Run SLAM poses and maps optimization using GPS positions as prior.
# The pose graph optimization is done via g2o, using SLAM and GPS covariances to
# fuse information from both sources. If logging is enabled, SLAM maps will also
# be updated. This allows to precisely link SLAM coordinates to real world
# coordinates and to correct SLAM drift.
# WARNING : this process is not real time.
uint8 GPS_SLAM_POSE_GRAPH_OPTIMIZATION = 2

# Tell SLAM node to listen to next GPS pose, and set SLAM pose estimate with it.
# NOTE: SLAM node use of GPS must be enabled.
# NOTE: Pose graph optimization must be already executed in order to match GPS
#       and SLAM coordinates systems.
uint8 SET_SLAM_POSE_FROM_NEXT_GPS = 4

# Disable/enable SLAM maps update with new frames keypoints. This can be usefull
# if we want to keep the pose graph optimized SLAM map untouched.
uint8 DISABLE_SLAM_MAP_UPDATE = 8
uint8 ENABLE_SLAM_MAP_UPDATE = 9