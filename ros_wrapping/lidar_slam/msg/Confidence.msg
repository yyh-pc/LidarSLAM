# The covariance in this message should be specified in the coordinate frame given by header.frame_id.

# See "std_msgs/Header.msg"
Header header

# Row-major representation of the 6x6 covariance matrix of ICP pose estimation.
# In order, the pose elements are:
# (x, y, z, rotation angle about X axis, rotation angle about Y axis, rotation angle about Z axis)
# Rotation is computed as Rz*Ry*Rx
float64[36] covariance

# Overlap estimation
# It is contained in [0, 1]
# Good values are greater than 0.6
# Bad values are lower than 0.4
# Jumps in this value can reflect pose estimation failure
float32 overlap

# Number of matches between extracted keypoints
# and current keypoint maps
# Good values are greater than 200
# Bad values are lower than 50
int16 nb_matches

# This value indicates if the estimated pose
# respects the specified physical constraints on velocity and acceleration
bool comply_motion_limits

# This value indicates the computation time from data reception to pose output
float32 computation_time
