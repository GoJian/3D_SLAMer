# /mapping/keyframe
# Announces a keyframe bundle that is ALREADY persisted to disk.
# Images are not carried here; load them from bundle_path.
# Bundle layout: config/schemas/keyframe_bundle.schema.json

std_msgs/Header header              # stamp: acquisition time mapped to ROS time
string session_id
string rig_id
uint64 keyframe_id
int64 acquisition_time_ns           # original sensor time, not callback arrival time
string clock_domain
string calibration_hash
uint64 pose_revision
geometry_msgs/Pose t_odom_base      # T_odom_base: maps base_link coords into odom
geometry_msgs/Pose t_map_base       # T_map_base: maps base_link coords into map
float32 estimator_quality           # 0..1, estimator-specific meaning documented in adapter
string bundle_path                  # path to persisted bundle manifest
